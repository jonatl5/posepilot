import io
import json

import pytest

from objectpilot.step_approach import (ApproachFlight, MemoryRecorder, approach_plan, flight_summary, main,
                                       simulate_episode)
from objectpilot.step_flight import SimAircraft, load_log, permitted_flight
from objectpilot.stepwise import Decision


def quiet(*args, **kwargs):
    pass


def sim_flight(pose, plan=None, aircraft_class=SimAircraft, seed=61000):
    plan = plan or approach_plan()
    aircraft = aircraft_class(seed, plan, pose=pose)
    recorder = MemoryRecorder(aircraft.clock)
    return ApproachFlight(aircraft, plan, recorder, clock=aircraft.clock, echo=quiet), aircraft, recorder


def test_simulated_oblique_starts_arrive_square_through_the_full_flight_loop(tmp_path):
    for seed in (60002, 60024):                 # 26 and 49 degrees off the print's normal in stepsim.scenario
        row, flight, recorder = simulate_episode(seed, approach_plan(), log_dir=tmp_path/str(seed))
        assert row["outcome"] == "arrived" and row["success"] and row["squared"], row
        rows = load_log(recorder.path)
        commands = [r["command"] for r in rows if r["event"] == "command" and r["kind"] != "preflight"]
        assert commands[0] == "takeoff" and commands[1].startswith("up ") and commands[-1] == "land"
        assert all(permitted_flight(c) for c in commands)
        summary = flight_summary(rows)
        assert summary["planner_outcome"] == "arrived" and "sweep" in summary["commands_by_phase"]
        assert summary["min_tof_m"] >= .42


def test_planner_command_outside_the_allowlist_aborts_and_lands():
    flight, aircraft, recorder = sim_flight((-1.5, 0., 0.))
    flight.planner.decide = lambda obs: Decision("cw 180", "face", "test")
    assert flight.run(confirm=lambda text: True) == "planner_command_refused: cw 180"
    sent = [r["command"] for r in recorder.rows if r["event"] == "command"]
    assert "cw 180" not in sent and sent[-1] == "land" and not aircraft.airborne


def test_flight_layer_aborts_inside_its_own_minimum_range():
    flight, aircraft, recorder = sim_flight((-.30, 0., 0.))            # inside the 0.32 m abort range
    assert flight.run(confirm=lambda text: True) == "too_close"
    sent = [r["command"] for r in recorder.rows if r["event"] == "command"]
    assert sent[-2:] == ["back 20", "land"]


def test_long_planner_waits_send_a_keepalive():
    flight, aircraft, recorder = sim_flight((-1.5, 0., 0.))
    waits = iter([Decision(None, "final", "arrival_hold")]*20+[Decision(None, "final", "arrived", "arrived")])
    flight.planner.decide = lambda obs: next(waits)
    assert flight.run(confirm=lambda text: True) == "arrived"
    airborne = [r for r in recorder.rows if r["event"] == "command" and r["t"] > 0 and r["command"] == "battery?"]
    assert airborne, "no keepalive during 20 planner waits"


def test_ctrl_c_during_the_planner_loop_lands():
    class Interrupted(SimAircraft):
        moves = 0

        def send(self, command, timeout):
            if command.split()[0] in ("forward", "back", "left", "right", "cw", "ccw"):
                Interrupted.moves += 1
                if Interrupted.moves == 5:
                    raise KeyboardInterrupt
            return super().send(command, timeout)

    flight, aircraft, recorder = sim_flight((-2.2, .3, .2), aircraft_class=Interrupted)
    assert flight.run(confirm=lambda text: True) == "operator_interrupt"
    assert [r["command"] for r in recorder.rows if r["event"] == "command"][-1] == "land"
    assert not aircraft.airborne


def test_cli_simulates_writes_summaries_and_refuses_unsafe_use(tmp_path, monkeypatch):
    out = tmp_path/"sim"
    assert main(["simulate", "--episodes", "3", "--seed", "60000", "--keep-logs", "1",
                 "--noise", "configs/tt_step_noise_measured.json", "--out", str(out)]) == 0
    summary = json.loads((out/"summary.json").read_text(encoding="utf-8"))
    measured = json.loads(open("configs/tt_step_noise_measured.json", encoding="utf-8").read())
    assert summary["results"]["episodes"] == 3 and summary["noise"]["move_scale_sd"] == measured["move_scale_sd"]
    assert (out/"episodes.csv").exists() and (out/"episode-60000"/"summary.json").exists()
    assert main(["summary", str(out/"episode-60000"/"flight.jsonl")]) == 0
    with pytest.raises(SystemExit):
        main(["simulate", "--episodes", "3", "--out", str(out)])               # existing folder
    with pytest.raises(SystemExit):
        main(["simulate", "--episodes", "3", "--seed", "59999", "--out", str(tmp_path/"reserved")])
    monkeypatch.setattr("sys.stdin", io.StringIO(""))
    with pytest.raises(SystemExit):
        main(["fly", "--out", str(tmp_path/"fly")])
    assert not (tmp_path/"fly").exists()


def test_refused_takeoff_sends_no_landing():
    class Hot(SimAircraft):
        def send(self, command, timeout):
            return "error, high temp" if command == "takeoff" else super().send(command, timeout)

    flight, aircraft, recorder = sim_flight((-1.5, 0., 0.), aircraft_class=Hot)
    assert flight.run(confirm=lambda text: True) == "takeoff_refused: error, high temp"
    assert "land" not in [r["command"] for r in recorder.rows if r["event"] == "command"]
