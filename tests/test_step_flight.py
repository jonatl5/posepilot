import io
import json

import pytest

from objectpilot import stepsim
from objectpilot.ground_link import GroundLink
from objectpilot.step_flight import (TRIALS, Abort, CalibrationFlight, FlightLink, FlightPlan, Recorder,
                                     SimAircraft, analyze, load_log, main, permitted_flight)

QUIET = dict(echo=lambda *args, **kwargs: None)
MOVES = ("forward", "back", "left", "right", "up", "down", "cw", "ccw")


class FakeAircraft:
    """Scripted aircraft: the same observation every time; records every command."""

    def __init__(self, tof=1.02, box=(.5, .5, .2, .34), battery="90", fail_once=(), observe_error_after=None,
                 airborne=False):
        self.sent, self.tof, self.box, self.battery, self.airborne = [], tof, box, battery, airborne
        self.fail_once, self.observe_error_after, self.observations = set(fail_once), observe_error_after, 0

    def send(self, command, timeout):
        assert permitted_flight(command), command
        self.sent.append(command)
        if command in self.fail_once:
            self.fail_once.discard(command)
            return "error"
        return {"battery?": self.battery, "sdk?": "30"}.get(command, "ok")

    def start(self):
        pass

    def wait(self, seconds):
        pass

    def observe(self, n):
        self.observations += 1
        if self.observe_error_after and self.observations > self.observe_error_after:
            raise RuntimeError("video stopped")
        airborne = self.airborne or "takeoff" in self.sent
        return dict(frames=[[("cup", *self.box, .9)] if self.box else [] for _ in range(n)], tof=[self.tof]*n,
                    tof_answered=n, state=dict(yaw=0., tof=160. if airborne else 10., h=160. if airborne else 0.,
                                               bat=90., age_s=.05))


def flight(aircraft, tmp_path, plan=None, clock=None):
    plan = plan or FlightPlan()
    recorder = Recorder(tmp_path/"log", clock or (lambda: 0.))
    return CalibrationFlight(aircraft, plan, recorder, clock=clock or (lambda: 0.), **QUIET), recorder


def test_flight_allowlist_permits_bounded_steps_only():
    for command in ("takeoff", "land", "stop", "speed 30", "forward 20", "left 100", "cw 90", "EXT tof?",
                    "battery?", "mon", "mdirection 0"):
        assert permitted_flight(command), command
        assert FlightLink.__new__(FlightLink).permitted(command)
    for command in ("rc 0 20 0 0", "go 20 0 0 10", "emergency", "flip l", "motoron", "throwfly", "forward 150",
                    "cw 180", "forward 19", "speed 5", "land 20", "takeoff\nforward 20", "command\nemergency"):
        assert not permitted_flight(command), command
    ground = GroundLink.__new__(GroundLink)
    assert not any(ground.permitted(c) for c in ("takeoff", "land", "stop", "forward 20", "cw 10", "speed 30"))


def test_operator_must_confirm_before_takeoff(tmp_path):
    aircraft = FakeAircraft()
    f, recorder = flight(aircraft, tmp_path)
    assert f.run(confirm=lambda text: False) == "not_confirmed"
    recorder.close()
    assert "takeoff" not in aircraft.sent and "land" not in aircraft.sent


def test_low_battery_stops_before_takeoff(tmp_path):
    aircraft = FakeAircraft(battery="35")
    f, recorder = flight(aircraft, tmp_path)
    assert f.run(confirm=lambda text: True) == "battery_35"
    recorder.close()
    assert "takeoff" not in aircraft.sent


def test_close_wall_backs_off_and_lands(tmp_path):
    aircraft = FakeAircraft(tof=.52)                       # 0.50 m after the 2 cm bench offset
    f, recorder = flight(aircraft, tmp_path)
    assert f.run(confirm=lambda text: True) == "too_close"
    recorder.close()
    assert aircraft.sent[-3:] == ["back 20", "stop", "land"] and not f.airborne


def test_forward_step_that_would_end_too_close_is_refused(tmp_path):
    aircraft = FakeAircraft()
    f, recorder = flight(aircraft, tmp_path)
    f.last = dict(tof_m=.70)
    with pytest.raises(Abort) as stop:
        f.send("forward 20", "trial")
    recorder.close()
    assert stop.value.reason == "forward_blocked" and aircraft.sent == []


def test_unexpected_exception_still_lands(tmp_path):
    aircraft = FakeAircraft(observe_error_after=2)
    f, recorder = flight(aircraft, tmp_path)
    assert f.run(confirm=lambda text: True).startswith("exception RuntimeError")
    recorder.close()
    assert aircraft.sent[-1] == "land" and not f.airborne


def test_failed_command_is_observed_and_sent_once_more(tmp_path):
    aircraft = FakeAircraft(fail_once={"left 20"}, airborne=True)
    f, recorder = flight(aircraft, tmp_path)
    f.observe("start")
    f.step("left 20", "trial")
    recorder.close()
    assert aircraft.sent == ["left 20", "left 20"]
    assert [r["tag"] for r in load_log(recorder.path) if r["event"] == "observe"] == ["start", "trial", "trial"]


def test_ctrl_c_mid_flight_stops_and_lands(tmp_path):
    class Interrupted(SimAircraft):
        def send(self, command, timeout):
            self.sent = getattr(self, "sent", [])+[command]
            if sum(c.split()[0] in MOVES for c in self.sent) == 12 and command.split()[0] in MOVES:
                raise KeyboardInterrupt           # once, while the twelfth step command is in flight
            return super().send(command, timeout)

    plan = FlightPlan()
    aircraft = Interrupted(60601, plan)
    f, recorder = flight(aircraft, tmp_path, plan, aircraft.clock)
    assert f.run(confirm=lambda text: True) == "operator_interrupt"
    recorder.close()
    assert aircraft.sent[-2:] == ["stop", "land"] and not aircraft.airborne


def test_rehearsal_completes_and_analysis_recovers_the_test_world(tmp_path):
    plan = FlightPlan()
    aircraft = SimAircraft(60600, plan)
    f, recorder = flight(aircraft, tmp_path, plan, aircraft.clock)
    assert f.run(confirm=lambda text: True) == "completed"
    recorder.close()
    rows = load_log(recorder.path)
    trials = {r["command"] for r in rows if r["event"] == "command" and r["kind"] == "trial"}
    assert {c for first, second, _ in TRIALS for c in (first, second)} <= trials
    assert min(-r["truth"]["x"] for r in rows if r["event"] == "observe") >= .55
    a = analyze(rows)
    assert a["state_yaw_increases_with_cw"]
    assert all(s["as_expected"] for s in a["image_signs"].values()) and len(a["image_signs"]) == 8
    assert 50 <= a["hfov_from_turns_deg"]["median"] <= 58           # the test world uses 54 degrees
    assert abs(a["sweep"]["normal_offset_deg"]-a["sweep"]["true_normal_offset_deg"]) <= 3
    assert .2 <= a["noise_for_stepsim"]["tof_oblique_excess"] <= .5     # the test world uses 0.35
    for verb in ("forward", "back", "left", "right"):
        assert .85 <= a["move_scale"][verb]["median"] <= 1.15


def test_cli_rehearsal_feeds_measured_noise_into_stepsim(tmp_path):
    assert main(["rehearse", "--out", str(tmp_path/"rehearsal"), "--seed", "60602", "--repeats", "1"]) == 0
    noise_path = tmp_path/"rehearsal"/"noise.json"
    noise = json.loads(noise_path.read_text(encoding="utf-8"))
    assert {"move_scale_sd", "move_cross_sd", "yaw_sd_deg", "move_heading_sd_deg"} <= set(noise)
    assert (tmp_path/"rehearsal"/"analysis.json").exists()
    stepsim.main(["--episodes", "2", "--seed", "60000", "--variants", "square", "--noise", str(noise_path),
                  "--out", str(tmp_path/"sim")])
    summary = json.loads((tmp_path/"sim"/"summary.json").read_text(encoding="utf-8"))
    assert all(summary["noise"][k] == v for k, v in noise.items())
    assert main(["analyze", str(tmp_path/"rehearsal"/"flight.jsonl")]) == 0


def test_cli_refuses_unsafe_or_unattended_use(tmp_path, monkeypatch):
    with pytest.raises(SystemExit):
        main(["rehearse", "--out", str(tmp_path/"a"), "--seed", "12345"])
    with pytest.raises(SystemExit):
        main(["rehearse", "--out", str(tmp_path/"b"), "--hover-m", "1.4"])
    monkeypatch.setattr("sys.stdin", io.StringIO(""))
    with pytest.raises(SystemExit):
        main(["fly", "--out", str(tmp_path/"c")])
    assert not (tmp_path/"c").exists()
