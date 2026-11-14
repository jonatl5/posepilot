"""Planner-driven approach: objectpilot.stepwise.StepPlanner flies the aircraft to a print.

THIS MODULE FLIES THE AIRCRAFT in `fly` mode. It uses objectpilot.step_flight's guarded
flight session unchanged: command allowlist, pre-flight checks, typed operator
confirmation, takeoff and climb, abort-and-land. After the climb, StepPlanner takes
over: each hover gives one observation (6 frames, 6 ToF replies, the state-stream yaw),
the planner chooses one SDK step command, the aircraft executes it and stops, and the
loop repeats until the planner reports an outcome. Then the aircraft lands.

    python -m objectpilot.step_approach simulate --episodes 300 --noise configs/tt_step_noise_measured.json --out runs/approach-sim
    python -m objectpilot.step_approach fly --class-name cup --weights models/yolo11s.pt --out runs/approach-01
    python -m objectpilot.step_approach summary runs/approach-01/flight.jsonl

`simulate` runs the same loop, guards included, against objectpilot.stepsim's geometric
test world from stepsim.scenario starts, so its numbers compare directly with stepsim's.
Procedure and results: research/STEP_APPROACH.md.
"""
from collections import Counter
from dataclasses import asdict, replace
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

from .step_flight import (Abort, FlightPlan, FlightSession, Recorder, SimAircraft, TelloAircraft, load_log,
                          permitted_flight, terminal_confirm)
from .stepwise import StepConfig, StepObservation, StepPlanner

KEEPALIVE_S = 8.        # the aircraft lands by itself after 15 s without a command
STAND_OFF_M = .50       # November 13: moved in from 0.65 m on request
MINIMUM_M = .35         # planner minimum; it backs off below 0.40 and the flight layer aborts below 0.32
CAMERA_TILT_DEG = 10.5


def approach_config(**fields):
    """StepPlanner settings for the approach flight."""
    return replace(StepConfig(), **{"stand_off_m": STAND_OFF_M, "minimum_m": MINIMUM_M, **fields})


def approach_plan(**fields):
    """Flight-layer limits for the approach: abort 3 cm inside the planner's own minimum."""
    return replace(FlightPlan(), **{"min_tof_m": MINIMUM_M-.03, "max_seconds": 240., **fields})


def print_center_cm(height_cm, stand_off_m):
    """Print center height that sits on the image center at the stand-off (camera tilted down)."""
    return height_cm-100*stand_off_m*math.tan(math.radians(CAMERA_TILT_DEG))


def step_observation(row):
    box = tuple(row["box"]) if row["box"] is not None else None
    return StepObservation(box, row["seen"], row["tof_m"], row["yaw"], row.get("conf"))


class MemoryRecorder(Recorder):
    """Recorder that keeps rows in memory: simulated episodes whose logs are not written."""

    def __init__(self, clock=time.monotonic):
        self.rows, self.clock, self.t0, self.save_frames, self.path = [], clock, clock(), False, None

    def write(self, row, images=()):
        row = dict(t=round(self.clock()-self.t0, 2), **row)
        self.rows.append(row)
        return row

    def close(self):
        pass


class ApproachFlight(FlightSession):
    """Guarded flight whose mission is the StepPlanner loop."""

    def __init__(self, aircraft, plan, recorder, config=None, clock=time.monotonic, echo=print):
        super().__init__(aircraft, plan, recorder, clock, echo)
        self.config = replace(config or approach_config(), label=plan.label)
        self.planner = StepPlanner(self.config)
        self.last_command_at = None

    def checklist(self):
        p, c = self.p, self.config
        return f"""
PLANNER APPROACH FLIGHT: StepPlanner flies the aircraft to the {p.label} print (at most {p.max_seconds/60:.0f} minutes).
  [ ] Propeller guards on; nobody within 2 m of the flight path except the operator
  [ ] Floor lit and textured under the whole path (downward positioning)
  [ ] Aircraft on the floor 1.5-3 m from the wall, within about 45 degrees of the print's normal (any heading)
  [ ] Nothing between the aircraft and the wall; 1 m clear around the path; ceiling above {p.height_cm+60:.0f} cm
  [ ] Print center about {print_center_cm(p.height_cm, c.stand_off_m):.0f} cm above the floor (flight height {p.height_cm:.0f} cm by downward ToF)
  [ ] Battery at least {p.min_battery_takeoff}%; your hand on Ctrl+C (stop and land)
The planner stops {c.stand_off_m:.2f} m from the wall; the aircraft lands after the planner reports an outcome.
If this program freezes, the aircraft lands by itself 15 s after its last command.
Type "fly" to take off, anything else to cancel: """

    def send(self, command, kind, **extra):
        reply = super().send(command, kind, **extra)
        self.last_command_at = self.clock()
        return reply

    def mission(self):
        obs = self.last
        while True:
            decision = self.planner.decide(step_observation(obs))
            self.rec.write(dict(event="decision", phase=decision.phase, reason=decision.reason,
                                command=decision.command, outcome=decision.outcome))
            if decision.outcome:
                self.note(f"planner outcome: {decision.outcome} after {self.planner.commands} commands")
                return decision.outcome
            if decision.command is None:
                self.echo(f"{'':10}{decision.phase}: wait ({decision.reason})")
                if self.last_command_at is not None and self.clock()-self.last_command_at > KEEPALIVE_S:
                    self.query("battery?")
                    self.last_command_at = self.clock()
                obs = self.observe(f"wait:{decision.reason}")
                self.guard(obs)
                continue
            if not permitted_flight(decision.command):
                raise Abort(f"planner_command_refused: {decision.command}")
            reply, obs = self.move(decision.command, decision.phase, reason=decision.reason)
            self.planner.applied(decision.command, reply)


# -- summaries -------------------------------------------------------------------------------------
def flight_summary(rows):
    """Outcome, planner commands by phase and reason, ranges and the final observation of one flight."""
    end = next((r for r in reversed(rows) if r["event"] == "end"), {})
    decisions = [r for r in rows if r["event"] == "decision"]
    planned = [r for r in rows if r["event"] == "command" and "reason" in r]
    takeoff = next((r["t"] for r in rows if r["event"] == "command" and r["command"] == "takeoff"), None)
    airborne = [r for r in rows if r["event"] == "observe" and r["tag"] != "ground"]
    ranges = [r["tof_m"] for r in airborne if r["tof_m"] is not None]
    final = airborne[-1] if airborne else None
    summary = dict(outcome=end.get("outcome"), planner_outcome=next((d["outcome"] for d in reversed(decisions)
                                                                     if d["outcome"]), None),
                   planner_commands=len(planned), errors=sum(r["reply"] != "ok" for r in planned),
                   seconds_after_takeoff=None if takeoff is None else round(end.get("t", takeoff)-takeoff, 1),
                   commands_by_phase=dict(Counter(r["kind"] for r in planned)),
                   commands_by_reason=dict(Counter(r["reason"] for r in planned)),
                   waits=dict(Counter(d["reason"] for d in decisions if d["command"] is None and not d["outcome"])),
                   min_tof_m=min(ranges) if ranges else None,
                   final=None if final is None else dict(box=final["box"], tof_m=final["tof_m"], yaw=final["yaw"]),
                   steps=[[r["kind"], r["reason"], r["command"], r["reply"]] for r in planned])
    if final is not None and "truth" in final:
        summary["final"]["truth"] = final["truth"]
    return summary


def print_summary(summary, echo=print):
    echo(f"outcome {summary['outcome']} (planner: {summary['planner_outcome']}), "
         f"{summary['planner_commands']} planner commands, {summary['seconds_after_takeoff']} s after takeoff")
    echo(f"commands by phase: {summary['commands_by_phase']}")
    echo(f"waits: {summary['waits']}; closest ToF {summary['min_tof_m']}; final {summary['final']}")


# -- simulation ------------------------------------------------------------------------------------
def simulate_episode(seed, plan, config=None, noise=None, log_dir=None):
    """One stepsim.scenario start through the full flight loop; true final pose scored as in stepsim."""
    from .stepsim import final_metrics, scenario
    s = scenario(seed)
    plan = replace(plan, label=s["label"])
    x, y, _, heading = s["pose"]
    aircraft = SimAircraft(seed+1_000_000, plan, noise, pose=(x, y, heading))
    recorder = Recorder(log_dir, aircraft.clock, save_frames=False) if log_dir else MemoryRecorder(aircraft.clock)
    flight = ApproachFlight(aircraft, plan, recorder, config, clock=aircraft.clock, echo=lambda *a, **k: None)
    outcome = flight.run(confirm=lambda text: True)
    recorder.close()
    row = dict(seed=seed, label=s["label"], start_distance_m=round(s["distance_m"], 3),
               start_view_deg=round(s["view_deg"], 1), start_heading_offset_deg=round(s["heading_offset_deg"], 1),
               outcome=outcome, commands=flight.planner.commands, observations=flight.observations,
               time_s=round(aircraft.world.time, 1), squared=flight.planner.squared,
               **final_metrics(aircraft.world, aircraft.target, outcome, flight.config.stand_off_m,
                               flight.config.minimum_m))
    return row, flight, recorder


def simulate(seeds, out, plan=None, config=None, noise=None, keep_logs=5, echo=print):
    from .stepsim import summarize_results
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    plan = plan or approach_plan()
    rows = []
    for k, seed in enumerate(seeds):
        log_dir = out/f"episode-{seed}" if k < keep_logs else None
        row, _, recorder = simulate_episode(seed, plan, config, noise, log_dir)
        if log_dir:
            (log_dir/"summary.json").write_text(json.dumps(flight_summary(load_log(recorder.path)), indent=1),
                                                encoding="utf-8")
        rows.append(row)
    summary = summarize_results(rows, (config or approach_config()).stand_off_m)
    with open(out/"episodes.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (out/"summary.json").write_text(json.dumps(dict(
        model="objectpilot.step_approach flight loop in the objectpilot.stepsim test world (not a flight model)",
        seeds=[seeds[0], seeds[-1]], plan=asdict(plan), noise=None if noise is None else asdict(noise),
        results=summary), indent=1), encoding="utf-8")
    echo(json.dumps(summary))
    return rows, summary


# -- command line ----------------------------------------------------------------------------------
def main(argv=None):
    from .stepsim import Noise, RESERVED_SEEDS
    parser = argparse.ArgumentParser(description="StepPlanner approach flight (see research/STEP_APPROACH.md)")
    sub = parser.add_subparsers(dest="mode", required=True)
    sim = sub.add_parser("simulate", help="the flight loop in the geometric test world; no network")
    fly = sub.add_parser("fly", help="FLY THE AIRCRAFT: StepPlanner approach to one print")
    show = sub.add_parser("summary", help="summarize a flight.jsonl")
    for p in (sim, fly):
        p.add_argument("--out", required=True)
        p.add_argument("--stand-off-m", type=float, default=STAND_OFF_M)
        p.add_argument("--minimum-m", type=float, default=MINIMUM_M,
                       help="planner minimum range; the flight aborts 3 cm inside it")
        p.add_argument("--max-seconds", type=float, default=240.)
    sim.add_argument("--height-cm", type=float, default=154., help="test-world prints are centered at 145 cm")
    sim.add_argument("--episodes", type=int, default=300)
    sim.add_argument("--seed", type=int, default=60000)
    sim.add_argument("--noise", help="JSON of stepsim Noise fields, e.g. configs/tt_step_noise_measured.json")
    sim.add_argument("--keep-logs", type=int, default=5, help="write flight.jsonl for the first N episodes")
    fly.add_argument("--height-cm", type=float, default=160.,
                     help="flight height by downward ToF: about the print center + 100 x stand-off x tan 10.5")
    fly.add_argument("--class-name", default="cup")
    fly.add_argument("--weights", default="models/yolo11s.pt")
    fly.add_argument("--confidence", type=float, default=.25,
                     help="detector threshold; search candidates below the planner's 0.45 are centered and re-checked")
    fly.add_argument("--imgsz", type=int, default=960, help="YOLO input width (TT video is 960x720)")
    fly.add_argument("--host", default="192.168.10.1")
    fly.add_argument("--no-frames", action="store_true", help="do not save observed frames as JPEG")
    show.add_argument("log")
    a = parser.parse_args(argv)
    if a.mode == "summary":
        summary = flight_summary(load_log(a.log))
        print_summary(summary)
        (Path(a.log).parent/"summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
        return 0
    if Path(a.out).exists():
        parser.error(f"{a.out} already exists; logs are never overwritten, choose a new --out folder")
    if not (.30 <= a.minimum_m <= a.stand_off_m-.12 and a.stand_off_m <= 1. and 80 <= a.height_cm <= 200
            and 60 <= a.max_seconds <= 600):
        parser.error("need 0.30 <= minimum <= stand-off - 0.12, stand-off <= 1.0 m, 80-200 cm and 60-600 s")
    config = approach_config(stand_off_m=a.stand_off_m, minimum_m=a.minimum_m)
    plan = approach_plan(height_cm=a.height_cm, max_seconds=a.max_seconds, min_tof_m=a.minimum_m-.03)
    if a.mode == "simulate":
        seeds = list(range(a.seed, a.seed+a.episodes))
        if a.episodes < 1 or seeds[0] < RESERVED_SEEDS.stop and seeds[-1] >= RESERVED_SEEDS.start:
            parser.error("seeds 10000-59999 belong to the PID/PPO simulator splits; use 60000 or higher")
        noise = None
        if a.noise:
            fields = json.loads(Path(a.noise).read_text(encoding="utf-8"))
            unknown = set(fields)-set(Noise.__dataclass_fields__)
            if unknown:
                parser.error(f"unknown Noise fields: {sorted(unknown)}")
            noise = replace(Noise(), **fields)
        simulate(seeds, a.out, plan, config, noise, a.keep_logs)
        return 0
    if not sys.stdin.isatty():
        parser.error("fly needs an interactive terminal for the pre-flight confirmation")
    plan = replace(plan, label=a.class_name)
    aircraft = TelloAircraft(a.host, a.weights, plan.label, a.confidence, a.imgsz)
    recorder = None
    try:
        recorder = Recorder(a.out, save_frames=not a.no_frames)
        recorder.write(dict(event="start", mode="fly", plan=asdict(plan), planner=asdict(config),
                            weights=Path(a.weights).name, confidence=a.confidence))
        ApproachFlight(aircraft, plan, recorder, config).run(confirm=terminal_confirm)
    finally:
        aircraft.close()
        if recorder:
            recorder.close()
    if recorder:
        summary = flight_summary(load_log(recorder.path))
        print_summary(summary)
        (Path(a.out)/"summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
