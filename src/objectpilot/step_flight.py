"""Supervised step-response calibration flight for the stop-and-go planner.

THIS MODULE FLIES THE AIRCRAFT. FlightSession here is the only code in the repository
that sends takeoff, movement or landing commands; this calibration flight and
objectpilot.step_approach build on it. Replay, collection and the ToF bench stay
ground-only. Follow the procedure in research/STEP_FLIGHT.md.

One flight measures what objectpilot.stepsim assumes. The aircraft hovers square in
front of the print, sends single SDK step commands in opposite pairs, and records the
target box, forward ToF and state-stream yaw before and after each. A yaw sweep at
0.8 m then checks the ToF wall-normal fit in flight.

    python -m objectpilot.step_flight rehearse --out runs/step-rehearsal    # test world, no network
    python -m objectpilot.step_flight fly --out runs/step-cal-01 --weights models/yolo11s.pt
    python -m objectpilot.step_flight analyze runs/step-cal-01/flight.jsonl

Safety: every command passes an allowlist (no rc, go, flip, motoron or emergency;
moves up to 100 cm, turns up to 90 degrees). The aircraft stops and lands on Ctrl+C,
forward ToF under 0.55 m, battery under 25%, a stale state stream, height outside
0.4-2.3 m, a lost target, repeated errors or timeouts, and the time limit. A forward
step that would end under 0.55 m by ToF is refused. If this program dies, the
aircraft lands by itself 15 s after its last command (Tello SDK behaviour).
"""
from dataclasses import asdict, dataclass, replace
import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time

import numpy as np

from .core import parse_ext_tof
from .ground_link import GroundLink
from .stepwise import check_command, fit_wall_normal, summarize
from .vision import detect_tiled

MAX_MOVE_CM = 100
MAX_TURN_DEG = 90
FLIGHT_ONLY = frozenset(("takeoff", "land", "stop", "mon", "moff", "mdirection 0"))
STATE_KEYS = ("yaw", "pitch", "roll", "vgx", "vgy", "vgz", "tof", "h", "bat", "baro", "templ", "temph",
              "mid", "x", "y", "z")

# (first, second, large). Each pair returns the aircraft to where it started. A large first
# command can push the box out of the frame, so its pair is ordered to swing the box toward
# the center first and the box may be missing until the return command.
TRIALS = (
    ("left 20", "right 20", False),
    ("forward 20", "back 20", False),
    ("right 20", "left 20", False),
    ("cw 10", "ccw 10", False),
    ("left 40", "right 40", True),
    ("ccw 10", "cw 10", False),
    ("forward 20", "back 20", False),
    ("cw 20", "ccw 20", True),
    ("right 40", "left 40", True),
    ("ccw 20", "cw 20", True),
    ("cw 40", "ccw 40", True),
    ("down 20", "up 20", True),
)
# Image direction the box moves for each command: (box axis, sign).
BOX_SHIFT = {"left": (0, 1), "right": (0, -1), "cw": (0, -1), "ccw": (0, 1), "up": (1, 1), "down": (1, -1)}


def permitted_flight(command):
    """Allowlist for flight: SDK setup and queries, takeoff/land/stop, speed, bounded steps."""
    if command in GroundLink.ALLOWED or command in FLIGHT_ONLY:
        return True
    parts = command.split()
    if len(parts) == 2 and parts[0] == "speed" and parts[1].isdigit():
        return 10 <= int(parts[1]) <= 100
    try:
        verb, value = check_command(command)
    except ValueError:
        return False
    return value <= (MAX_TURN_DEG if verb in ("cw", "ccw") else MAX_MOVE_CM)


class FlightLink(GroundLink):
    """SDK command link that may fly. Used only by this module."""

    def permitted(self, command):
        return permitted_flight(command)


def command_timeout(command, speed_cm_s):
    if command == "takeoff":
        return 20.
    if command == "land":
        return 15.
    verb, value = check_command(command)
    return 6.+value/30 if verb in ("cw", "ccw") else 6.+2*value/speed_cm_s


def wrap(degrees):
    return (degrees+180) % 360-180


def clamp(value, low, high):
    return max(low, min(high, value))


def height_cm(state):
    """Height above the floor: downward ToF when it is in range, else the takeoff-relative height."""
    tof = state.get("tof")
    if tof is not None and 10 < tof < 600:
        return tof
    return state.get("h")


def clean_state(fields, now):
    out = {}
    for key in STATE_KEYS:
        try:
            value = float(fields.get(key))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            out[key] = value
    received = fields.get("received_monotonic")
    out["age_s"] = None if received is None else round(now-received, 3)
    return out


@dataclass
class FlightPlan:
    label: str = "cup"
    start_m: float = 1.5            # taped lens-to-wall distance at takeoff
    hover_m: float = 1.0            # ToF distance for the step trials (ToF is valid up to 1.2 m)
    sweep_m: float = .80            # ToF distance for the wall-normal sweep
    height_cm: float = 160.         # downward-ToF height; the print center should be about 13 cm lower
    speed_cm_s: int = 30
    repeats: int = 2
    sweep: bool = True
    sweep_step_deg: int = 10
    sweep_limit_deg: int = 40
    frames: int = 6
    settle_s: float = .5
    hfov_deg: float = 54.
    center_x: float = .08
    approach_tol_m: float = .10
    distance_tol_m: float = .15
    lateral_tol_m: float = .15
    heading_tol_deg: float = 4.
    height_tol_cm: float = 15.
    min_tof_m: float = .55
    min_battery_takeoff: int = 40   # at low charge the battery falls ~0.5% per second in flight
    min_battery: int = 25
    max_seconds: float = 480.
    max_errors: int = 5
    max_timeouts: int = 2
    state_stale_s: float = 1.5
    min_height_cm: float = 40.
    max_height_cm: float = 230.
    mission_pad: bool = False


def checklist(plan):
    return f"""
STEP CALIBRATION FLIGHT: the aircraft takes off and flies for about 5 minutes.
  [ ] Propeller guards on; nobody within 2 m of the flight path except the operator
  [ ] Floor lit and textured under the whole path (downward positioning)
  [ ] Aircraft on the tape mark {plan.start_m:.2f} m from the wall, facing the {plan.label} print square
  [ ] Print center about {plan.height_cm-13:.0f} cm above the floor
  [ ] 1 m clear on each side and behind the mark; ceiling above {plan.height_cm+60:.0f} cm
  [ ] Battery at least {plan.min_battery_takeoff}%; your hand on Ctrl+C (stop and land)
If this program freezes, the aircraft lands by itself 15 s after its last command.
Type "fly" to take off, anything else to cancel: """


class Abort(Exception):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class Recorder:
    """flight.jsonl, one row per event, plus the observed frames as JPEG files."""

    def __init__(self, out, clock=time.monotonic, save_frames=True):
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=False)
        self.path = self.out/"flight.jsonl"
        self.handle = self.path.open("x", encoding="utf-8")
        self.clock, self.t0, self.save_frames = clock, clock(), save_frames

    def write(self, row, images=()):
        row = dict(t=round(self.clock()-self.t0, 2), **row)
        if images and self.save_frames:
            import cv2
            folder = self.out/"frames"
            folder.mkdir(exist_ok=True)
            row["images"] = []
            for k, image in enumerate(images):
                name = f"obs{row['n']:04d}_{k}.jpg"
                cv2.imwrite(str(folder/name), image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                row["images"].append(name)
        self.handle.write(json.dumps(row, allow_nan=False)+"\n")
        self.handle.flush()
        return row

    def close(self):
        self.handle.close()


class FlightSession:
    """Guarded flight shared by every flying script in this package.

    Pre-flight checks, typed operator confirmation, takeoff, climb to the plan height,
    guarded commands and observations, and abort-and-land. Subclasses implement mission(),
    which runs after the climb and returns an outcome string.
    """

    def __init__(self, aircraft, plan, recorder, clock=time.monotonic, echo=print):
        self.a, self.p, self.rec, self.clock, self.echo = aircraft, plan, recorder, clock, echo
        self.airborne = False
        self.errors = self.timeouts = self.commands = self.observations = 0
        self.started = self.last = self.last_sent = None

    def checklist(self):
        return checklist(self.p)

    def mission(self):
        raise NotImplementedError

    def before_takeoff(self):
        """Runs after the operator confirmed, before takeoff; Abort here ends without flying."""

    def run(self, confirm):
        outcome = "completed"
        try:
            self.preflight()
            if not confirm(self.checklist()):
                return self.finish("not_confirmed")
            self.before_takeoff()
            self.started = self.clock()
            self.airborne = True             # from here on, always try to land
            reply = self.send("takeoff", "takeoff")
            if reply != "ok":
                if reply != "timeout":
                    # An answered refusal ("error, high temp", low battery) means the motors never
                    # started; only a missing reply leaves the aircraft possibly airborne.
                    self.airborne = False
                hot = self.last["state"].get("temph") if self.last else None
                raise Abort(f"takeoff_refused: {reply}" + (f" (board {hot:.0f} C)" if hot is not None else ""))
            self.guard(self.observe("takeoff"))
            self.climb()
            outcome = self.mission() or outcome
        except Abort as stop:
            outcome = stop.reason
        except KeyboardInterrupt:
            outcome = "operator_interrupt"
        except Exception as exc:  # any failure still lands the aircraft
            outcome = f"exception {type(exc).__name__}: {exc}"
        finally:
            if self.airborne:
                self.land()
        return self.finish(outcome)

    def preflight(self):
        p = self.p
        if self.query("command") != "ok":
            raise Abort("no_sdk_mode")
        self.query("sdk?")
        battery = self.query("battery?")
        if not battery.isdigit() or int(battery) < p.min_battery_takeoff:
            raise Abort(f"battery_{battery}")
        if self.query(f"speed {p.speed_cm_s}") != "ok":
            raise Abort("speed_rejected")
        for command in ("mon", "mdirection 0") if p.mission_pad else ():
            if self.query(command) != "ok":
                raise Abort("mission_pad_rejected")
        self.a.start()
        obs = self.observe("ground")
        state = obs["state"]
        if state.get("age_s") is None or state["age_s"] > p.state_stale_s or state.get("yaw") is None:
            raise Abort("no_state_stream")
        if len(obs["frames"]) < p.frames:
            raise Abort(f"no_video ({len(obs['frames'])} of {p.frames} frames)")
        if obs["tof_answered"] < p.frames/2:
            raise Abort("no_tof_replies")

    def climb(self):
        p = self.p
        for attempt in range(5):
            height = height_cm(self.last["state"])
            if height is None:
                raise Abort("no_height")
            diff = p.height_cm-height
            if abs(diff) < p.height_tol_cm:
                return
            if attempt == 4:
                raise Abort("climb_failed")
            cm = int(clamp(round(abs(diff)), 20, MAX_MOVE_CM))
            self.move(f"{'up' if diff > 0 else 'down'} {cm}", "climb")

    def land(self):
        for attempt in range(5):
            start = self.clock()
            try:
                if attempt == 0:
                    self.a.send("stop", 3.)          # SDK 3.0: hover now, even mid-move
                reply = self.a.send("land", 15.)
            except KeyboardInterrupt:
                self.echo("Landing; Ctrl+C is ignored until the aircraft is down.")
                continue
            except Exception as exc:
                reply = f"exception {type(exc).__name__}"
            self.rec.write(dict(event="command", kind="land", command="land", reply=reply,
                                seconds=round(self.clock()-start, 2)))
            self.echo(f"land -> {reply}")
            if reply == "ok":
                self.airborne = False
                return True
            self.a.wait(1.)
        self.echo("LANDING NOT CONFIRMED. Keep clear: the aircraft lands by itself 15 s after its last command.")
        return False

    def finish(self, outcome):
        self.rec.write(dict(event="end", outcome=outcome, commands=self.commands, errors=self.errors,
                            timeouts=self.timeouts, observations=self.observations))
        self.echo(f"outcome: {outcome}")
        return outcome

    # -- primitives ----------------------------------------------------------------------------
    def query(self, command, timeout=5.):
        reply = self.a.send(command, timeout)
        self.rec.write(dict(event="command", kind="preflight", command=command, reply=reply, seconds=0.))
        return reply

    def note(self, text):
        self.rec.write(dict(event="note", text=text))
        self.echo(text)

    def send(self, command, kind, **extra):
        p = self.p
        if command.startswith("forward ") and self.last is not None and self.last["tof_m"] is not None:
            if self.last["tof_m"]-check_command(command)[1]/100 < p.min_tof_m:
                raise Abort("forward_blocked")
        start = self.clock()
        self.last_sent = command
        reply = self.a.send(command, command_timeout(command, p.speed_cm_s))
        seconds = self.clock()-start
        self.commands += 1
        self.rec.write(dict(event="command", kind=kind, command=command, reply=reply, seconds=round(seconds, 2),
                            **extra))
        elapsed = self.clock()-self.started if self.started is not None else 0.
        self.echo(f"[{elapsed:6.1f} s] {kind:<14} {command:<11} -> {reply} ({seconds:.1f} s)")
        if reply == "timeout":
            self.timeouts += 1
            self.rec.write(dict(event="command", kind="stop", command="stop", reply=self.a.send("stop", 3.),
                                seconds=0.))
            if self.timeouts >= p.max_timeouts:
                raise Abort("timeouts")
        elif reply != "ok":
            self.errors += 1
            if self.errors > p.max_errors:
                raise Abort("errors")
        return reply

    def observe(self, tag):
        raw = self.a.observe(self.p.frames)
        state = raw["state"]
        obs = summarize(raw["frames"], raw["tof"], self.p.label, yaw_deg=state.get("yaw"))
        self.observations += 1
        row = dict(event="observe", n=self.observations, tag=tag,
                   frames=[[[d[0]]+[round(float(v), 4) for v in d[1:6]] for d in f] for f in raw["frames"]],
                   tof=[None if r is None else round(float(r), 4) for r in raw["tof"]],
                   tof_answered=raw["tof_answered"], state=state,
                   box=None if obs.box is None else [round(float(v), 4) for v in obs.box],
                   seen=round(obs.seen, 2), tof_m=None if obs.tof is None else round(float(obs.tof), 4),
                   yaw=obs.yaw_deg, conf=None if obs.conf is None else round(float(obs.conf), 3))
        if "truth" in raw:
            row["truth"] = raw["truth"]
        if "detection" in raw:
            row["detection"] = raw["detection"]
        self.last = self.rec.write(row, raw.get("images", ()))
        return self.last

    def reobserve(self):
        obs = self.observe("reobserve")
        self.guard(obs)
        return obs

    def guard(self, obs):
        p, state = self.p, obs["state"]
        if self.started is not None and self.clock()-self.started > p.max_seconds:
            raise Abort("time_limit")
        if state.get("age_s") is None or state["age_s"] > p.state_stale_s:
            raise Abort("state_stale")
        if state.get("bat") is not None and state["bat"] < p.min_battery:
            raise Abort("battery_low")
        height = height_cm(state)
        if height is not None and height < p.min_height_cm and self.started is not None \
                and not (self.last_sent or "").startswith("down"):
            # November 15: at 32% the TT descended by itself (vgz -6 dm/s) and answered the next
            # turn with "error Not joystick": its own low-battery landing, not a planner move.
            raise Abort(f"aircraft_descending_on_its_own: {height:.0f} cm, battery {state.get('bat')}%")
        if height is None or not p.min_height_cm <= height <= p.max_height_cm:
            raise Abort("height_out_of_bounds")
        if obs["tof_m"] is not None and obs["tof_m"] < p.min_tof_m:
            self.send("back 20", "safety")
            raise Abort("too_close")

    def move(self, command, kind, **extra):
        """One guarded command, a settle pause and the observation after it: (reply, observation)."""
        reply = self.send(command, kind, **extra)
        self.a.wait(self.p.settle_s)
        obs = self.observe(kind)
        self.guard(obs)
        box = obs["box"]
        self.echo(f"{'':10}box {'none' if box is None else f'cx {box[0]:.3f} cy {box[1]:.3f} h {box[3]:.3f}'}"
                  f" | ToF {obs['tof_m']} | yaw {obs['yaw']}")
        return reply, obs


class CalibrationFlight(FlightSession):
    """Step-trial sequence: approach to the trial distance, opposite step pairs, ToF sweep."""

    def __init__(self, aircraft, plan, recorder, clock=time.monotonic, echo=print):
        super().__init__(aircraft, plan, recorder, clock, echo)
        self.yaw0 = self.box_h0 = self.yaw_sign = None
        self.tan_half = math.tan(math.radians(plan.hfov_deg)/2)

    def mission(self):
        self.approach()
        for _ in range(self.p.repeats):
            for first, second, large in TRIALS:
                self.recenter()
                if large:
                    first, second = self.toward_center(first, second)
                self.step(first, "trial", expect_target=not large)
                self.step(second, "trial")
        if self.p.sweep:
            self.sweep()
        return "completed"

    def approach(self):
        """Yaw the box to the center, then step to the trial distance by ToF."""
        p = self.p
        obs = self.last
        if obs["box"] is None:
            obs = self.reobserve()
            if obs["box"] is None:
                raise Abort("target_not_seen")
        ex = obs["box"][0]-.5
        if abs(ex) > p.center_x:
            degrees = round(math.degrees(math.atan(2*ex*self.tan_half)))
            if degrees:
                obs = self.step(f"{'cw' if degrees > 0 else 'ccw'} {abs(degrees)}", "approach")
        blind_cm = 0
        for _ in range(10):
            tof = obs["tof_m"]
            if tof is not None and abs(tof-p.hover_m) <= p.approach_tol_m:
                break
            if tof is None:
                blind_cm += 20       # no forward ToF yet: at most the taped distance, in 20 cm steps
                if blind_cm > (p.start_m-p.hover_m)*100+10:
                    raise Abort("tof_unavailable")
                command = "forward 20"
            elif tof > p.hover_m:
                command = f"forward {int(clamp(round((tof-p.hover_m)*100), 20, 40))}"
            else:
                command = "back 20"
            obs = self.step(command, "approach")
        else:
            raise Abort("approach_failed")
        self.yaw0, self.box_h0 = obs["yaw"], obs["box"][3]
        self.note(f"trial heading: state yaw {self.yaw0}, ToF {obs['tof_m']}, box h {self.box_h0}")

    def recenter(self):
        """Keep the trials near one spot: height, distance by ToF, heading by yaw, offset by the box."""
        p = self.p
        self.hold_height()
        for attempt in range(4):
            obs = self.last if self.last["tof_m"] is not None else self.reobserve()
            tof = obs["tof_m"]
            if tof is not None and abs(tof-p.hover_m) <= p.distance_tol_m:
                break
            if attempt == 3:
                raise Abort("distance_hold_failed")
            # No ToF reply: beyond its ~1.2 m range only if the box is smaller than at the trial distance.
            if tof is None and not (obs["box"] is not None and obs["box"][3] < .95*self.box_h0):
                raise Abort("tof_unavailable")
            self.step("back 20" if tof is not None and tof < p.hover_m else "forward 20", "correction")
        self.hold_heading()
        lateral = self.lateral_m(self.last)
        if lateral is not None and abs(lateral) >= p.lateral_tol_m:
            cm = int(clamp(round(abs(lateral)*100), 20, 40))
            self.step(f"{'right' if lateral > 0 else 'left'} {cm}", "correction")

    def hold_heading(self):
        if self.yaw_sign is None or self.yaw0 is None or self.last["yaw"] is None:
            return
        error = wrap(self.last["yaw"]-self.yaw0)*self.yaw_sign      # > 0: clockwise of the trial heading
        if abs(error) >= self.p.heading_tol_deg:
            self.step(f"{'ccw' if error > 0 else 'cw'} {int(min(30, round(abs(error))))}", "correction")

    def hold_height(self):
        height = height_cm(self.last["state"])
        diff = None if height is None else self.p.height_cm-height
        if diff is not None and abs(diff) >= 20:
            cm = int(clamp(round(abs(diff)), 20, 60))
            self.step(f"{'up' if diff > 0 else 'down'} {cm}", "correction", expect_target=False)

    def sweep(self):
        """Yaw sweep at the sweep distance: ToF against heading, for the wall-normal fit."""
        p = self.p
        self.hold_height()
        for _ in range(4):
            tof = self.last["tof_m"]
            if tof is None:
                raise Abort("tof_unavailable")
            if abs(tof-p.sweep_m) <= .08:
                break
            command = (f"forward {int(clamp(round((tof-p.sweep_m)*100), 20, 30))}" if tof > p.sweep_m
                       else "back 20")
            self.step(command, "sweep_approach")
        self.hold_heading()
        self.note("sweep start")
        n, deg = p.sweep_limit_deg//p.sweep_step_deg, p.sweep_step_deg
        for command in [f"cw {deg}"]*n+[f"ccw {deg}"]*(2*n)+[f"cw {deg}"]*n:
            self.step(command, "sweep", expect_target=False)

    def step(self, command, kind, expect_target=True, retry=True):
        """One guarded command, a settle pause and the observation after it.

        A command answered with an error is observed (no motion expected) and sent once more.
        A box that should be visible but is not gets a second look and, if the yaw sign is
        known, a turn back to the trial heading before the flight is aborted.
        """
        before = self.last
        reply, obs = self.move(command, kind)
        box = obs["box"]
        if reply not in ("ok", "timeout") and retry:
            return self.step(command, kind, expect_target, retry=False)
        if expect_target and box is None:
            obs = self.reobserve()
            if obs["box"] is None and kind != "correction":
                self.hold_height()
                self.hold_heading()
                obs = self.last
            if obs["box"] is None:
                raise Abort("target_lost")
        if reply == "ok" and self.yaw_sign is None:
            self.learn_yaw_sign(command, before, obs)
        return obs

    def toward_center(self, first, second):
        """Order a large pair so that its first command moves the box toward the image center."""
        box = self.last["box"]
        axis, sign = BOX_SHIFT.get(check_command(first)[0], (None, 0))
        if box is None or axis is None:
            return first, second
        return (second, first) if (box[axis]-.5)*sign > 0 else (first, second)

    def learn_yaw_sign(self, command, before, after):
        verb, value = check_command(command)
        if verb not in ("cw", "ccw") or value < 8 or before is None or before["yaw"] is None or after["yaw"] is None:
            return
        turn = wrap(after["yaw"]-before["yaw"])
        if abs(turn) >= value/2:
            self.yaw_sign = int(math.copysign(1, turn))*(1 if verb == "cw" else -1)
            self.note(f"state yaw {'increases' if self.yaw_sign > 0 else 'decreases'} with cw")

    def lateral_m(self, obs):
        """Target offset to the right of the aircraft, from the box and the forward ToF."""
        if obs["box"] is None or obs["tof_m"] is None:
            return None
        return (obs["box"][0]-.5)*2*self.tan_half*obs["tof_m"]


# -- aircraft --------------------------------------------------------------------------------------
class TelloAircraft:
    """The delivered TT: FlightLink commands, UDP 8890 state, H.264 video and a YOLO detector."""

    def __init__(self, host, weights, label, confidence=.45, imgsz=640, tile_below_h=.2):
        from .tof_bench import StateListener
        from .vision import YOLODetector
        # While the target is unseen or small (box under tile_below_h of the frame height), each
        # frame is also searched in four overlapping crops (objectpilot.vision.detect_tiled).
        # On November 13-15 frames a single pass at 960 scored a cup at 1.8 m (frame edge) 0.08-0.66
        # and at 2.5 m 0.04-0.07; full frame + crops at 640 scored 0.80-0.85 and median 0.37.
        self.detector = YOLODetector(weights, label, confidence, imgsz)   # load weights before any network use
        self.tile_below_h, self.wide = tile_below_h, True
        # The first YOLO call takes seconds on a CPU; pay it here, not inside the first observation.
        self.detector.detect(np.zeros((720, 960, 3), np.uint8), 0., 0)
        self.link = FlightLink(host)
        try:
            self.listener = StateListener()
        except BaseException:
            self.link.socket.close()
            raise
        self.video = None
        self.last_frame_id = None

    def send(self, command, timeout):
        try:
            return self.link.request(command, timeout)[0]
        except RuntimeError as exc:
            return str(exc) or "error"
        except TimeoutError:
            return "timeout"

    @property
    def labels(self):
        return set(self.detector.model.names.values())

    def set_label(self, label):
        """Retarget the detector, e.g. after the operator named the print."""
        if label not in self.labels:
            raise Abort(f"unknown_label: {label}")
        self.detector.label = label

    def start(self):
        from .video import LatestVideo
        if self.send("streamon", 5.) != "ok":
            raise Abort("streamon_failed")
        self.link.streaming = True
        self.video = LatestVideo("tello").start()
        deadline = time.monotonic()+10.
        while self.video.latest() is None:
            if self.video.error or time.monotonic() > deadline:
                raise Abort("no_video")
            time.sleep(.05)

    def wait(self, seconds):
        time.sleep(seconds)

    def state(self):
        return clean_state(dict(self.listener.latest), time.monotonic())

    def observe(self, n):
        frames, images = [], []
        # Give up only when no new frame has arrived for 3 s, so slow inference does not count as lost video.
        deadline = time.monotonic()+3.
        while len(frames) < n and time.monotonic() < deadline:
            if self.video.error:
                raise RuntimeError(self.video.error)
            sample = self.video.latest()
            if sample is None or sample.frame_id == self.last_frame_id:
                time.sleep(.005)
                continue
            self.last_frame_id = sample.frame_id
            if self.wide:
                detections = detect_tiled(self.detector, sample.image, sample.captured_at, sample.frame_id)
            else:
                detections = self.detector.detect(sample.image, sample.captured_at, sample.frame_id)
            frames.append([(d.label, d.cx, d.cy, d.width, d.height, d.confidence) for d in detections])
            images.append(sample.image)
            deadline = time.monotonic()+3.
        tof, answered = [], 0
        for _ in range(n):
            try:
                reply, start, _ = self.link.request("EXT tof?", .5)
            except (TimeoutError, RuntimeError, OSError):
                tof.append(None)
                continue
            answered += 1
            tof.append(parse_ext_tof(reply, start).distance_m)
        mode = "full+tiles" if self.wide else "full"
        heights = [max((d for d in f if d[0] == self.detector.label), key=lambda d: d[5])[4]
                   for f in frames if any(d[0] == self.detector.label for d in f)]
        self.wide = not (len(heights) >= len(frames)/2 and heights and float(np.median(heights)) >= self.tile_below_h)
        return dict(frames=frames, images=images, tof=tof, tof_answered=answered, state=self.state(), detection=mode)

    def close(self):
        if self.video:
            self.video.close()
        self.listener.close()
        self.link.close()


class SimAircraft:
    """Rehearsal: the same flight against objectpilot.stepsim's geometric test world. No network."""

    def __init__(self, seed, plan, noise=None, pose=None):
        """pose: (x, y, heading_rad) start on the floor; default: square in front of the print at start_m."""
        from .stepsim import StepWorld, default_prints
        rng = np.random.default_rng(seed)
        prints = default_prints()
        target = next(t for t in prints if t.label == plan.label)
        if pose is None:
            pose = (-plan.start_m, target.y+rng.normal(0, .05), math.radians(rng.normal(0, 3)))
        pose = (pose[0], pose[1], 0., pose[2])
        self.target = target
        self.world = StepWorld(rng, pose, prints, noise=noise, frames=plan.frames, speed_cm_s=plan.speed_cm_s)
        self.rng, self.airborne, self.battery = rng, False, 88.

    def clock(self):
        return self.world.time

    def set_label(self, label):
        """Retarget: the test world's prints are labelled cup, bottle and bag."""
        self.target = next((t for t in self.world.prints if t.label == label), None)
        if self.target is None:
            raise Abort(f"unknown_label: {label}")

    def send(self, command, timeout):
        if not permitted_flight(command):
            raise ValueError(f"not permitted: {command!r}")
        w = self.world
        if command == "battery?":
            return str(int(self.battery))
        if command == "sdk?":
            return "30"
        if command == "takeoff":
            w.z, self.airborne = .85+self.rng.normal(0, .05), True
            w.time += 6.
            return "ok"
        if command == "land":
            w.z, self.airborne = 0., False
            w.time += 4.
            return "ok"
        if command in ("command", "streamon", "streamoff", "stop", "mon", "moff", "mdirection 0") \
                or command.startswith("speed "):
            return "ok"
        if not self.airborne:
            return "error Motor stop"
        self.battery -= .1
        return w.execute(command)

    def start(self):
        pass

    def wait(self, seconds):
        self.world.time += seconds

    def observe(self, n):
        w = self.world
        frames, replies = w.observe()
        state = dict(yaw=w.imu_yaw(), tof=round(w.z*100) if w.z > .1 else 10., h=round(w.z*100),
                     bat=float(int(self.battery)), vgx=0., vgy=0., vgz=0., age_s=.05)
        truth = dict(x=round(w.x, 4), y=round(w.y, 4), z=round(w.z, 4),
                     heading_deg=round(math.degrees(w.heading), 2))
        return dict(frames=frames, tof=replies, tof_answered=len(replies), state=state, truth=truth)

    def close(self):
        pass


# -- analysis --------------------------------------------------------------------------------------
HORIZONTAL = ("forward", "back", "left", "right")
# Expected image response if the SDK and camera conventions match the planner.
EXPECTED_SIGN = {"left": ("dex", 1), "right": ("dex", -1), "cw": ("dex", -1), "ccw": ("dex", 1),
                 "forward": ("dh", 1), "back": ("dh", -1), "up": ("dey", 1), "down": ("dey", -1)}


def load_log(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def step_pairs(rows):
    """(observation before, command, observation after) for every step command in the log."""
    out, before, pending = [], None, None
    for row in rows:
        if row["event"] == "observe":
            if pending is not None and pending[0] is not None:
                out.append((pending[0], pending[1], row))
            pending, before = None, row
        elif row["event"] == "command":
            try:
                check_command(row["command"])
            except ValueError:
                continue
            pending = (before, row)
    return out


def measure(before, command, after, tan_half):
    verb, value = check_command(command["command"])
    m = dict(command=command["command"], verb=verb, value=value, kind=command["kind"], reply=command["reply"],
             seconds=command["seconds"])
    b, a = before["box"], after["box"]
    if b and a:
        m.update(exb=b[0]-.5, exa=a[0]-.5, dex=a[0]-b[0], dey=a[1]-b[1], dh=a[3]/b[3]-1)
    if before["tof_m"] is not None and after["tof_m"] is not None:
        m["dtof"] = after["tof_m"]-before["tof_m"]
        if b and a:
            m["dlat"] = ((a[0]-.5)*after["tof_m"]-(b[0]-.5)*before["tof_m"])*2*tan_half
    if before["yaw"] is not None and after["yaw"] is not None:
        m["dyaw"] = wrap(after["yaw"]-before["yaw"])
    hb, ha = height_cm(before["state"]), height_cm(after["state"])
    if hb is not None and ha is not None:
        m["dheight_cm"] = ha-hb
    speeds = [abs(after["state"][k]) for k in ("vgx", "vgy", "vgz") if k in after["state"]]
    if speeds:
        m["speed_after_dm_s"] = max(speeds)
    return m


def rms(values):
    return math.sqrt(sum(v*v for v in values)/len(values))


def stats(values, digits=3):
    if not values:
        return None
    return dict(n=len(values), median=round(statistics.median(values), digits),
                sd=round(statistics.stdev(values), digits) if len(values) > 1 else None)


def hfov_from_turn(exb, exa, turn_deg):
    """Video HFOV that makes a measured turn move the box from exb to exa (pinhole, tilt ignored)."""
    best = None
    for hfov in np.arange(30., 90.01, .1):
        u = 2*math.tan(math.radians(hfov)/2)
        predicted = math.tan(math.atan(u*exb)-math.radians(turn_deg))/u
        error = abs(predicted-exa)
        if best is None or error < best[0]:
            best = (error, float(hfov))
    return best[1] if best[0] < .02 else None


def sweep_analysis(rows, yaw_sign):
    """Fit the wall normal to the in-flight sweep and compare oblique readings with D/cos."""
    first = next((i for i, r in enumerate(rows) if r["event"] == "command" and r["kind"] == "sweep"), None)
    if first is None:
        return None
    start = next((i for i in range(first-1, -1, -1) if rows[i]["event"] == "observe"), None)
    if start is None or rows[start]["yaw"] is None:
        return None
    reference = rows[start]
    observed = [reference]+[after for _, command, after in step_pairs(rows[start:]) if command["kind"] == "sweep"]
    points = [(wrap(o["yaw"]-reference["yaw"])*yaw_sign, o["tof_m"] if o["tof_m"] is not None else math.inf)
              for o in observed if o["yaw"] is not None]
    finite = [(h, r) for h, r in points if math.isfinite(r)]
    if len(finite) < 3 or max(h for h, _ in finite)-min(h for h, _ in finite) < 15:
        return dict(points=len(points), finite=len(finite), fit=None)
    normal, _ = fit_wall_normal(points, 0., 20.)
    near = [r*math.cos(math.radians(h-normal)) for h, r in finite if abs(h-normal) <= 10]
    distance = statistics.median(near) if near else None
    excess, by_angle = [], {}
    for h, r in finite:
        angle = h-normal
        if distance is None:
            continue
        ratio = r/(distance/math.cos(math.radians(angle)))
        by_angle.setdefault(int(round(angle/10)*10), []).append(round(ratio, 3))
        if abs(angle) >= 15:
            excess.append((ratio-1)/(1/math.cos(math.radians(angle))-1))
    scatter = None
    if distance is not None and len(finite) >= 4:
        k = clamp(statistics.median(excess), 0., 2.) if excess else 0.
        residuals = [r-distance/math.cos(math.radians(h-normal))*(1+k*(1/math.cos(math.radians(h-normal))-1))
                     for h, r in finite if abs(h-normal) <= 30]
        scatter = statistics.stdev(residuals) if len(residuals) >= 3 else None
    result = dict(points=len(points), finite=len(finite), normal_offset_deg=round(normal, 1),
                  perpendicular_m=None if distance is None else round(distance, 3),
                  ratio_to_d_over_cos=dict(sorted(by_angle.items())),
                  oblique_excess=round(statistics.median(excess), 3) if excess else None,
                  scatter_m=None if scatter is None else round(scatter, 4),
                  out_of_range_beyond_deg=min((abs(h) for h, r in points if not math.isfinite(r)), default=None))
    if "truth" in reference:
        result["true_normal_offset_deg"] = round(-reference["truth"]["heading_deg"], 1)
    return result


def analyze(rows, hfov_deg=54.):
    tan_half = math.tan(math.radians(hfov_deg)/2)
    samples = [measure(b, c, a, tan_half) for b, c, a in step_pairs(rows)]
    ok = [s for s in samples if s["reply"] == "ok"]
    turns = [s["dyaw"]*(1 if s["verb"] == "cw" else -1) for s in ok if s["verb"] in ("cw", "ccw") and "dyaw" in s]
    yaw_sign = -1 if turns and statistics.median(turns) < 0 else 1

    commands = {}
    for s in samples:
        g = commands.setdefault(s["command"], dict(n=0, ok=0, errors=0, timeouts=0, seconds=[], dex=[], dey=[],
                                                   dh=[], dtof=[], dyaw=[]))
        g["n"] += 1
        g["ok" if s["reply"] == "ok" else "timeouts" if s["reply"] == "timeout" else "errors"] += 1
        if s["reply"] == "ok":
            g["seconds"].append(s["seconds"])
            for key in ("dex", "dey", "dh", "dtof", "dyaw"):
                if key in s:
                    g[key].append(s[key])
    by_command = {name: dict(n=g["n"], ok=g["ok"], errors=g["errors"], timeouts=g["timeouts"],
                             **{k: stats(g[k]) for k in ("seconds", "dex", "dey", "dh", "dtof", "dyaw")})
                  for name, g in sorted(commands.items())}

    scale, cross, heading_drift, rotation, hfov = {}, [], [], [], []
    for s in ok:
        verb, metres = s["verb"], s["value"]/100
        if verb in ("forward", "back") and "dtof" in s:
            scale.setdefault(verb, []).append(-s["dtof"]/metres if verb == "forward" else s["dtof"]/metres)
            if "dlat" in s:
                cross.append((metres, s["dlat"]))
        elif verb in ("left", "right") and "dlat" in s:
            scale.setdefault(verb, []).append(s["dlat"]/metres if verb == "left" else -s["dlat"]/metres)
            cross.append((metres, -s["dtof"]))
        elif verb in ("up", "down") and "dheight_cm" in s:
            scale.setdefault(verb, []).append(s["dheight_cm"]/s["value"]*(1 if verb == "up" else -1))
        if verb in HORIZONTAL and "dyaw" in s:
            heading_drift.append(s["dyaw"])
        if verb in ("cw", "ccw") and "dyaw" in s:
            turn = s["dyaw"]*yaw_sign*(1 if verb == "cw" else -1)
            rotation.append((s["value"], turn-s["value"]))
            if "dex" in s and abs(turn) >= 5 and abs(s["exa"]) < .45 and abs(s["exb"]) < .45:
                estimate = hfov_from_turn(s["exb"], s["exa"], turn*(1 if verb == "cw" else -1))
                if estimate:
                    hfov.append(estimate)
    signs = {}
    for verb, (key, sign) in EXPECTED_SIGN.items():
        values = [s[key] for s in ok if s["verb"] == verb and key in s]
        if values:
            signs[verb] = dict(key=key, median=round(statistics.median(values), 3),
                               as_expected=statistics.median(values)*sign > 0)
    settle = [s["speed_after_dm_s"] for s in ok if "speed_after_dm_s" in s]
    sweep = sweep_analysis(rows, yaw_sign)

    noise = {}
    horizontal = [v for verb in HORIZONTAL for v in scale.get(verb, [])]
    if len(horizontal) >= 4:
        # A fixed shortfall is kept apart from the spread around it.
        centre = statistics.median(horizontal)
        noise["move_scale_bias"] = round(centre-1, 3)
        noise["move_scale_sd"] = round(rms([v-centre for v in horizontal]), 3)
    if len(cross) >= 4:
        # stepsim draws cross drift with sd 0.01 m + move_cross_sd * move length.
        mean_m = statistics.mean(m for m, _ in cross)
        noise["move_cross_sd"] = round(max(0., (rms([c for _, c in cross])-.01)/mean_m), 3)
    if len(rotation) >= 4:
        values = np.array([v for v, _ in rotation], float)
        shortfall = -float(np.median([e for _, e in rotation]))
        noise["yaw_bias_deg"] = round(shortfall, 2)
        errors = np.array([e for _, e in rotation], float)+shortfall
        if len(set(values)) >= 2:
            a, b = np.linalg.lstsq(np.c_[np.ones_like(values), values**2], errors**2, rcond=None)[0]
            noise["yaw_sd_deg"], noise["yaw_scale_sd"] = round(math.sqrt(max(a, 0.)), 2), round(math.sqrt(max(b, 0.)), 3)
        else:
            noise["yaw_sd_deg"] = round(rms(list(errors)), 2)
    if len(heading_drift) >= 4:
        noise["move_heading_sd_deg"] = round(rms(heading_drift), 2)
    if len(samples) >= 10:
        noise["fail_prob"] = round(sum(s["reply"] != "ok" for s in samples)/len(samples), 3)
    if sweep and sweep.get("oblique_excess") is not None:
        noise["tof_oblique_excess"] = round(clamp(sweep["oblique_excess"], 0., 2.), 3)
    if sweep and sweep.get("scatter_m") is not None:
        # Per-hover scatter beyond the per-reply sensor noise stepsim already draws (about 2 mm per hover).
        noise["tof_obs_sd_m"] = round(math.sqrt(max(0., sweep["scatter_m"]**2-.002**2)), 4)

    end = next((r for r in reversed(rows) if r["event"] == "end"), {})
    return dict(outcome=end.get("outcome"), step_commands=len(samples), ok=len(ok), hfov_assumed_deg=hfov_deg,
                state_yaw_increases_with_cw=yaw_sign > 0, image_signs=signs,
                move_scale={v: stats(x) for v, x in scale.items()}, cross_drift_m=stats([c for _, c in cross]),
                heading_change_on_moves_deg=stats(heading_drift, 2),
                rotation_error_deg={str(v): stats([e for value, e in rotation if value == v], 2)
                                    for v in sorted({value for value, _ in rotation})},
                hfov_from_turns_deg=stats(hfov, 1), speed_after_ok_dm_s=stats(settle, 1),
                sweep=sweep, noise_for_stepsim=noise, by_command=by_command)


def write_analysis(log_path, hfov_deg=54., echo=print):
    log_path = Path(log_path)
    result = analyze(load_log(log_path), hfov_deg)
    (log_path.parent/"analysis.json").write_text(json.dumps(result, indent=1), encoding="utf-8")
    (log_path.parent/"noise.json").write_text(json.dumps(result["noise_for_stepsim"], indent=1), encoding="utf-8")
    echo(f"outcome {result['outcome']}; {result['ok']}/{result['step_commands']} step commands ok")
    echo(f"{'command':<11}{'n':>3}{'ok':>4}{'s':>6}{'d_ex':>8}{'d_ey':>8}{'box h':>8}{'d_tof':>8}{'d_yaw':>7}")
    for name, g in result["by_command"].items():
        def cell(key, fmt):
            width = int(fmt.strip("+").split(".")[0])
            return format(g[key]["median"], fmt) if g[key] else "-".rjust(width)
        echo(f"{name:<11}{g['n']:>3}{g['ok']:>4}{cell('seconds', '6.1f')}{cell('dex', '+8.3f')}"
             f"{cell('dey', '+8.3f')}{cell('dh', '+8.1%')}{cell('dtof', '+8.3f')}{cell('dyaw', '+7.1f')}")
    wrong = [verb for verb, s in result["image_signs"].items() if not s["as_expected"]]
    echo("image response signs: " + ("all as the planner assumes" if not wrong else f"UNEXPECTED for {wrong}"))
    echo(f"move scale: {json.dumps(result['move_scale'])}")
    echo(f"video HFOV from turns: {result['hfov_from_turns_deg']}; sweep: {json.dumps(result['sweep'])}")
    echo(f"noise for stepsim -> {log_path.parent/'noise.json'}: {json.dumps(result['noise_for_stepsim'])}")
    return result


# -- command line ----------------------------------------------------------------------------------
def terminal_confirm(text):
    return input(text).strip().lower() == "fly"


def main(argv=None):
    from .stepsim import RESERVED_SEEDS
    parser = argparse.ArgumentParser(description="Step-response calibration flight (see research/STEP_FLIGHT.md)")
    sub = parser.add_subparsers(dest="mode", required=True)
    rehearse = sub.add_parser("rehearse", help="run the whole flight in the geometric test world; no network")
    fly = sub.add_parser("fly", help="FLY THE AIRCRAFT: supervised step calibration")
    analyze_cmd = sub.add_parser("analyze", help="summarize a flight.jsonl")
    for p in (rehearse, fly):
        p.add_argument("--out", required=True)
        p.add_argument("--class-name", default="cup")
        p.add_argument("--start-m", type=float, default=1.5)
        p.add_argument("--hover-m", type=float, default=1.0)
        p.add_argument("--sweep-m", type=float, default=.80)
        p.add_argument("--height-cm", type=float, default=160.)
        p.add_argument("--repeats", type=int, default=2)
        p.add_argument("--no-sweep", action="store_true")
        p.add_argument("--max-seconds", type=float, default=480.)
    rehearse.add_argument("--seed", type=int, default=60500)
    fly.add_argument("--weights", default="models/yolo11s.pt")
    fly.add_argument("--confidence", type=float, default=.45)
    fly.add_argument("--imgsz", type=int, default=640, help="YOLO input size; small targets are also searched in crops")
    fly.add_argument("--host", default="192.168.10.1")
    fly.add_argument("--mission-pad", action="store_true", help="enable downward mission-pad detection (mon)")
    fly.add_argument("--no-frames", action="store_true", help="do not save observed frames as JPEG")
    analyze_cmd.add_argument("log")
    analyze_cmd.add_argument("--hfov-deg", type=float, default=54.)
    a = parser.parse_args(argv)
    if a.mode == "analyze":
        write_analysis(a.log, a.hfov_deg)
        return 0
    plan = replace(FlightPlan(), label=a.class_name, start_m=a.start_m, hover_m=a.hover_m, sweep_m=a.sweep_m,
                   height_cm=a.height_cm, repeats=a.repeats, sweep=not a.no_sweep, max_seconds=a.max_seconds,
                   mission_pad=getattr(a, "mission_pad", False))
    if Path(a.out).exists():
        parser.error(f"{a.out} already exists; logs are never overwritten, choose a new --out folder")
    if not (.6 <= plan.sweep_m < plan.hover_m <= 1.1 and plan.hover_m+.2 <= plan.start_m <= 2.5):
        parser.error("need 0.6 <= sweep-m < hover-m <= 1.1 and hover-m + 0.2 <= start-m <= 2.5")
    if not (1 <= plan.repeats <= 4 and 60 <= plan.max_seconds <= 600 and 80 <= plan.height_cm <= 200):
        parser.error("need 1-4 repeats, 60-600 s and a height of 80-200 cm")
    if a.mode == "rehearse":
        if a.seed in RESERVED_SEEDS:
            parser.error("seeds 10000-59999 belong to the PID/PPO simulator splits; use 60000 or higher")
        aircraft = SimAircraft(a.seed, plan)
        recorder = Recorder(a.out, aircraft.clock)
        recorder.write(dict(event="start", mode="rehearse", seed=a.seed, plan=asdict(plan),
                            world="objectpilot.stepsim geometric test world, not a flight model"))
        try:
            CalibrationFlight(aircraft, plan, recorder, clock=aircraft.clock).run(confirm=lambda text: True)
        finally:
            recorder.close()
        write_analysis(recorder.path)
        return 0
    if not sys.stdin.isatty():
        parser.error("fly needs an interactive terminal for the pre-flight confirmation")
    aircraft = TelloAircraft(a.host, a.weights, plan.label, a.confidence, a.imgsz)
    recorder = None
    try:
        recorder = Recorder(a.out, save_frames=not a.no_frames)
        recorder.write(dict(event="start", mode="fly", plan=asdict(plan), weights=Path(a.weights).name,
                            confidence=a.confidence))
        CalibrationFlight(aircraft, plan, recorder).run(confirm=terminal_confirm)
    finally:
        aircraft.close()
        if recorder:
            recorder.close()
    if recorder and any(r["event"] == "command" and r["kind"] == "trial" for r in load_log(recorder.path)):
        write_analysis(recorder.path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
