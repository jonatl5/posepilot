"""Stop-and-go approach with discrete Tello SDK commands.

Each step the aircraft hovers, the PC looks at a few frames and ToF replies, the
planner chooses ONE SDK command (forward/back/left/right/up/down in cm, cw/ccw in
degrees), the aircraft executes it and stops, and the loop repeats. Nothing is
streamed, so latency and braking do not enter the decision.

Phases: search (turn in place) -> face (yaw the box to the center, step forward
until ToF is valid) -> sweep (yaw in place; the heading with the smallest ToF
reading is the wall normal) -> lateral (strafe by D*tan(angle) so the aircraft
stands square in front of the print) -> center (strafe on the box offset) ->
final (ToF stand-off) -> verify (a +-10 degree re-check of the wall normal) ->
center/final again -> arrived.

This module only decides; it never talks to an aircraft. objectpilot.stepsim
provides a geometric test world for it.
"""
from dataclasses import dataclass
import math
import statistics

import numpy as np

SDK_RANGES = {"forward": (20, 500), "back": (20, 500), "left": (20, 500), "right": (20, 500),
              "up": (20, 500), "down": (20, 500), "cw": (1, 360), "ccw": (1, 360)}
OPPOSITE = {"forward": "back", "back": "forward", "left": "right", "right": "left",
            "up": "down", "down": "up"}


def check_command(command):
    """Return (verb, value) for a valid SDK move or rotate command; raise ValueError otherwise."""
    parts = command.split()
    if len(parts) != 2 or parts[0] not in SDK_RANGES or not parts[1].isdigit():
        raise ValueError(f"Not a step command: {command!r}")
    low, high = SDK_RANGES[parts[0]]
    value = int(parts[1])
    if not low <= value <= high:
        raise ValueError(f"{command!r} is outside the SDK range {low}-{high}")
    return parts[0], value


@dataclass
class StepObservation:
    """What the planner sees after one hover: one box for the requested label and one range."""
    box: tuple | None          # (cx, cy, w, h) in frame fractions, median over frames
    seen: float                # share of frames in which the label was detected
    tof: float | None          # median valid forward ToF in metres, bench offset removed
    yaw_deg: float | None = None  # IMU yaw from the SDK state stream (port 8890), if available
    conf: float | None = None     # median detector confidence of the chosen boxes


def summarize(frames, tof_replies, label, min_seen=.5, min_tof_valid=.5, tof_offset_m=.02, yaw_deg=None):
    """Collapse one hover's detections and ToF replies into a StepObservation.

    frames: per video frame, a list of (label, cx, cy, w, h, confidence) detections.
    tof_replies: forward ToF readings in metres, None for out-of-range or failed replies.
    yaw_deg: the state stream's yaw after the hover, passed through unchanged.
    """
    boxes, confs = [], []
    for detections in frames:
        own = [d for d in detections if d[0] == label]
        if own:
            best = max(own, key=lambda d: d[5])
            boxes.append(best[1:5])
            confs.append(best[5])
    seen = len(boxes)/max(1, len(frames))
    box = tuple(statistics.median(b[i] for b in boxes) for i in range(4)) if seen >= min_seen else None
    conf = statistics.median(confs) if box is not None else None
    valid = [r for r in tof_replies if r is not None]
    tof = None
    if tof_replies and len(valid)/len(tof_replies) >= min_tof_valid:
        tof = statistics.median(valid) - tof_offset_m
    return StepObservation(box, seen, tof, yaw_deg, conf)


def fit_wall_normal(points, around, search_deg=12.):
    """Least-squares fit of ToF(heading) = D / cos(heading - normal).

    points: (heading_deg, reading_m) pairs from the sweep; infinite readings are ignored.
    Returns (normal_heading_deg, perpendicular_distance_m).
    """
    pts = [(h, r) for h, r in points if math.isfinite(r) and abs(h-around) <= 45]
    headings = np.array([h for h, _ in pts], dtype=float)
    readings = np.array([r for _, r in pts], dtype=float)
    best = None
    for normal in np.arange(around-search_deg, around+search_deg+1e-9, .25):
        g = 1/np.cos(np.radians(headings-normal))
        distance = float(readings@g/(g@g))
        residual = float(((readings-distance*g)**2).sum())
        if best is None or residual < best[0]:
            best = (residual, float(normal), distance)
    return best[1], best[2]


@dataclass
class StepConfig:
    label: str = "cup"
    hfov_deg: float = 54.0          # TT 720p video, from the 32 degree vertical FOV
    stand_off_m: float = .65
    minimum_m: float = .40
    forward_margin_m: float = .08   # a forward step may not end closer than minimum + margin
    forward_overshoot: float = 1.3  # ... nor closer than minimum + 3 cm if it overshoots by this factor
    sweep_entry_m: float = 1.10     # run the wall-normal sweep once ToF reads this or less
    sweep_box_h: float = .26        # box this tall without ToF: one more 20 cm step, then sweep
    near_box_h: float = .33         # box this tall without ToF and no sweep left: stop
    sweep_deg: int = 10
    sweep_limit_deg: float = 70.
    sweep_span_deg: float = 20.     # fit the normal on readings reaching this far on both sides of the minimum
    verify_deg: int = 0             # re-check sweep at the stand-off; 0 disables it (see STEPWISE.md)
    search_deg: int = 30            # a target is always within 15 degrees of some view's center
    confirm_conf: float = .45       # a search candidate must reach this once centered; weaker boxes are candidates
    candidate_checks: int = 2       # centered looks at a weak candidate before it is rejected
    candidate_steps: int = 2        # forward steps toward a small weak candidate before those looks
    candidate_step_m: float = .40
    center_x: float = .05           # yaw-centering tolerance before stepping forward
    lateral_tol_m: float = .04
    max_strafe_m: float = .40       # longer strafes are split and re-checked
    strafe_clearance_m: float = .75  # back off to this ToF range before a strafe of 20 cm or more
    arrival_range_m: float = .05
    arrival_x: float = .06          # centering tolerance when the aircraft is not squared
    hold_observations: int = 3      # consecutive in-tolerance hovers (about 1.8 s)
    hold_slack: float = 1.4         # once holding, tolerances widen by this factor (hysteresis)
    fine_budget: int = 3            # small corrections per axis before its tolerance widens
    budget_widen: float = 1.5       # tolerance factor once the fine budget is spent
    learn_steps: bool = True        # measure and compensate move and turn shortfall in flight
    vertical_limit: float = .33     # |cy - 0.5| that triggers one 20 cm climb or descent
    max_commands: int = 90
    max_observations: int = 250
    max_retries: int = 2            # re-send a command the aircraft answered with "error"
    heading_hold_deg: float = 3.    # with IMU yaw: re-align to the wall normal beyond this
    square_attempts: int = 3
    square: bool = True             # False: yaw-center and approach only (for comparison)


@dataclass
class Decision:
    command: str | None             # None: hover and observe again
    phase: str
    reason: str
    outcome: str | None = None      # set once the approach has ended


class StepPlanner:
    def __init__(self, config=None):
        self.c = config or StepConfig()
        self.tan_half = math.tan(math.radians(self.c.hfov_deg)/2)
        self.phase = "search"
        # Degrees, clockwise positive. Follows the IMU yaw when observations carry it,
        # otherwise the sum of acknowledged rotation commands.
        self.heading = self.imu_heading = 0.
        self.last_yaw = None
        self.queue = []
        self.commands = self.observations = self.retries = 0
        self.search_turned = self.lost_turns = self.waits = self.tof_waits = 0
        self.vertical_moves = self.square_attempts = self.blind_steps = self.hold = self.lost_backoffs = 0
        self.last_ex = 0.
        self.squared = self.verified = False
        self.sweep, self.verify_points = [], []
        self.target_heading = self.normal_heading = self.perp_m = None
        self.last_range = None          # (ToF, box height) of the last reading once squared or final
        self.range_steps = 0
        self.lateral_m = 0.
        self.pending = self.current_tof = None
        self.turn_shortfalls, self.move_scales, self.rejected = [], [], []
        self.candidate_looks = self.candidate_moves = self.fine_steps = self.center_steps = 0
        self.reached_final = False
        self.outcome = None

    # -- outputs ---------------------------------------------------------------
    def _issue(self, command, reason):
        verb, value = check_command(command)
        self.commands += 1
        self.pending = (verb, value, self.heading, self.current_tof)
        return Decision(command, self.phase, reason)

    # -- step compensation learned in flight (November 13: moves ~9% short, turns ~2 degrees short) --
    def _turn_bias(self):
        recent = self.turn_shortfalls[-6:]
        return min(4., max(0., statistics.median(recent))) if len(recent) >= 2 else 0.

    def _move_scale(self):
        recent = self.move_scales[-6:]
        return min(1.25, max(.75, statistics.median(recent))) if len(recent) >= 2 else 1.

    def _learn(self, obs):
        pending, self.pending = self.pending, None
        if pending is None or not self.c.learn_steps:
            return
        verb, value, heading, tof = pending
        if verb in ("cw", "ccw") and obs.yaw_deg is not None and value >= 5:
            self.turn_shortfalls.append(value-abs(self.heading-heading))
        elif verb in ("forward", "back") and self.squared and tof is not None and obs.tof is not None:
            moved = tof-obs.tof if verb == "forward" else obs.tof-tof
            self.move_scales.append(moved/(value/100))

    def _wait(self, reason):
        return Decision(None, self.phase, reason)

    def _end(self, outcome):
        self.outcome = outcome
        self.queue.clear()
        return Decision(None, self.phase, outcome, outcome)

    def _yaw(self, degrees, reason):
        n = max(1, min(360, round(abs(degrees)+self._turn_bias())))
        return self._issue(f"{'cw' if degrees > 0 else 'ccw'} {n}", reason)

    def _yaw_to_ex(self, ex, reason):
        return self._yaw(math.degrees(math.atan(2*ex*self.tan_half)), reason)

    def _sideways(self, metres, reason):
        """Strafe right for positive metres. Below 20 cm: overshoot by 20 cm, then return."""
        verb = "right" if metres > 0 else "left"
        cm = round(abs(metres)*100/self._move_scale())
        if cm >= 20:
            return self._issue(f"{verb} {min(cm, 500)}", reason)
        self.queue.append(f"{OPPOSITE[verb]} 20")
        return self._issue(f"{verb} {cm+20}", reason)

    def _needs_clearance(self, obs, metres):
        # Every strafe has a leg of at least 20 cm (small ones overshoot and return), and strafe
        # drift plus a few degrees of heading error carry it toward the wall: strafe only from
        # strafe_clearance_m. In flight-noise simulation a 31 cm overshoot leg from 0.53 m ended at 0.38 m.
        return obs.tof is not None and obs.tof < self.c.strafe_clearance_m-.05

    def _safe_forward_m(self, tof):
        """Longest forward step that ends at minimum + margin, and at minimum + 3 cm even if it
        overshoots by forward_overshoot (flight spread: SD 11%; a 45 cm step moved 58 cm in simulation)."""
        c = self.c
        return min(tof-c.minimum_m-c.forward_margin_m, (tof-c.minimum_m-.03)/c.forward_overshoot)

    def _back_off(self, obs):
        return self._issue(f"back {max(20, round((self.c.strafe_clearance_m-obs.tof)*100))}",
                           "clear_before_strafe")

    def _forward(self, obs, metres, reason):
        # Compensate the learned shortfall, but never command beyond the safe step.
        command = metres/self._move_scale()
        if obs.tof is not None:
            command = min(command, self._safe_forward_m(obs.tof))
        if command < .20:
            return self._wait("no_safe_forward_step")
        return self._issue(f"forward {min(500, round(command*100))}", reason)

    def applied(self, command, reply):
        """Call after the aircraft replies: "error" re-queues the command, "ok" rotations update the heading."""
        if reply != "ok":
            self.pending = None
            if self.retries < self.c.max_retries:
                self.retries += 1
                self.queue.insert(0, command)
            else:
                self.retries = 0
                self.queue.clear()
            return
        self.retries = 0
        verb, value = check_command(command)
        if self.last_yaw is None:
            if verb == "cw":
                self.heading += value
            elif verb == "ccw":
                self.heading -= value

    def _hold_heading(self):
        """With IMU yaw, undo the small yaw each move adds once it exceeds heading_hold_deg."""
        if self.last_yaw is None or not self.squared or self.normal_heading is None:
            return None
        error = self.normal_heading-self.heading
        if abs(error) >= self.c.heading_hold_deg:
            return self._yaw(error, "hold_heading")
        return None

    # -- decision ---------------------------------------------------------------
    def decide(self, obs):
        c = self.c
        if self.outcome:
            return Decision(None, self.phase, self.outcome, self.outcome)
        self.observations += 1
        if obs.yaw_deg is not None:
            if self.last_yaw is not None:
                self.heading = self.imu_heading+(obs.yaw_deg-self.last_yaw+180) % 360-180
            self.last_yaw, self.imu_heading = obs.yaw_deg, self.heading
        self._learn(obs)
        self.current_tof = obs.tof
        if obs.tof is not None and obs.tof < c.minimum_m+.05:
            self.queue.clear()
            return self._issue("back 20", "too_close")
        if self.queue:
            command = self.queue.pop(0)
            verb, value = check_command(command)
            if not (verb == "forward" and obs.tof is not None and value/100 > self._safe_forward_m(obs.tof)):
                return self._issue(command, "queued")
            self.queue.clear()
        if self.commands >= c.max_commands or self.observations >= c.max_observations:
            return self._end("timeout")
        if obs.box is not None:
            self.last_ex = obs.box[0]-.5
            self.waits = self.lost_backoffs = 0
            if obs.tof is not None and self.phase in ("lateral", "center", "final"):
                self.last_range, self.range_steps = (obs.tof, obs.box[3]), 0
        return getattr(self, "_"+self.phase)(obs, obs.box)

    def _beyond_range(self, box):
        """ToF lost after drifting away: the last reading was far and the box has not grown since."""
        if self.last_range is None or self.range_steps >= 2:
            return False
        tof, height = self.last_range
        return tof >= self.c.sweep_entry_m-.15 and box[3] <= 1.05*height

    def _lost(self, obs):
        if (self.phase in ("center", "final") and obs.tof is not None and obs.tof < self.c.stand_off_m
                and self.lost_backoffs < 2):
            # Lost inside the stand-off: an overshoot puts the print partly out of the frame,
            # and turning away from it there cannot find it again. Back off first.
            self.lost_backoffs += 1
            self.queue.clear()
            return self._issue("back 20", "lost_close_back_off")
        self.waits += 1
        if self.waits <= 2:
            return self._wait("target_missing")
        self.waits = 0
        if self.phase in ("center", "final"):
            # Square-up assumed where the print is; look for it again from the face phase.
            self.squared = self.verified = False
            self.phase = "face"
        if self.lost_turns < 3:
            self.lost_turns += 1
            return self._yaw(20 if self.last_ex >= 0 else -20, "reacquire")
        self.phase, self.search_turned, self.lost_turns = "search", 0, 0
        return self._search(obs, None)

    def _vertical(self, obs, box):
        ey = box[1]-.5
        if abs(ey) > self.c.vertical_limit and self.vertical_moves < 2 and (obs.tof is None or obs.tof > .8):
            self.vertical_moves += 1
            return self._issue("down 20" if ey > 0 else "up 20", "vertical")
        return None

    def _search(self, obs, box):
        c = self.c
        if box is not None and obs.conf is not None and obs.conf < c.confirm_conf:
            # Weak detection (small, at the frame edge or oblique): center it, then require
            # confirm_conf. A rejected candidate's bearing is skipped for the rest of the search.
            ex = box[0]-.5
            bearing = self.heading+math.degrees(math.atan(2*ex*self.tan_half))
            if any(abs((bearing-h+180) % 360-180) < 10 for h in self.rejected):
                box = None
            elif abs(ex) > c.center_x:
                return self._yaw_to_ex(ex, "search_candidate")
            elif box[3] < c.sweep_box_h and self.candidate_moves < c.candidate_steps:
                # Small and weak: far away, and detectors score a print higher once it is larger
                # (cup at 2.5 m: median 0.3; at 1.8 m: 0.5-0.9). Step toward it and look again.
                self.candidate_moves += 1
                return self._forward(obs, c.candidate_step_m, "candidate_approach")
            else:
                self.candidate_looks += 1
                if self.candidate_looks < c.candidate_checks:
                    return self._wait("search_candidate")
                self.rejected.append(bearing)
                self.candidate_looks, self.candidate_moves, box = 0, 0, None
        if box is not None:
            self.phase, self.search_turned, self.lost_turns = "face", 0, 0
            self.candidate_looks = self.candidate_moves = 0
            return self._face(obs, box)
        if self.search_turned >= 360:
            return self._end("not_found")
        self.search_turned += self.c.search_deg
        return self._yaw(self.c.search_deg, "search")

    def _face(self, obs, box):
        c = self.c
        if box is None:
            return self._lost(obs)
        self.lost_turns = 0
        vertical = self._vertical(obs, box)
        if vertical:
            return vertical
        ex = box[0]-.5
        if abs(ex) > c.center_x:
            return self._yaw_to_ex(ex, "face_target")
        self.target_heading = self.heading+math.degrees(math.atan(2*ex*self.tan_half))
        can_square = c.square and not self.squared and self.square_attempts < c.square_attempts
        if obs.tof is not None:
            if can_square:
                if obs.tof > c.sweep_entry_m:
                    return self._forward(obs, .20, "approach")
                return self._start_sweep(obs, box)
            self.phase = "final"
            return self._final(obs, box)
        if box[3] >= c.sweep_box_h:
            # Close but no range: at a steep viewing angle the slant ToF ray reads long and
            # drops out. Turning toward the wall normal shortens it, so look for it there.
            if can_square and (self.blind_steps >= 1 or box[3] >= c.near_box_h):
                return self._start_sweep(obs, box)
            if box[3] >= c.near_box_h:
                return self._end("range_unavailable")
            self.blind_steps += 1
            return self._forward(obs, .20, "approach")
        step = .50 if box[3] < .15 else .40 if box[3] < .22 else .25
        return self._forward(obs, step, "approach")

    def _start_sweep(self, obs, box):
        self.phase, self.sweep = "sweep", []
        self.square_attempts += 1
        return self._sweep(obs, box)

    def _sweep(self, obs, box):
        c = self.c
        self.sweep.append((self.heading, obs.tof if obs.tof is not None else math.inf))
        points = sorted(self.sweep)
        best_heading, best_range = min(points, key=lambda p: p[1])
        if not math.isfinite(best_range):
            # No valid reading yet: try both sides, then approach 20 cm and try once more.
            visited = [h-self.target_heading for h, _ in points]
            for offset in (20, -20, 40, -40):
                if all(abs(offset-v) > 3 for v in visited):
                    return self._yaw(self.target_heading+offset-self.heading, "sweep_explore")
            self.blind_steps = 0
            return self._abort_sweep()
        index = points.index((best_heading, best_range))
        left = index > 0
        right = index < len(points)-1
        if left and right:
            if not (math.isfinite(points[index-1][1]) and math.isfinite(points[index+1][1])):
                # Readings beside the minimum drop out: too far for a reliable fit. Approach, retry.
                self.blind_steps = 0
                return self._abort_sweep()
            # Near the minimum D/cos is flat (1.5% at 10 degrees, below the ~1.3 cm per-hover ToF
            # scatter measured in flight); widen the sweep so the fit sees the curvature.
            low, high = points[0][0], points[-1][0]
            if (high-best_heading < c.sweep_span_deg-2 and math.isfinite(points[-1][1])
                    and abs(high+c.sweep_deg-self.target_heading) <= c.sweep_limit_deg):
                return self._yaw(high+c.sweep_deg-self.heading, "sweep_span")
            if (best_heading-low < c.sweep_span_deg-2 and math.isfinite(points[0][1])
                    and abs(low-c.sweep_deg-self.target_heading) <= c.sweep_limit_deg):
                return self._yaw(low-c.sweep_deg-self.heading, "sweep_span")
            self.normal_heading, self.perp_m = fit_wall_normal(points, best_heading)
            self.squared = True
            self.lateral_m = self.perp_m*math.tan(math.radians(self.target_heading-self.normal_heading))
            self.phase = "lateral"
            delta = self.normal_heading-self.heading
            if abs(delta) >= .5:
                return self._yaw(delta, "face_wall_normal")
            return self._lateral(obs, box)
        goal = (max(h for h, _ in points)+c.sweep_deg if not right
                else min(h for h, _ in points)-c.sweep_deg)
        if abs(goal-self.target_heading) > c.sweep_limit_deg:
            return self._abort_sweep()
        return self._yaw(goal-self.heading, "sweep")

    def _abort_sweep(self):
        self.phase, self.squared = "face", False
        delta = self.target_heading-self.heading
        if abs(delta) >= 1:
            return self._yaw(delta, "sweep_failed")
        return self._wait("sweep_failed")

    def _lateral(self, obs, box):
        """Strafe the planned D*tan(angle) in chunks until the box is well inside the frame."""
        c = self.c
        if abs(self.lateral_m) < c.lateral_tol_m or (box is not None and .1 < box[0] < .9):
            self.phase = "center"
            return self._center(obs, box)
        if self._needs_clearance(obs, self.lateral_m):
            return self._back_off(obs)
        chunk = self.lateral_m/math.ceil(abs(self.lateral_m)/c.max_strafe_m)
        self.lateral_m -= chunk
        return self._sideways(chunk, "square_lateral")

    def _lateral_error(self, obs, box):
        depth = obs.tof if obs.tof is not None else self.perp_m
        return (box[0]-.5)*2*self.tan_half*depth

    def _center(self, obs, box):
        c = self.c
        if box is None:
            return self._lost(obs)
        hold = self._hold_heading()
        if hold:
            return hold
        error = self._lateral_error(obs, box)
        if abs(error) > self._lateral_tol():
            if self._needs_clearance(obs, error):
                return self._back_off(obs)
            if self.reached_final:
                self.center_steps += 1
            return self._sideways(max(-c.max_strafe_m, min(c.max_strafe_m, error)), "center_lateral")
        self.phase = "final"
        return self._final(obs, box)

    def _lateral_tol(self, holding=False):
        """Lateral tolerance: wider while holding (hysteresis) and after fine_budget corrections."""
        c = self.c
        return c.lateral_tol_m*(c.hold_slack if holding else 1.)*(c.budget_widen if self.center_steps >= c.fine_budget else 1.)

    def _range_tol(self, holding=False):
        c = self.c
        return c.arrival_range_m*(c.hold_slack if holding else 1.)*(c.budget_widen if self.fine_steps >= c.fine_budget else 1.)

    def _verify(self, obs, box):
        """At the stand-off: readings at 0, +d and -d degrees refit the wall normal."""
        c = self.c
        self.verify_points.append((self.heading, obs.tof if obs.tof is not None else math.inf))
        if len(self.verify_points) == 1:
            return self._yaw(c.verify_deg, "verify_square")
        if len(self.verify_points) == 2:
            return self._yaw(-2*c.verify_deg, "verify_square")
        self.verified = True
        self.phase = "center"
        center = self.verify_points[0][0]
        if sum(math.isfinite(r) for _, r in self.verify_points) == 3:
            self.normal_heading, _ = fit_wall_normal(self.verify_points, center, search_deg=1.5*c.verify_deg)
        else:
            self.normal_heading = center
        return self._yaw(self.normal_heading-self.heading, "verify_square")

    def _final(self, obs, box):
        c = self.c
        if box is None:
            return self._lost(obs)
        self.reached_final = True
        if self.squared:
            hold = self._hold_heading()
            if hold:
                self.hold = 0
                return hold
            if abs(self._lateral_error(obs, box)) > self._lateral_tol(holding=True):
                self.phase, self.hold = "center", 0
                return self._center(obs, box)
        else:
            ex = box[0]-.5
            if abs(ex) > c.arrival_x:
                self.hold = 0
                return self._yaw_to_ex(ex, "center_yaw")
        if obs.tof is None:
            self.tof_waits += 1
            if self.tof_waits >= 2 and self._beyond_range(box):
                # Strafe drift can carry the aircraft past the ~1.2 m ToF range; the last
                # reading of at least 0.95 m bounds a 20 cm step to at least 0.75 m.
                self.tof_waits, self.range_steps = 0, self.range_steps+1
                return self._issue("forward 20", "range_beyond_tof")
            if self.tof_waits <= 3:
                return self._wait("range_missing")
            return self._end("range_unavailable")
        self.tof_waits = 0
        error = obs.tof-c.stand_off_m
        if abs(error) <= self._range_tol(holding=self.hold > 0):
            if self.squared and c.verify_deg and not self.verified:
                self.phase, self.verify_points, self.hold = "verify", [], 0
                return self._verify(obs, box)
            self.hold += 1
            if self.hold >= c.hold_observations:
                return self._end("arrived")
            return self._wait("arrival_hold")
        self.hold = 0
        if abs(error) < .25:
            self.fine_steps += 1
        if error >= .40:
            return self._forward(obs, min(.40, error-.20), "final_approach")
        if error >= .20:
            return self._forward(obs, error, "final_approach")
        # Less than one 20 cm step: always move away from the wall first. Both legs fall short
        # by the learned scale s, so the net move s*(a-b) is set to the error.
        cm = round(abs(error)*100/self._move_scale())
        if error > 0:
            self.queue.append(f"forward {cm+20}")
            return self._issue("back 20", "final_fine")
        self.queue.append("forward 20")
        return self._issue(f"back {cm+20}", "final_fine")
