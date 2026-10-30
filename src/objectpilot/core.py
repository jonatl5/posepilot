"""The same timestamped measurements, feature builder and supervisor in sim/replay.

All timestamps use one monotonic clock in seconds. Image coordinates are fractions
of the full frame. A missing range is None, never zero. RC values are channel
inputs, not metres/second. No ground-truth state enters FeatureBuilder.
"""
from dataclasses import dataclass, asdict
from collections import deque
from pathlib import Path
import json
import math
import re
import numpy as np

FEATURE_NAMES = (
    "ex", "ey", "log_size_error", "visual_range_scaled", "forward_range_scaled",
    "forward_valid", "range_associated", "confidence", "detection_valid",
    "frame_age_scaled", "range_age_scaled", "ex_rate_scaled", "log_size_rate_scaled",
    "previous_forward", "previous_yaw", "near_sensor_required",
)
SCHEMA = "objectpilot-v1-16x4"
CONTROL_CONTRACT = "objectpilot-control-v2"
ACTION_NAMES = ("forward", "yaw")

@dataclass
class Config:
    dt: float = 0.1
    history: int = 4
    stand_off_m: float = 0.65
    minimum_m: float = 0.40
    near_entry_m: float = 1.15
    forward_range_max_m: float = 1.20
    range_stale_s: float = 0.40
    frame_stale_s: float = 0.35
    # h_fraction * distance_m, fitted for the printed object bounding box.
    visual_scale_m: float = 0.22
    confidence_min: float = 0.45
    center_gate: float = 0.12
    vertical_gate: float = 0.15
    arrival_x: float = 0.05
    arrival_range_m: float = 0.08
    arrival_hold_s: float = 2.0
    rc_forward_cap: int = 20
    rc_yaw_cap: int = 25
    max_steps: int = 400

    def __post_init__(self):
        for name, value in asdict(self).items():
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a finite positive number")
        if self.history != 4 or type(self.history) is not int:
            raise ValueError("Schema objectpilot-v1-16x4 requires four history ticks")
        if not self.minimum_m < self.stand_off_m < self.near_entry_m <= self.forward_range_max_m:
            raise ValueError("Require minimum < stand-off < near-entry <= forward range")
        if not self.arrival_x < self.center_gate <= .5 or self.vertical_gate > .5:
            raise ValueError("Invalid normalized centering tolerances")
        if self.confidence_min > 1 or self.arrival_range_m >= self.stand_off_m-self.minimum_m:
            raise ValueError("Invalid confidence or arrival tolerance")
        for name in ("rc_forward_cap", "rc_yaw_cap", "max_steps"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")
        if max(self.rc_forward_cap, self.rc_yaw_cap) > 100:
            raise ValueError("RC caps must not exceed 100")

    @classmethod
    def from_json(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))

@dataclass
class Detection:
    cx: float
    cy: float
    width: float
    height: float
    confidence: float
    timestamp: float
    frame_id: int
    label: str = "target"


def policy_contract(config):
    """Record behavior as well as tensor layout; old checkpoints require retraining."""
    return {"schema": SCHEMA, "control_contract": CONTROL_CONTRACT,
            "features": list(FEATURE_NAMES), "action_names": list(ACTION_NAMES),
            "config": asdict(config)}

@dataclass
class RangeSample:
    distance_m: float | None
    timestamp: float
    source: str = "forward_ext_tof"
    status: str = "ok"
    sequence: int | None = None

def parse_ext_tof(reply: str, timestamp: float) -> RangeSample:
    """Parse only SDK EXT tof? responses (mm). Reject telemetry/tof? formats."""
    match = re.fullmatch(r"\s*tof\s+(\d+)\s*", reply)
    if not match:
        return RangeSample(None, timestamp, status="malformed")
    mm = int(match[1])
    if mm in (0, 8192, 65535):
        return RangeSample(None, timestamp, status="out_of_range")
    # Receipt time is not sensor sample time. Stock replies may contain a cache.
    return RangeSample(mm / 1000.0, timestamp, status="sample_age_unknown")

class CustomRangeParser:
    """Reject duplicate/backwards samples; reset parser explicitly after MCU restart."""
    def __init__(self, config=None):
        self.c = config or Config()
        self.previous_sequence = None
        self.reset_required = False

    def parse(self, reply, request_start, response_time):
        if not all(math.isfinite(t) for t in (request_start, response_time)):
            return RangeSample(None, 0., "forward_custom", "invalid_clock")
        match = re.fullmatch(r"\s*oprange (\d+) (\d+) (\d+) ([01]) (\d+)\s*", reply)
        if not match:
            return RangeSample(None, request_start, "forward_custom", "malformed")
        seq, device_ms, mm, valid, age_ms = map(int,match.groups())
        if max(seq, device_ms, age_ms) >= 2**32 or mm > 65535:
            return RangeSample(None, request_start, "forward_custom", "malformed")
        delta = None if self.previous_sequence is None else (seq-self.previous_sequence) % (2**32)
        if delta is not None and delta >= 2**31:
            self.reset_required = True
        fresh_seq = not self.reset_required and (delta is None or 0 < delta < 2**31)
        if fresh_seq: self.previous_sequence = seq
        timestamp = request_start-age_ms/1000
        status = "ok"
        if not fresh_seq: status = "repeated_or_reset"
        elif not valid or mm in (0,8192,65535): status = "sensor_invalid"
        elif not 0 <= response_time-request_start <= .25 or response_time-timestamp > self.c.range_stale_s: status = "stale"
        return RangeSample(mm/1000 if status=="ok" else None,timestamp,"forward_custom",status,seq)

class FeatureBuilder:
    def __init__(self, config=None):
        self.c = config or Config()
        self.reset()

    def reset(self):
        self.frames = deque(maxlen=self.c.history)
        self.last_detection = None
        self.near_required = False

    def build(self, detection, ranging, now, previous_action):
        c = self.c
        f = np.zeros(len(FEATURE_NAMES), dtype=np.float32)
        f[9:11] = 1.0
        valid = detection is not None
        if valid:
            d = detection
            valid = all(math.isfinite(v) for v in asdict(d).values() if isinstance(v, (int, float)))
            valid = valid and 0 <= now-d.timestamp <= c.frame_stale_s
            valid = valid and c.confidence_min <= d.confidence <= 1 and 0 < d.height <= 1 and 0 < d.width <= 1
            valid = valid and 0 <= d.cx <= 1 and 0 <= d.cy <= 1
        dvis = None
        if valid:
            ex, ey = d.cx - .5, d.cy - .5
            dvis = c.visual_scale_m / d.height
            log_size = math.log(dvis / c.stand_off_m)
            f[:4] = (ex, ey, np.clip(log_size / 2, -1, 1), min(dvis / 4, 1))
            f[7:10] = (d.confidence, 1, min((now-d.timestamp)/c.frame_stale_s, 1))
            old = self.last_detection
            if old is not None and d.frame_id != old.frame_id:
                delta = d.timestamp - old.timestamp
                if 0 < delta <= c.frame_stale_s:
                    f[11] = np.clip((d.cx-old.cx)/delta/2, -1, 1)
                    f[12] = np.clip(math.log(old.height/d.height)/delta/4, -1, 1)
            if old is None or d.timestamp > old.timestamp:
                self.last_detection = d
        range_ok = (ranging is not None and ranging.source in ("forward_ext_tof", "forward_custom", "sim_forward")
                    and ranging.status == "ok" and ranging.distance_m is not None
                    and math.isfinite(ranging.distance_m)
                    and 0.02 <= ranging.distance_m <= c.forward_range_max_m
                    and 0 <= now-ranging.timestamp <= c.range_stale_s)
        if range_ok:
            f[4:6] = (ranging.distance_m/c.forward_range_max_m, 1)
            f[10] = (now-ranging.timestamp)/c.range_stale_s
            # Conservative proxy for calibrated beam/box overlap, both axes.
            f[6] = float(valid and abs(f[0]) <= min(c.center_gate, d.width/2)
                         and abs(f[1]) <= min(c.vertical_gate, d.height/2))
        if range_ok or (dvis is not None and dvis <= c.near_entry_m):
            self.near_required = True
        f[13:15] = np.clip(previous_action, -1, 1)
        f[15] = float(self.near_required)
        if not self.frames:
            for _ in range(c.history-1):
                self.frames.append(np.zeros_like(f))
        self.frames.append(f)
        return np.concatenate(self.frames).astype(np.float32)

def latest(observation):
    return np.asarray(observation, dtype=np.float32)[-len(FEATURE_NAMES):]

class PID:
    """Configurable PID, initial integral gains zero (PD baseline). Tune on validation."""
    def __init__(self, config=None, kp=(1.5, 3.5), ki=(0., 0.), kd=(.08, .05)):
        self.c = config or Config()
        self.kp, self.ki, self.kd = map(np.asarray, (kp, ki, kd))
        self.reset()

    def reset(self):
        self.integral = np.zeros(2)
        self.last = None
        self.last_mode = None

    def predict(self, observation):
        f = latest(observation)
        if f[8] < .5:
            self.reset()
            return np.zeros(2, dtype=np.float32)
        mode = bool(f[6] > .5)
        distance = f[4]*self.c.forward_range_max_m if mode else f[3]*4
        error = np.array([distance-self.c.stand_off_m, f[0]])
        if mode != self.last_mode:
            self.last = None  # No derivative kick at the visual/ToF handover.
            self.integral[:] = 0
        self.integral = np.clip(self.integral + error*self.c.dt, -.5, .5)
        rate = np.zeros(2) if self.last is None else (error-self.last)/self.c.dt
        self.last, self.last_mode = error, mode
        return np.clip(self.kp*error + self.ki*self.integral + self.kd*rate, -1, 1).astype(np.float32)

class Supervisor:
    """Always applied after either controller. Fail closed; no autonomous reverse."""
    def __init__(self, config=None):
        self.c = config or Config()
        self.previous = np.zeros(2, dtype=np.float32)

    def apply(self, action, observation, *, battery_ok=True, telemetry_fresh=True, operator_stop=False):
        c = self.c
        try:
            obs = np.asarray(observation, dtype=np.float32)
            a = np.asarray(action, dtype=np.float32)
            if obs.shape != (16*c.history,):
                raise ValueError("Wrong observation schema")
            f = latest(obs)
        except (ValueError, TypeError):
            self.previous[:] = 0
            return self.previous.copy(), "invalid_action_or_state"
        reason = "tracking"
        if a.shape != (2,) or not np.isfinite(a).all() or not np.isfinite(f).all():
            reason = "invalid_action_or_state"
        elif operator_stop or not battery_ok or not telemetry_fresh:
            reason = "operator_or_telemetry_stop"
        elif f[8] < .5 or f[9] >= 1:
            reason = "stale_or_missing_detection"
        elif f[5] > .5 and f[4]*c.forward_range_max_m <= c.minimum_m:
            reason = "front_clearance_stop"  # Independent of visual association.
        elif f[15] > .5 and f[5] < .5:
            reason = "near_range_unavailable"
        if reason != "tracking":
            self.previous[:] = 0
            return self.previous.copy(), reason
        a = np.clip(a, (-0., -1.), (1., 1.))
        a = np.clip(a, self.previous-.3, self.previous+.3)
        if abs(f[0]) > c.center_gate or abs(f[1]) > c.vertical_gate:
            a[0] = 0
            reason = "centering"
        if f[5] > .5:
            # Brake before minimum; this is a candidate margin, pending flight data.
            margin = f[4]*c.forward_range_max_m-c.minimum_m
            a[0] = min(a[0], max(0, margin/.35))
        self.previous = a.astype(np.float32)
        return self.previous.copy(), reason

def rc_channels(action, config=None):
    c = config or Config()
    a = np.asarray(action)
    if a.shape != (2,) or not np.isfinite(a).all():
        return (0, 0, 0, 0)
    return (0, round(float(np.clip(a[0], 0, 1))*c.rc_forward_cap), 0,
            round(float(np.clip(a[1], -1, 1))*c.rc_yaw_cap))
