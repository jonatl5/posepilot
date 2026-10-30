"""One controller path for camera/file dry runs and future measured deployment."""
from dataclasses import asdict
import math
import numpy as np
from .core import Config, FeatureBuilder, PID, Supervisor, latest, rc_channels, SCHEMA, CONTROL_CONTRACT


class ArrivalMonitor:
    """Two-second measured centering/range hold; this is not a velocity estimate."""

    def __init__(self, config):
        self.c = config
        self.since = None
        self.last_time = None
        self.arrived = False

    def update(self, observation, now):
        f, c = latest(observation), self.c
        contiguous = self.last_time is None or 0 <= now-self.last_time <= 2*c.dt+1e-6
        self.last_time = now
        if not contiguous:
            self.since = None
        in_window = (math.isfinite(now) and np.isfinite(f).all() and f[8] > .5
                     and f[5] > .5 and f[6] > .5 and f[9] < 1 and f[10] < 1
                     and abs(f[0]) < c.arrival_x
                     and abs(f[4]*c.forward_range_max_m-c.stand_off_m) < c.arrival_range_m)
        if not in_window:
            self.since = None
        elif self.since is None:
            self.since = now
        if self.since is not None and now-self.since >= c.arrival_hold_s-1e-6:
            self.arrived = True
        return in_window, self.arrived


class ControlGate:
    """Identical post-policy supervision and arrival behavior in sim and replay."""

    def __init__(self, config=None):
        self.c = config or Config()
        self.supervisor = Supervisor(self.c)
        self.arrival = ArrivalMonitor(self.c)

    def apply(self, proposed, obs, now, ranging=None, **supervisor_flags):
        safe, reason = self.supervisor.apply(proposed, obs, **supervisor_flags)
        # A recently received close stock reply can veto motion, but cannot
        # authorize approach or arrival because its sample age is unknown.
        raw_close = (ranging is not None and ranging.source == "forward_ext_tof"
                     and ranging.status == "sample_age_unknown"
                     and ranging.distance_m is not None and math.isfinite(ranging.distance_m)
                     and 0 < ranging.distance_m <= self.c.minimum_m
                     and 0 <= now-ranging.timestamp <= self.c.range_stale_s)
        if raw_close:
            safe[:] = 0
            reason = "front_clearance_stop"
        if reason in ("tracking", "centering"):
            in_window, arrived = self.arrival.update(obs, now)
            if in_window or arrived:
                safe[:] = 0
                reason = "arrived" if arrived else "arrival_hold"
        else:
            self.arrival.since = None
            self.arrival.last_time = now
        # Arrival is terminal until the operator starts a new approach.
        if self.arrival.arrived:
            safe[:] = 0
            reason = "arrived"
        self.supervisor.previous = safe.copy()
        return safe, reason


class ApproachPipeline:
    """Inputs are selected detections, in full-frame normalized coordinates."""

    def __init__(self, config=None, policy=None):
        self.c = config or Config()
        self.policy = policy
        self.reset()

    def reset(self):
        self.builder = FeatureBuilder(self.c)
        self.gate = ControlGate(self.c)
        self.supervisor, self.arrival = self.gate.supervisor, self.gate.arrival
        self.pid = PID(self.c)
        self.previous = np.zeros(2, dtype=np.float32)

    def step(self, detection, ranging, now, **supervisor_flags):
        obs = self.builder.build(detection, ranging, now, self.previous)
        proposed = (self.pid.predict(obs) if self.policy is None
                    else self.policy.predict(obs, deterministic=True)[0])
        safe, reason = self.gate.apply(proposed, obs, now, ranging, **supervisor_flags)
        self.previous = safe.copy()
        return {"schema": SCHEMA, "control_contract": CONTROL_CONTRACT,
                "config": asdict(self.c), "mode": "dry_run",
                "timestamp": now, "detection": asdict(detection) if detection else None,
                "forward_range": asdict(ranging) if ranging else None,
                "observation": obs.tolist(), "proposed_action": np.asarray(proposed).tolist(),
                "safe_action": safe.tolist(), "rc": rc_channels(safe, self.c),
                "supervisor": reason, "arrived": self.arrival.arrived, "sent_rc": False}
