"""Independent command-age watchdog for a future qualified flight adapter.

No CLI or Tello transport instantiates this class. The injected sink must provide
bounded, nonblocking send_rc(tuple) and request_land() methods. It must serialize
its own SDK acknowledgements. There is deliberately no takeoff operation here.
"""
from dataclasses import asdict, dataclass
import math
import threading
import time
from .core import Config, SCHEMA, CONTROL_CONTRACT, rc_channels


@dataclass(frozen=True)
class Telemetry:
    timestamp: float
    battery_pct: float
    height_cm: float


class CommandDispatcher:
    def __init__(self, sink, config=None, *, clock=time.monotonic,
                 command_timeout=.35, heartbeat_timeout=.5, fault_land_after=3.):
        self.sink, self.c, self.clock = sink, config or Config(), clock
        if not all(math.isfinite(v) and v > 0 for v in
                   (command_timeout, heartbeat_timeout, fault_land_after)):
            raise ValueError("Watchdog deadlines must be finite and positive")
        self.command_timeout = command_timeout
        self.heartbeat_timeout = heartbeat_timeout
        self.fault_land_after = fault_land_after
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.worker = None
        self.armed = self.tracking = self.land_requested = False
        self.telemetry = None
        self.proposal = None
        self.heartbeat_at = -math.inf
        self.fault_since = None
        self.reason = "disarmed"

    def update_telemetry(self, telemetry):
        with self.lock:
            self.telemetry = telemetry

    def heartbeat(self):
        with self.lock:
            self.heartbeat_at = self.clock()

    def _telemetry_ok(self, now):
        t = self.telemetry
        return (t is not None and all(math.isfinite(v) for v in asdict(t).values())
                and 0 <= now-t.timestamp <= .75 and 20 < t.battery_pct <= 100
                and 30 <= t.height_cm <= 150)

    def arm_after_takeoff(self):
        """Integration hook only, after separately supervised takeoff/qualification."""
        with self.lock:
            now = self.clock()
            if not self._telemetry_ok(now) or self.telemetry.battery_pct < 40:
                raise ValueError("Fresh airborne telemetry and at least 40% battery required")
            if self.land_requested:
                raise ValueError("Create a new dispatcher after confirmed landing")
            self.armed = True
            self.tracking = False
            self.heartbeat_at = now
            self.proposal = None

    def resume(self):
        with self.lock:
            if not self.armed or self.land_requested:
                raise ValueError("Dispatcher is not available for tracking")
            self.tracking = True

    def hold(self):
        with self.lock:
            self.tracking = False
            self.proposal = None
            if self.armed:
                self.sink.send_rc((0, 0, 0, 0))

    def submit(self, row):
        """Accept only the canonical pipeline's finite, bounded safe output."""
        try:
            timestamp = float(row["timestamp"])
            action = tuple(float(v) for v in row["safe_action"])
            observation = tuple(float(v) for v in row["observation"])
            valid = (row["schema"] == SCHEMA and row["control_contract"] == CONTROL_CONTRACT
                     and row["config"] == asdict(self.c)
                     and len(action) == 2 and len(observation) == 64
                     and all(math.isfinite(v) for v in (*action, *observation, timestamp))
                     and 0 <= action[0] <= 1 and -1 <= action[1] <= 1
                     and tuple(row["rc"]) == rc_channels(action, self.c))
        except (TypeError, ValueError, KeyError):
            valid = False
        with self.lock:
            if not valid:
                self.proposal = None
                raise ValueError("Invalid canonical pipeline command")
            self.proposal = (timestamp, rc_channels(action, self.c), row.get("supervisor", "invalid_state"))

    def _land(self):
        self.tracking = False
        if not self.land_requested:
            # Latch before calling I/O: a failing adapter must not cause retries.
            self.land_requested = True
            self.sink.request_land()

    def tick(self):
        with self.lock:
            if not self.armed:
                return (0, 0, 0, 0)
            now = self.clock()
            zero = (0, 0, 0, 0)
            fault = None
            if not math.isfinite(now) or not self._telemetry_ok(now):
                fault = "telemetry_stop"
            elif not 0 <= now-self.heartbeat_at <= self.heartbeat_timeout:
                fault = "operator_heartbeat_lost"
            elif self.tracking and (self.proposal is None or
                    not 0 <= now-self.proposal[0] <= self.command_timeout):
                fault = "command_stale"
            elif self.tracking and self.proposal[2] not in (
                    "tracking", "centering", "arrival_hold", "arrived"):
                fault = "supervisor_hold"
            if fault:
                if self.fault_since is None:
                    self.fault_since = now
                self.reason = fault
                try:
                    self.sink.send_rc(zero)
                finally:
                    if fault == "telemetry_stop" or now-self.fault_since >= self.fault_land_after:
                        self._land()
                return zero
            self.fault_since = None
            command = self.proposal[1] if self.tracking and not self.land_requested else zero
            self.reason = "tracking" if command != zero else "hold"
            self.sink.send_rc(command)
            return command

    def start(self):
        if self.worker is not None:
            raise RuntimeError("Dispatcher already started")
        def run():
            try:
                while not self.stop_event.wait(self.c.dt):
                    self.tick()
            except Exception:
                with self.lock:
                    self.reason = "dispatch_error"
                    if self.armed:
                        try:
                            self.sink.send_rc((0, 0, 0, 0))
                        finally:
                            self._land()
        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()
        return self

    def close(self):
        self.stop_event.set()
        if self.worker:
            self.worker.join(timeout=1.)
        with self.lock:
            if self.armed:
                try:
                    self.sink.send_rc((0, 0, 0, 0))
                finally:
                    self._land()
