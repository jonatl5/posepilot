# Observation v1 and control behavior v2

`src/objectpilot/core.py` is the executable contract. Schema `objectpilot-v1-16x4` fixes order, scaling and history. Behavior version is `objectpilot-control-v2`. Checkpoints record the entire Config, ordered features and action names; the loader rejects mismatches and checks the actual Box spaces. Old behavior checkpoints must be retrained. Default cycle is 10 Hz. Four chronological 16-value observations form a 64-value float32 vector. Initial history is zero padded. Scaling is fixed; no VecNormalize file.

| Index | Feature | Encoding |
|---|---|---|
| 0, 1 | ex, ey | Normalized box center minus 0.5 |
| 2 | log size error | log(d_visual / stand_off) / 2, clipped to ±1 |
| 3 | visual distance | k / bbox height / 4 m, clipped 0–1 |
| 4 | forward distance | ToF / configured maximum (default 1.2 m); zero placeholder when unavailable |
| 5 | forward valid | Correct source, valid finite range, fresh timestamp |
| 6 | range associated | Both-axis box/optical-center overlap proxy plus center gates |
| 7, 8 | confidence, detection valid | Confidence and validity mask |
| 9, 10 | frame age, range age | Age / timeout, clipped 0–1; unavailable = 1 |
| 11, 12 | ex rate, log size rate | Per-second derivatives / 2 and / 4; no repeated-frame differentiation |
| 13, 14 | previous safe actions | Supervisor outputs, not raw policy requests |
| 15 | near sensor required | Latched on near vision or valid forward ToF; reset per episode |

PPO proposes `[forward, yaw]` in `[-1,1]^2`. The supervisor forbids reverse for the wall demo, clips yaw, limits action changes and rejects invalid data. Mapping: `rc 0 round(20*forward) 0 round(25*yaw)`. RC values are channel inputs, not m/s. Positive yaw turns toward a right-of-image target in simulation; verify the hardware sign. Aircraft stabilization holds altitude; no policy altitude channel.

Implemented: target confirmation, centering, approach, near-sensor-required hold and simulated arrival. Automatic takeoff, room search, routes, recovery turns and a live command dispatcher remain explicit integration tasks. Replay with no target produces zero motion.

Detection records carry normalized boxes, class, confidence, frame ID and timestamp. RangeSample carries meters or null, source, status, timestamp and optional sequence. All times must share a host monotonic clock, or one video timebase for replay. MCU millis cannot be compared directly with host time. Factory query receipt does not prove sample freshness.

The environment generates synthetic Detection and RangeSample records; rendered rectangles do not validate YOLO. Only reward/info may use ground truth. Simulation and replay share `TargetLock` and `ControlGate`, including zero output during a measured two-second hold and a latched arrival. Simulator success additionally requires true clearance/centering and residual speed below 0.04 m/s at that measured arrival; otherwise it terminates as `arrival_unverified`. Simulator dynamics consume quantized RC values scaled to nominal RC=20/25 gains. Measured arrival, minimum-distance violation, boundary exit or prolonged loss terminate; time limit truncates.

Replay JSONL includes source timestamps, detections, range, observations, proposed/safe actions, RC values, reason and inference duration. Webcam timestamps mean host decode receipt, not exposure time. File replay follows video time and samples about 10 Hz; it is not real-time latency evidence.

Starting assumptions: 0.65 m stand-off, 0.40 m minimum, 0.35 s image timeout, 0.40 s range timeout. Actual braking, body offsets, camera latency, independent range cadence, battery response and extrinsics need measurement. Both-axis overlap remains a proxy, not proof that a beam hit a 3D object. The first task is restricted to wall prints.

All train/evaluate/demo/replay CLIs accept `--config`. The exact protocol/project and contributor comparison is in [TT_CONTROL_AUDIT.md](TT_CONTROL_AUDIT.md). Replay rows now include a separate control-contract version alongside the unchanged 64-value observation schema.

PID gains are controller parameters outside this contract: `PIDGains` defaults to the starting gains, `--gains` loads a selected file in evaluate, demo and replay, and every PID row/summary records `pid_gains`. Seed ranges partition the simulator scenarios (development 10000–10029 and 20000–20029, validation 30000–30099, tuning 40000–40099, final test 50000–50099 reserved until feature freeze); see [CONTROLLER_TUNING.md](CONTROLLER_TUNING.md).
