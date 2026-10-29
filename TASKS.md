# objectPilot tasks

Updated October 29, 2026. Printed-wall targets, fixed altitude, forward/yaw approach. Dates are Pacific Time. Earlier MediaPipe/person-following plans are superseded for active implementation.

## Completed preparation

- [x] Original planning and move to TT with laptop compute.
- [x] Select wall prints for first demo; remove people/night operation from active scope.
- [x] Python package, YOLO adapter, video/webcam dry runs, target confirmation and latest frame.
- [x] Shared versioned 64-value state, PID/PPO supervisor, proximity/staleness/source checks.
- [x] Seeded Gymnasium environment, PPO training/save/load and evaluation.
- [x] Data extraction, group split, explicit reviewed labels and dataset preparation.
- [x] Official ESP32 source inspection and range-age extension generator.
- [x] Document collection procedure and observation/action contracts.
- [x] Publish Rapid prototypes with recorded simulation and diagnostic episode data.
- [x] Add an offline simulator demo, installation/run guide and no-hardware CI workflow.

## Before November 3

- [ ] A: review firmware, board toolchain and factory restoration; prepare bench procedure.
- [ ] B: choose actual classes and prints; record grouped videos, annotate and benchmark.
- [ ] B: fit per-print/per-camera visual scales using separate calibration samples.
- [ ] C: tune PID and train at least three PPO seeds; select checkpoints on validation only.
- [ ] C: expand independent range delay, burst loss and false-target scenarios beyond current randomization.
- [ ] D: show detector dry run and synthetic closed loop separately, with measured timing and limits.
- [ ] D: update the public progress log when real printed-target or hardware evidence becomes available.

## November 4–7 arrival qualification

- [ ] Activate and record TT/ESP32 versions; verify factory restore procedure.
- [ ] Measure forward range against a tape; distinguish downward telemetry.
- [ ] Compile generated firmware for the delivered board; verify custom query, validity, sequence and age.
- [ ] If needed, qualify external sensor voltage, I2C address ownership, mount and weight.
- [ ] Measure camera calibration, optical overlap, body offsets, stream and query latency.
- [ ] Implement one flight dispatcher, command watchdog, battery/telemetry freshness and manual takeover.
- [ ] Validate command signs, stop/land and link-loss behavior before autonomous flight.
- [ ] Measure low-speed response, braking, drift and battery effects; update simulator.

## Integration and final evaluation

- [ ] November 8–19: supervised PID printed-wall approach; TT-specific videos and detector validation.
- [ ] Add bounded search/recovery only after clearance qualification; routes are not required.
- [ ] November 10 noon: Peer Review 1.
- [ ] November 20–December 3: tuned PID versus nominal/randomized PPO, shared held-out conditions.
- [ ] If qualified, at least 10 paired flight trials/controller; report uncertainty and interventions.
- [ ] December 3: feature freeze and choose most reliable demo controller.
- [ ] December 4–9: rehearse; December 10: final demonstration.
- [ ] December 15 noon: final report and Peer Review 2.

No autonomous flight, firmware flashing, real dataset collection or hardware results are marked complete.
