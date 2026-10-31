# objectPilot task list

The canonical requirements are in [PRD.md](PRD.md). A checked software task is not hardware qualification.

## Completed software preparation

- [x] Preserve the team route: stationary wall prints, YOLO, forward ToF, PID/PPO and a shared 64-value contract.
- [x] Import the linked starter source, existing simulator and data/firmware workflows with provenance.
- [x] Keep the original 30-scenario diagnostic CSVs and trajectory unchanged.
- [x] Integrate marker bench inputs, latest-frame video, camera calibration and grounded capture into the canonical package.
- [x] Add measured arrival hold, stock-ToF freshness handling and close-range vetoes.
- [x] Add a command-age/operator-heartbeat watchdog scaffold with fake-I/O checks.
- [x] Add schema, timestamp, arrival, marker, range and replay regression tests.
- [x] Provide an English data plan, hardware gates, control explanation, source archive and synchronized repository/website PRD.

## Before delivery

- [ ] Record and label real wall prints, distractors and negative clips.
- [ ] Choose two or three target classes from measured pretrained-detector reliability.
- [ ] Fit per-print visual scale on calibration-only data and benchmark held-out detection.
- [x] Tune PID on tuning seeds 40000–40099 and select on validation seeds 30000–30099; reserve final test seeds 50000–50099 (simulation only, see [CONTROLLER_TUNING.md](research/CONTROLLER_TUNING.md)).
- [x] Train three control-v2 PPO seeds at two training budgets and validate them on the same 100 scenarios.
- [ ] Decide the three simulator findings that gains cannot fix: hold resets on brief detection loss, the near-entry latch against the ToF range, and supervisor creep at the minimum clearance.
- [ ] Re-tune gains and re-select checkpoints on a measured-response simulator; run the reserved final test once at feature freeze.

## Hardware and flight

- [ ] Identify exact aircraft, firmware, expansion board and sensor orientation.
- [ ] Verify actual TT video, exposure-to-host delay and wireless behavior.
- [ ] Compile/test firmware and validate sequence, sample age, invalid readings and query routing.
- [ ] Measure physical command response, stopping drift, body/sensor offsets and battery effects.
- [ ] Integrate one qualified flight sink, operator controls, confirmed landing and onboard link-loss behavior.
- [ ] Complete repeated supervised PID approaches and record all failures/interventions.
- [ ] Compare calibrated controllers only after the hardware gates pass.
- [ ] Evaluate optional external ToF only if the built-in sensor measurements justify it.

## Delivery

- [ ] Select the most reliable qualified configuration at feature freeze.
- [ ] Preserve model/configuration, calibration, validation/test splits, logs and limitations.
- [ ] Keep dated evidence, progress pages and PRD statuses aligned after each implementation change.

## TT control alignment audit

- [x] Compare official TT SDK and public TT/Tello projects with our detection/action contract.
- [x] Reconcile both contributors: shared target confirmation, measured arrival gate, RC quantization and calibration config.
- [x] Version controller behavior and reject incompatible PPO checkpoints.
- [x] Retire the conflicting root dependency snapshot; retain the original under research.
- [x] Add cross-path regression tests and Windows/Linux CI with resolved environment artifacts.
- [x] Retrain multiple PPO seeds under control-v2 (three seeds, two budgets, validation seeds 30000–30099).
- [ ] Qualify actual TT response.
