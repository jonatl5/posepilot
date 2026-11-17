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

## Platform research (aircraft delivered by November 5, 2026)

- [x] Summarize TT control paths, sensor/telemetry interfaces and their consequences in [TT_PLATFORM.md](research/TT_PLATFORM.md).
- [x] Choose the recording route: TT camera through the Tello app during manual flight, with a per-clip shot list in [RECORDING_PLAN.md](research/RECORDING_PLAN.md).
- [ ] Record the delivered unit's aircraft firmware, `sdk?`, `sn?`, `hardware?`, `EXT version?`, app version and expansion-controller mode (`objectpilot.tof_bench` stores the SDK replies in `session.json`).
- [x] Add read-only identity queries (`sdk?`, `sn?`, `hardware?`, `battery?`, `wifi?`, `wifiversion?`) to the `GroundLink` allowlist with tests.

## Perception data (TT camera, see [RECORDING_PLAN.md](research/RECORDING_PLAN.md))

- [x] Session 0: app recordings are 1280×720 at 11–30 fps; camera tilts down about 10.5° ([TT_MEASUREMENTS.md](research/TT_MEASUREMENTS.md)).
- [x] First screening: the backpack print fails with every pretrained size; the cup print passes with YOLO11s from 0.5 to 3.0 m.
- [x] Stand calibration ladder for cup print A: k = 0.343 (YOLO11s), recorded in `configs/tt_cup_measured.json` with `vertical_gate` 0.30.
- [ ] Re-mount the cup center at 1.47 m and confirm framing at 0.65, 1.2 and 3.0 m.
- [ ] Screen at least one more class and a held-out cup print B; record the cup A coverage, negatives and distractors listed in the recording plan status section.
- [ ] Sessions 3–4: record the room R1 (train) and room R2 (validation) flight sets, negatives, distractors and occlusions.
- [ ] Session 5: record the held-out print family, label every test frame and benchmark the pretrained detector.
- [ ] Fine-tune only if the held-out benchmark shows a useful gap.

## Simulation and controller

- [x] Tune PID on tuning seeds 40000–40099 and select on validation seeds 30000–30099; reserve final test seeds 50000–50099 (simulation only, see [CONTROLLER_TUNING.md](research/CONTROLLER_TUNING.md)).
- [x] Train three control-v2 PPO seeds at two training budgets and validate them on the same 100 scenarios.
- [ ] Decide the three simulator findings that gains cannot fix: hold resets on brief detection loss, the near-entry latch against the ToF range, and supervisor creep past the minimum clearance.
- [ ] Re-tune gains and re-select checkpoints on a measured-response simulator; run the reserved final test once at feature freeze.

## Hardware and flight

- [ ] Identify exact aircraft, firmware, expansion board and sensor orientation.
- [ ] Verify actual TT video, exposure-to-host delay and wireless behavior in a laptop SDK session (the Tello app recordings cannot measure delay).
- [x] Write the labeled forward-ToF bench procedure and tool ([TOF_BENCH.md](research/TOF_BENCH.md), `objectpilot.tof_bench`).
- [ ] Run the ToF bench on whiteboard, cup print and dark print, then decide `forward_range_max_m`, `near_entry_m` and whether custom firmware is needed.
- [ ] Compile/test firmware and validate sequence, sample age, invalid readings and query routing.
- [x] Write the step calibration flight tool: allowlist, operator confirmation, abort-and-land rules, test-world rehearsal and analysis ([STEP_FLIGHT.md](research/STEP_FLIGHT.md), `objectpilot.step_flight`).
- [x] Fly the step calibration once and re-run `objectpilot.stepsim --noise` with its `noise.json` (November 13: 72/72 commands, planner 96.7% with measured noise).
- [x] Connect `StepPlanner` to the aircraft through the guarded flight session (`objectpilot.step_approach`); full-loop simulation 97.3% with measured noise ([STEP_APPROACH.md](research/STEP_APPROACH.md)).
- [x] Fly the planner approach (November 13): 1.5 m straight and 30° to the side arrived; the oblique start over-corrected (17 fine corrections); a search among distractors missed a cup seen at frame edges below 0.45 confidence ([STEP_APPROACH.md](research/STEP_APPROACH.md)).
- [x] Reduce over-correction and fix the search: sweep span, no verify refit, hysteresis, correction budget, step learning, 30° search with candidates, 960 px inference, strafe-clearance and overshoot-safe forward rules; stand-off 0.50 m (simulated 96.3% with flight noise).
- [ ] Fly the changed planner: 1.5 m straight, 30° to the side, then a search with distractors.
- [x] Spoken target: laptop microphone, offline faster-whisper, English/Chinese vocabulary with synonyms and homophones, asked after the checklist and before takeoff (`objectpilot.voice`, `step_approach fly --listen`).
- [x] Replace free Whisper transcription with Vosk grammar keyword recognition and WASAPI exclusive capture after two voice flights misheard "cup" (6/6 recorded clips now).
- [x] Voiced-sound endpointing (keyboard clicks ignored), tiled detection for far or small targets, approach-to-confirm for weak search candidates (November 15 evening failures).
- [x] Read the raw microphone through a callback (blocking reads dropped a third of the samples) and recognize the whole recording (November 16, `approach-voice-06`).
- [ ] Fine-tune YOLO11s on TT-stream frames of the cup and bottle prints for starts beyond 2.5 m.
- [x] Fly with `--listen`: "cup" recognized at confidence 1.0 on the second attempt, then arrived (November 16, `approach-voice-07`).
- [ ] Try the spoken target with other voices, in the demo room, and with propeller noise nearby.
- [ ] Screen detection against viewing angle (0–65°) and the bottle and third prints on the stand.
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
