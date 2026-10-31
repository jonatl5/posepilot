# objectPilot Product Requirements

Controller tuning edition · October 31, 2026

Aligned with the team's October 29, 2026 Revision 2 PRD. [Source PRD](https://docs.google.com/document/d/1kxEY6A_N0bDsK6_wongQXkz956Aoth89/edit).

Generated from docs/data/prd.json. Run npm run build:docs after editing the source.

## Purpose and scope

Recognize a stationary printed object image on a wall, center it in the RoboMaster TT camera, approach slowly and hold a measured stand-off. A laptop runs YOLO11n, a PID baseline or PPO policy, and an independent supervisor. The ESP32 expansion supplies forward infrared time-of-flight measurements.

The primary demonstration uses matte wall prints, normal indoor lighting and fixed altitude. Physical three-dimensional objects require a separate validation stage. The aircraft is still awaiting delivery; no real TT flight or sensor qualification is claimed.

The repository now contains the team starter implementation and aligned additions. YOLO wall prints remain the demonstration target. ArUco/red-card inputs are bench fixtures using the same 64-value contract, PID/PPO interface and supervisor.

A follow-up audit of both contributors and public TT/Tello projects aligns simulation and camera replay under objectpilot-control-v2. Observation layout remains 16 x 4; old PPO checkpoints require retraining.

An October 31 follow-up tunes the PID gains on tuning seeds, selects on validation seeds, trains three control-v2 PPO seeds at two budgets and reserves seeds 50000–50099 for the frozen final comparison. Three simulator findings await a team decision.

| Area | Requirement |
| --- | --- |
| Primary target | Two or three common-object categories depicted in matte wall prints; select classes through measured detector reliability. |
| Actions | Forward and yaw only. Lateral and throttle outputs stay zero; onboard flight stabilization holds altitude. |
| Provisional distances | Wall-track simulation: 0.65 m stand-off and 0.40 m minimum. Distances are unqualified settings, not approved physical clearances. |
| Arrival | Fresh associated forward range, horizontal centering within 0.05 frame widths, range within 0.08 m of stand-off and a two-second stable hold. |
| Not in the first demonstration | Moving targets, person following, identity recognition, night flight, policy-controlled altitude, onboard neural inference or landing on the target. |


## Functional requirements

### F1 · Video

Priority: Must.

Newest-frame buffering with frame ID and host receive timestamp; measure capture-to-host delay separately.

Current evidence: Latest-frame webcam/synthetic/Tello decoder integrated; actual TT stream qualification pending.

### F2 · Detection

Priority: Must.

YOLO detection of the selected printed-object class; output normalized box, confidence and timestamps.

Current evidence: YOLO integration exercised; printed-target accuracy pending.

### F3 · Selection

Priority: Must.

Three fresh-frame confirmation, same-class association and explicit loss; reject silent identity switches.

Current evidence: Three-frame target confirmation shared by simulator and camera replay; real distractor tests pending.

### F4 · State

Priority: Must.

One versioned 16-feature schema and four-tick history shared by simulation, replay and deployment.

Current evidence: One 16 x 4 observation schema plus explicit control-v2 behavior version; feature/action order and actual policy tensor spaces checked.

### F5 · Policy

Priority: Must.

Deterministic PPO inference on the laptop; save model, config, scaling, schema and seed together.

Current evidence: Shared JSON config accepted by train/evaluate/demo/replay; old behavior checkpoints rejected; three control-v2 seeds trained at 100k and 1M transitions and validated on 100 scenarios, best 85/100; checkpoint freeze pending hardware.

### F6 · Baseline

Priority: Must.

Configurable PID using the same observation and supervisor; tune on validation scenarios.

Current evidence: Gains tuned on tuning seeds and selected on validation seeds, configs/pid_gains_tuned.json, 92/100 against 87/100 for the starting gains; hardware response calibration pending.

### F7 · Control

Priority: Must.

Two channels, forward and yaw; zero lateral/throttle in the initial design. RC values require response calibration.

Current evidence: Same integer RC mapping and cap response in simulator and camera output; physical response calibration pending.

### F8 · Arrival

Priority: Must.

Fresh associated ToF, centering/range tolerances and two-second hold; then terminate the approach and maintain hover.

Current evidence: One measured arrival/zero-output gate in simulation and replay; simulator truth separately audits physical success.

### F9 · Supervisor

Priority: Must.

Reject stale/missing detections, invalid actions, stale telemetry and near-range loss. Close raw range overrides association.

Current evidence: Stale input, near-range loss, invalid state and raw proximity guards covered by software checks; simulation shows a policy can creep to the minimum clearance without hysteresis, pending a control-v3 decision.

### F10 · Flight gate

Priority: Must.

Single dispatcher, operator takeover, command watchdog and tested link-loss behavior before autonomous flight.

Current evidence: Independent command/heartbeat watchdog scaffold added with injected test I/O; no live flight launcher or hardware qualification.

### F11 · Dry run

Priority: Must.

Camera/video pipeline produces logs and proposed bounded RC values without sending flight commands.

Current evidence: Ground-only file/webcam/synthetic/Tello entry point; no takeoff or RC send path.

### F12 · Logging

Priority: Must.

Record source/receipt timestamps, detections, range/status/age, features, proposed/safe actions, model/config and outcome.

Current evidence: Diagnostic outputs available; hardware provenance pending.

### F13 · Simulator

Priority: Must.

Planar wall-target environment with first-order response, action/observation delay, noise and dropout; seeded reset.

Current evidence: Planar simulator and initial comparisons completed; dropout hold resets and the near-entry/ToF-range margin recorded as open findings.

### F14 · Calibration

Priority: Must.

Fit per-print/per-camera visual scale with separate calibration data; invalidate after resizing/camera changes.

Current evidence: Per-print visual-scale workflow and intrinsic-calibration utility included; actual measurements pending.

### F15 · Range

Priority: Must.

Separate forward ToF from downward telemetry; units, invalid sentinels and sample age checked explicitly.

Current evidence: Stock query sample age remains unknown; custom sequence/age parsing and raw-close veto integrated.

### F16 · Firmware

Priority: Must.

Generate a reviewed ESP32 extension exposing sequence, validity and age; compile/flash/test after board identification.

Current evidence: Extension generated; board compilation/flashing unverified.

### F17 · External ToF

Priority: Could.

Optional VL53L1X backend with verified electrical interface and no I2C address conflict.

Current evidence: Optional adapter prepared and disabled.

### F18 · Dataset

Priority: Must.

Group split by session/room/print family, explicit negatives, manually reviewed YOLO labels and held-out TT data.

Current evidence: Team grouped-data tools retained; collection, label validation and held-out detector evaluation added.

## Operating requirements

| Area | Target / constraint |
| --- | --- |
| Control cadence | 10 Hz initial wall-track design; measure the delivered pipeline. |
| Latency | Report median and p95 capture-to-host, inference and command timing separately. Host timestamps do not establish camera exposure time. |
| Action limits | Initial forward cap 20 and yaw cap 25 RC units; calibrate their physical effects. |
| Freshness | Initial image-age limit 0.35 s and range-age limit 0.40 s. A cached sensor reply must not acquire a new sample timestamp. |
| Environment | Short supervised indoor trials on a lit, textured floor; confirm battery budget and installed hardware mass. |
| Reproducibility | Record seed, software versions, model/configuration, feature schema, calibration and logs for each run. |
| Licensing | Preserve third-party software, model and data terms separately from this repository's MIT license. |


## Observation and action contract

| Features | Meaning |
| --- | --- |
| 0–3 | Horizontal/vertical center error, scaled log visual-distance error and calibrated visual range. |
| 4–8 | Forward range, forward-valid flag, association flag, detector confidence and detection-valid flag. |
| 9–12 | Frame age, range age, center-error rate and log-size rate. |
| 13–15 | Previous supervised forward/yaw actions and a latched near-range requirement. |
| History / outputs | Sixteen ordered float32 values stacked across four ticks: 64 inputs. Two normalized outputs: forward and yaw. |


## Forward sensing and firmware

- Measure the factory forward ToF first. EXT tof? returns millimeters; invalid sentinels are rejected. Stock replies have unknown sample age and cannot authorize near approach or arrival. A recent close stock reading still vetoes motion. Downward telemetry is never forward clearance.

- The prepared EXT oprange? extension is application-defined, not an official DJI command. It is intended to expose a sample sequence, validity and age; compilation, query routing and returned freshness must be verified on the delivered ESP32/firmware.

- Forward range near the wall is required for the primary arrival rule. Close raw proximity overrides target association. Once near-range dependence is latched, invalid or stale range blocks further approach.

- An external VL53L1X remains optional and disabled until its voltage, I2C ownership/address, mounting and optical axis are verified. IR ranging is not IR imaging; adding a forward sensor does not qualify low-light flight or fix downward position hold.

## Data and calibration

- Record real printed targets in the intended room: approaches, yaw, pauses, occlusions, partial crops, distractors and empty walls. Use marked distances and a repeatable camera height; a moving clip cannot use one distance label for every frame.

- Start with pretrained YOLO11n. Fine-tune only when held-out tests show a useful gap. Review detection boxes and explicit negative labels; split by recording session, room and print family rather than adjacent frames.

- Fit visual scale using a separate calibration set for each print size and camera. After delivery, collect TT footage and measure camera geometry, exposure/transport delay, range validity and control response.

- Videos support perception and noise estimation. PPO requires an interactive simulator whose next state responds to its actions; a prerecorded video cannot establish closed-loop control performance.

## Prototype evidence and boundaries

| Track | Evidence / limitation |
| --- | --- |
| Original team diagnostics | 15 reported automated checks; 30 scenarios, seeds 10000–10029. Initial PID 30/30, PPO at 100,352 transitions 14/30, PPO at 8,192 transitions 0/30. Published results and trace are preserved verbatim. |
| Integrated implementation | Integrated on top of the teammate source publication at 50d18b8. The offline demo, template, tests and CI are retained. Shared 64-value camera pipeline, marker bench inputs, grounded video/range capture, measured arrival and dispatcher fault checks added; 33 local tests pass, one optional SB3 check skipped. See the dated integration validation record. |
| Control-v2 audit | Official SDK and six public project/library references reviewed. Shared gate, target confirmation, RC quantization, configurable entrypoints and checkpoint checks implemented. New diagnostics are stored separately under docs/evidence/control-v2; older evidence remains unchanged. Windows and Linux CI each passed 46 tests without skips, including a nondefault-config PPO workflow and separate 1024-transition smoke run. |
| Controller tuning | 324 PID gain candidates ranked on 50 tuning scenarios, shortlist selected on 100 validation scenarios: tuned 92/100, starting 87/100, zero minimum-distance violations. Three PPO seeds at 100k and 1M transitions: 34/15/80 and 76/67/85 successes, seed 1 with 77 and 29 violations. Final test seeds 50000–50099 reserved and unrun. |
| Comparison boundary | The earlier local 11-input marker controller is superseded, not deployed. Its 50-scenario score and 52-test suite are not evidence for this integrated implementation. |
| Hardware evidence | No TT flight, real printed-target accuracy, sensor cadence, firmware compilation/flashing or qualified braking margin is established. The dispatcher is tested with fake I/O only. |


## Evaluation and acceptance

- Retain PID as the demonstration baseline unless a qualified alternative is more reliable. Compare tuned PID, nominal PPO and randomized PPO with identical detector settings, action caps and supervisor.

- Train at least three PPO seeds. Select checkpoints and gains using validation seeds 30000–30099 only, then freeze them. Inspected diagnostic seeds and tuning seeds 40000–40099 are development data; the reserved seeds 50000–50099 are the final comparison and are run once, at feature freeze.

- Start with 100 held-out simulated episodes per training seed. Aim for at least ten paired hardware trials per controller after hardware gates pass; report actual sample size, failures and interventions.

- Report success with Wilson intervals, measured wall-distance and image-center error, arrival time with failure outcomes, command changes, overshoot and minimum clearance. Do not claim a speed winner by dropping PPO failures.

- Use identical resolved JSON Config and control-contract version across training and evaluation. A measured arrival that fails the simulator truth audit is arrival_unverified, not success. Preserve older behavior results separately; do not relabel or pool them.

| Gate | Completion evidence required |
| --- | --- |
| G1 · Perception bench | Benchmark real wall prints, distractors and negatives; verify calibration and record timing distributions. |
| G2 · Platform and sensing | Identify firmware/board, verify query routing, sample freshness and camera/ToF geometry against measured distances. |
| G3 · Flight dispatch | Qualify a single dispatcher, operator takeover, independent command-age watchdog, battery/telemetry handling and onboard link-loss behavior. |
| G4 · Conservative PID flight | Measure command response, stopping drift and sensor-to-propeller offset; demonstrate repeated supervised approaches within the qualified envelope. |
| G5 · Policy comparison | Update simulation from measurements; validate multiple seeds and held-out conditions before any policy flight. |
| G6 · Demonstration freeze | Select the most reliable validated configuration and preserve its model/configuration, evidence and known limitations. |


## Course milestones

| Date (2026) | Deliverable |
| --- | --- |
| Oct 29 | Rapid prototypes: separate synthetic-control and camera-only evidence. |
| Oct 30–Nov 3 | Printed-target collection, detector benchmark and November 3 status presentation. |
| Nov 4–7 | Approximate delivery, platform bring-up, camera/range tests and measured response. Arrival remains subject to confirmation. |
| Nov 8–19 | Supervised PID trials and repeatability; Peer Review 1 due November 10 at noon Pacific. |
| Nov 20–Dec 3 | Calibrated controller comparison; feature freeze December 3. |
| Dec 4–15 | Rehearse December 4–9; demonstration December 10; final report and Peer Review 2 December 15 at noon Pacific. |


## Risks and open work

- Printed-image domain shift, visual-scale bias, Wi-Fi delay and missed detections can produce incorrect approach commands.

- Range may be cached, stale, out of range or associated with the wrong surface; sensor sample age and mounting geometry require independent checks.

- Simulation-to-flight mismatch, battery degradation and poor floor texture/lighting can invalidate software-only results.

- A broken radio link can prevent a laptop-issued land command from arriving. Flight safety depends on qualified onboard behavior and an operator procedure.

- If delivery slips, reduce optional features instead of skipping calibration or the flight gates.

## Evidence links

- [Wall-track diagnostics](https://jonatl5.github.io/posepilot/#results)
- [Aligned rapid prototypes](https://jonatl5.github.io/posepilot/prototypes.html)
- [Website PRD](https://jonatl5.github.io/posepilot/prd.html)

The active implementation follows one 64-value wall-approach contract. Historical diagnostics remain attributed to their original run; no real-flight outcome is claimed.
