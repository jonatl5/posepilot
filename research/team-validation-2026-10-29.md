# Local validation record

Archive note: the dependency snapshot originally called `requirements-lock.txt` is now preserved in [team-windows-requirements.txt](team-windows-requirements.txt). The results below predate control-v2; use the active package extras for new installations.

Date: October 29, 2026. Host: Windows, Python 3.12.14, CPU. Dependency versions are in `requirements-lock.txt`. This is pre-hardware evidence only.

| Check | Observed outcome |
|---|---|
| Automated tests | 15 tests pass: SDK units/sentinels, sensor-source separation, stale/future samples, proximity, near-range latch, nonfinite actions, target confirmation, custom firmware sequence/age, Gymnasium contract/seeding, termination/truncation, nominal PID completion and dataset review/leakage checks |
| Detector | YOLO11n weights downloaded; actual 720p blank-frame inference executed |
| Video end to end | Ten control samples from a generated blank video passed through YOLO, feature builder, PID and supervisor; all RC values were zero; JSONL parsed successfully |
| PPO smoke | 8,192 transitions completed, checkpoint and metadata saved; reload/evaluation completed |
| Initial PPO | Requested 100,000 transitions; SB3 rollout rounding produces 100,352 actual transitions; one seed only |
| Firmware | Official RMTT source cloned at b8ece60ec31eeca3350209a2559f549864e91dce; extension generator executed successfully |
| PRD | Original Word structure retained and updated; 10 rendered pages visually reviewed using Microsoft Word PDF export and PDFium after bundled renderer reported missing LibreOffice |

## Initial controller experiment

Thirty randomized synthetic-sensor episodes with shared seeds 10000–10029, default configuration and the same supervisor. These seeds are now development diagnostics, not a reusable final unseen test set. PID gains were initial starting gains, not a completed tuning exercise. The PPO model is one short training run, not a selected converged policy.

| Controller | Success | 95% Wilson interval | Minimum-distance events | Mean successful arrival time |
|---|---:|---|---:|---:|
| PID starting gains | 30/30 | 88.6%–100% | 0 | 16.79 s |
| PPO after 8,192 transitions | 0/30 | 0%–11.4% | 0 | Not applicable |
| PPO after 100,352 transitions | 14/30 | 30.2%–63.9% | 0 | 14.41 s |

Success-conditioned timing is not a fair speed win when failure rates differ. These results support keeping PID as the baseline and continuing PPO training/evaluation. They do not establish PID superiority after full tuning, real-world safety, detector quality, or flight performance.

## Still unvalidated

- Real printed-target videos and labels; webcam capture on the user's camera; print-specific calibration and held-out detection metrics.
- TT SDK, ToF optical geometry, units on the delivered firmware, fresh sample rate, radio latency, RC response, braking and battery effects.
- ESP32 compilation/flashing, actual custom EXT query routing and external VL53L1X electrical integration. No TT Arduino toolchain is installed here.
- Live flight dispatcher/watchdog, manual takeover, takeoff/landing and link-loss behavior; no autonomous flight launcher is provided.
- Sensor realism beyond the simple randomization: independent asynchronous range delivery, burst failures, misleading boxes, camera-exposure timing and 3D effects.
- PID tuning, multiple PPO training seeds, validation-based checkpoint selection and new untouched final test scenarios.

The package includes raw episode CSVs and summaries for the diagnostic runs. Preserve them as development evidence. After changing environment parameters or choosing a model, generate a separate frozen evaluation set and report all seeds and failure outcomes.
