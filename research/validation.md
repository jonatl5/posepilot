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

The repository includes raw diagnostic episode CSVs and summaries in `docs/evidence/`. Checkpoints and the original run directories are in the linked October 29 starter package, not in Git. Preserve them as development evidence. After changing environment parameters or choosing a model, generate a separate frozen evaluation set and report all seeds and failure outcomes.

## Repository sync verification — October 29, 2026

The README commands were exercised again in the existing Windows / Python 3.12 environment, with fresh `runs/github_verify/` output directories to preserve previous runs.

| Check | Observed outcome |
|---|---|
| Editable install | `pip install -e ".[vision,test]"` succeeds; `pip check` reports no broken requirements |
| Tests | 17 pass, including the offline demo CLI, embedded trace consistency and output preservation |
| Offline PID demo | Seed 10000 succeeds in 16.9 simulated seconds, with final clearance 0.65665 m; HTML, JSONL and summary generated |
| Fresh PPO smoke run | 8,192 transitions; model and metadata saved, loaded for evaluation and used to generate a separate HTML demo |
| Matched development evaluation | Seeds 20000–20019: PID 19/20, short PPO 0/20; neither records a minimum-distance event. These are pipeline diagnostics, not a new final model comparison |
| Blank-video replay | 10 YOLO/control samples; all logged RC channels zero; no hardware dispatch |
| Firmware generation | Succeeds from pinned upstream revision without modifying that checkout; board compilation/flash still untested |
| Distributable package | Wheel built; HTML template included; PID demo successfully executed by importing from the wheel |
| Offline HTML checks | Embedded data matches JSONL; JavaScript syntax passes. The in-app browser blocks `file://`, so no visual browser check of this new offline artifact was completed |

The Python CI workflow repeats unit tests, short PPO save/load, offline demos and pinned-source firmware generation on a fresh Linux runner. It does not compile ESP32 firmware, open a camera, download YOLO weights, or connect to a TT.
