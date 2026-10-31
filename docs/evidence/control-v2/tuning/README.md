# Controller tuning evidence — October 31, 2026

Simulation results under `objectpilot-control-v2` with `configs/wall_approach.json`. These files describe the planar Python simulator with uncalibrated response, not drone flights, sensor tests or detector accuracy. The protocol, interpretation and three open simulator findings are in [research/CONTROLLER_TUNING.md](../../../../research/CONTROLLER_TUNING.md).

## Scenario roles

Development seeds 10000–10029 and 20000–20029 are the inspected diagnostics behind the earlier results. Tuning seeds 40000–40049 ranked the PID candidates. Validation seeds 30000–30099 selected the shortlist and compared every controller below. Final test seeds 50000–50099 are reserved and have not been run; `objectpilot.evaluate` and `objectpilot.demo` refuse them without `--final-test`.

## Files

- `comparison.json`: every controller on the 100 validation scenarios, with Wilson intervals, outcomes, censored and successful mean times, final distance error and lowest clearance.
- `pid-candidates.csv`: all 324 gain candidates ranked on the 50 tuning scenarios.
- `pid-validation.csv`: the top five candidates and the starting gains re-run on the validation scenarios; rank 1 is the selected candidate.
- `pid-selected-gains.json`: the selection record, identical to `configs/pid_gains_tuned.json`.
- `validation-pid-starting-*`, `validation-pid-tuned-*`: `objectpilot.evaluate` output for both gain sets on the validation scenarios. The tuned rows match `pid-validation.csv` episode by episode.
- `development-pid-tuned-*`: the tuned gains on development seeds 20000–20029 (26/30), for comparison with the published 25/30 starting-gain run in the parent folder.
- `validation-ppo-100k-seed*`, `validation-ppo-1m-seed*`: three training seeds at 100,352 and 1,000,448 actual transitions; `ppo-*-metadata.json` holds each checkpoint's contract, configuration, seed and library versions. Checkpoints themselves stay in local `runs/` directories, not in Git.
- `demo-pid-tuned/`: offline seed-10000 episode with the tuned gains (success at 19.6 s); the starting-gain recording is in `../demo/`.

## Results on validation seeds 30000–30099

| Controller | Success | 95% Wilson | Minimum-distance violations | Censored mean time |
|---|---:|---|---:|---:|
| PID starting gains | 87/100 | 79.0–92.2% | 0 | 21.27 s |
| PID tuned gains | 92/100 | 85.0–95.9% | 0 | 16.99 s |
| PPO 100k, seeds 0 / 1 / 2 | 34 / 15 / 80 | 25.5–43.7% · 9.3–23.3% · 71.1–86.7% | 1 / 77 / 8 | 32.12 / 35.76 / 20.78 s |
| PPO 1M, seeds 0 / 1 / 2 | 76 / 67 / 85 | 66.8–83.3% · 57.3–75.4% · 76.7–90.7% | 0 / 29 / 0 | 22.89 / 22.77 / 18.93 s |

Censored mean time counts failed episodes at the 40 s limit. The validation scenarios selected the tuned gains, so their score is optimistic for that candidate; the reserved seeds give the unbiased comparison at feature freeze. Do not pool these numbers with the October 29 development-seed results.

## Environment

Windows 11, Python 3.12.16, numpy 2.5.4, gymnasium 1.4.0, torch 2.15.0 (CPU), stable-baselines3 2.9.2, opencv-python 4.14.0; 56 local tests passed. The `git_base` fields name the parent revision `c31b413`; the tuning tool and these files land in the recording commit. [Windows and Linux CI](https://github.com/jonatl5/posepilot/actions/runs/41240684647) passed for `c218de6`, the reviewed revision of these tools: 56 tests on Linux and 56 on Windows with no skips, the PID tuning smoke run, the committed tuned-gains evaluation and demo, dependency checks, English/PRD synchronization checks and firmware source generation. Re-running the tuned PID, starting PID and PPO 1M seed 2 validations with that revision reproduced the committed episode rows exactly.

## Reproduce

Run the commands in the research note from a clean environment with new output directories. Tuning is deterministic for the same grid and seeds. PPO training on CPU reproduced identical step counts here; library upgrades can change the learned weights, so compare evaluations rather than checkpoints.
