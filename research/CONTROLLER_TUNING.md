# Controller tuning and control-v2 PPO seeds — October 31, 2026

Simulation-only follow-up to the [TT control audit](TT_CONTROL_AUDIT.md). Behavior version `objectpilot-control-v2`, observation layout `objectpilot-v1-16x4`, configuration `configs/wall_approach.json`. Host: Windows 11, Python 3.12.16, numpy 2.5.4, gymnasium 1.4.0, torch 2.15.0 (CPU), stable-baselines3 2.9.2, opencv-python 4.14.0; 56 local tests passed (46 existing, 10 new). The code revision is the commit that records this file; the `git_base` fields in the evidence name its parent, `c31b413`. No flight, sensor, firmware or detector result is claimed, and the reserved final test seeds were not run.

## Scenario roles

Scenario seeds are the simulator's only randomness, so a seed range is a fixed set of scenarios. `objectpilot.evaluate` records the role of every requested range in `summary.json`.

| Role | Seeds | Use | Status |
|---|---|---|---|
| Development | 10000–10029, 20000–20029 | Inspected diagnostics behind the published October 29 results | Do not use for selection |
| Tuning | 40000–40099 (this run used 40000–40049) | Rank PID gain candidates | Development data |
| Validation | 30000–30099 | Re-run the shortlist, compare controllers and PPO training seeds, select | Development data; scores of the selected candidate are optimistic |
| Final test | 50000–50099 | One frozen comparison at feature freeze (December 3) | Reserved. `evaluate` and `demo` refuse the range without `--final-test`; `tune_pid` always refuses it |

## PID tuning

`python -m objectpilot.tune_pid --config configs/wall_approach.json --out runs/pid-tuning` ran the grid kp_forward {1.0, 1.5, 2.0, 2.5, 3.0, 4.0} × ki_forward {0, 0.1, 0.2} × kd_forward {0, 0.08, 0.2} × kp_yaw {2.0, 3.5, 5.0} × ki_yaw {0} × kd_yaw {0.05, 0.15}: 324 candidates including the starting gains, 50 tuning episodes each, 16,200 episodes in 59 s on 16 worker processes. Ranking: highest success rate, then fewest minimum-distance violations, shortest mean episode time with failures counted at the 40 s limit, smoothest commands; exact ties keep the starting gains. The top five and the starting gains were then run on the 100 validation scenarios and ranked by the same rule.

| Tuning result (50 episodes, seeds 40000–40049) | Value |
|---|---|
| Starting gains kp [1.5, 3.5], kd [0.08, 0.05] | 39/50, rank 250 of 324 |
| Twelve candidates tied at the top | 43/50, all with kp_forward 3.0 or 4.0 |
| Mean successes by kp_forward 1.0 / 1.5 / 2.0 / 2.5 / 3.0 / 4.0 | 36.9 / 39.4 / 40.8 / 41.6 / 42.0 / 41.6 |
| Minimum-distance violations, any candidate | 0 |

| Validation (100 episodes, seeds 30000–30099) | Success | 95% Wilson | Censored mean time | Successful mean time | Mean final distance error | Mean squared action change | Lowest clearance |
|---|---:|---|---:|---:|---:|---:|---:|
| c283 · kp [4.0, 2.0], ki [0, 0], kd [0.2, 0.15] — selected | 92/100 | 85.0–95.9% | 16.99 s | 14.98 s | 0.034 m | 0.0306 | 0.579 m |
| c285 · kp [4.0, 3.5], kd [0.2, 0.15] | 92/100 | 85.0–95.9% | 17.05 s | 15.05 s | 0.034 m | 0.0319 | 0.577 m |
| c284 · kp [4.0, 3.5], kd [0.2, 0.05] | 92/100 | 85.0–95.9% | 17.05 s | 15.05 s | 0.034 m | 0.0316 | 0.581 m |
| c255 · kp [3.0, 3.5], ki [0.2, 0], kd [0, 0.15] | 91/100 | 83.8–95.2% | 17.53 s | 15.30 s | 0.035 m | 0.0301 | 0.575 m |
| c254 · kp [3.0, 3.5], ki [0.2, 0], kd [0, 0.05] | 91/100 | 83.8–95.2% | 17.53 s | 15.31 s | 0.036 m | 0.0298 | 0.575 m |
| Starting gains | 87/100 | 79.0–92.2% | 21.27 s | 18.47 s | 0.065 m | 0.0203 | 0.664 m |

The selected candidate is [configs/pid_gains_tuned.json](../configs/pid_gains_tuned.json), with the full selection record inside. The gain surface is a plateau: kp_forward 3.0–4.0 candidates differ by one validation episode and by hundredths of a second, which is noise at this sample size. The 87 → 92 improvement has overlapping Wilson intervals, so it is suggestive rather than established; the timing, final distance error and smoothness differences are consistent across both seed sets. Higher gains approach faster and settle closer to the stand-off, overshoot it by up to about 7 cm (lowest clearance 0.58 m against the 0.40 m minimum) and change commands more. On the development seeds 20000–20029 the tuned gains complete 26/30 against the published 25/30: seeds 20004 and 20015 now succeed, 20001, 20002 and 20021 still time out, and seed 20016 ends as `arrival_unverified` at 7.7 s because the overshoot left the true clearance at 0.567 m, 0.083 m from the stand-off, while the noisy range satisfied the 0.08 m window. Both seed-10000 demos succeed at 19.6 s; the tuned run settles at 0.680 m against 0.708 m for the starting gains. Both recordings are in `docs/evidence/control-v2/`; the 16.9 s in the October 29 records belongs to the pre-control-v2 demo and is not comparable.

These gains multiply RC-unit errors in a simulator whose nominal response is a guess (RC 20 forward and RC 25 yaw). Treat the file as the simulation candidate for the first supervised PID trials, then re-run the same tuning on a simulator updated with measured response, braking and sensor timing.

## PPO seeds under control-v2

`python -m objectpilot.train --config configs/wall_approach.json --seed {0,1,2}` with four randomized environments and the unchanged PPO settings (MLP 64×64, 256 steps per environment per rollout, batch 64, learning rate 3e-4, entropy 0.01). Two budgets were trained for each seed: 100,000 requested transitions (100,352 actual, about 74 s per seed on one CPU thread) and 1,000,000 requested transitions (1,000,448 actual, about 11.4 minutes per seed). Every checkpoint passed the contract check and was evaluated on the 100 validation scenarios. Checkpoints stay in `runs/` and out of Git, as before; each `metadata.json` is in the evidence folder.

| Controller on validation seeds 30000–30099 | Success | 95% Wilson | Minimum-distance violations | Other outcomes | Censored mean time | Mean final distance error |
|---|---:|---|---:|---|---:|---:|
| PID starting gains | 87/100 | 79.0–92.2% | 0 | 13 timeouts | 21.27 s | 0.065 m |
| PID tuned gains | 92/100 | 85.0–95.9% | 0 | 8 timeouts | 16.99 s | 0.034 m |
| PPO 100k, seed 0 | 34/100 | 25.5–43.7% | 1 | 59 timeouts, 6 arrival_unverified | 32.12 s | 0.116 m |
| PPO 100k, seed 1 | 15/100 | 9.3–23.3% | 77 | 4 timeouts, 3 lost, 1 arrival_unverified | 35.76 s | 0.211 m |
| PPO 100k, seed 2 | 80/100 | 71.1–86.7% | 8 | 10 timeouts, 2 arrival_unverified | 20.78 s | 0.062 m |
| PPO 1M, seed 0 | 76/100 | 66.8–83.3% | 0 | 24 timeouts | 22.89 s | 0.054 m |
| PPO 1M, seed 1 | 67/100 | 57.3–75.4% | 29 | 3 timeouts, 1 arrival_unverified | 22.77 s | 0.103 m |
| PPO 1M, seed 2 | 85/100 | 76.7–90.7% | 0 | 14 timeouts, 1 arrival_unverified | 18.93 s | 0.048 m |

Ten times more training raised every seed (34 → 76, 15 → 67, 80 → 85 successes) and removed the violations of seeds 0 and 2, yet seed 1 still ended 29 episodes below the minimum clearance and the best seed stays below both PID results. PID remains the demonstration baseline. Training-seed variance at 100k transitions is large (15–80%), so one seed never describes PPO; the original 14/30 result came from one seed under the older behavior and is not comparable.

## Three simulator findings that gains cannot fix

These were found while attributing the remaining timeouts and the PPO violations. Each needs a team decision because it changes either the canonical configuration or the versioned control behavior, which would retire control-v2 checkpoints.

1. **Detection dropout resets the two-second hold.** Four of the five starting-gain timeouts on seeds 20000–20029 (20001, 20002, 20004, 20015) had per-frame dropout 0.086–0.112, reached 18–19 of the 20 required hold ticks and were reset 10–23 times by a missing detection and 4–10 times by a missing range sample. One empty frame drops `TargetLock`, which then needs three new frames; the supervisor reports `stale_or_missing_detection` and `ControlGate` clears the hold. At 12% dropout the chance that 20 consecutive ticks keep both detection and range is about 2%; at 3% it is about 40%. The command is already zero during the hold, so the open question is whether a hold may bridge gaps shorter than `frame_stale_s` (0.35 s) and whether the lock should coast for one or two frames before declaring loss. Either change is control-v3.
2. **Near-entry latch versus ToF range.** Seed 20021 (visual-scale bias +8%) latched `near_sensor_required` at 3.6 s, when the visual distance read 1.13 m while the ray to the wall was still 1.25 m, 5 cm beyond the 1.20 m sensor maximum. The supervisor answered `near_range_unavailable` for 291 of 400 ticks and the drone coasted to a stop at 1.20 m, where the ray measured 1.21 m and the range stayed invalid. The 5 cm between `near_entry_m` (1.15) and `forward_range_max_m` (1.20) is smaller than a 10% visual-scale error at that distance (about 12 cm). Options: lower `near_entry_m` in the canonical configuration (it is part of the policy contract, so PPO checkpoints retrain) or qualify a larger usable range on the delivered sensor. Gains do not change this outcome.
3. **Supervisor creep past the minimum clearance.** PPO seed 1 at 100k transitions ended 77 of 100 validation episodes below 0.40 m. In the traced episode (seed 30001) the policy proposed full forward on every tick below the stand-off; the braking rule `margin / 0.35` passed RC 1–3, the drone crept at 0.01–0.04 m/s to 0.40 m, and ±1 cm range noise alternated `front_clearance_stop` with `tracking` until the true clearance crossed the limit. Two further contributors apply: the simulated ToF measures the oblique ray `-x / cos(heading)` while the collision test uses the perpendicular clearance, so the supervisor over-reads clearance by up to about 6 cm at 0.5 rad of heading, and first-order lag plus command lag carries the aircraft on after the last command; in 30 of the 77 episodes the supervisor never saw a range at or below 0.40 m before the colliding tick. The supervisor slowed every approach before the limit (speed entering the braking zone 0.09–0.40 m/s, mean 0.25 m/s) but it protects only the minimum, not the stand-off, and has no hysteresis. A latched stop with a release margin addresses the noise alternation but would not have fired in those 30 episodes unless its trigger also covers the heading over-read and the stopping distance; a forward cap below the stand-off covers all 77. Both are control-v3 changes. Until then, a learned policy can use the supervisor as its brake, and simulated zero violations for PID say nothing about a different controller.

## Reproduce

```bash
python -m objectpilot.tune_pid --config configs/wall_approach.json --out runs/pid-tuning
python -m objectpilot.evaluate --config configs/wall_approach.json --episodes 100 --seed 30000 --out runs/val-pid-starting
python -m objectpilot.evaluate --config configs/wall_approach.json --gains configs/pid_gains_tuned.json --episodes 100 --seed 30000 --out runs/val-pid-tuned
python -m objectpilot.evaluate --config configs/wall_approach.json --gains configs/pid_gains_tuned.json --episodes 30 --seed 20000 --out runs/dev-pid-tuned
python -m objectpilot.demo --config configs/wall_approach.json --gains configs/pid_gains_tuned.json --seed 10000 --out runs/demo-pid-tuned
for seed in 0 1 2; do
  python -m objectpilot.train --config configs/wall_approach.json --steps 100000 --seed $seed --out runs/ppo-v2-seed$seed
  python -m objectpilot.train --config configs/wall_approach.json --steps 1000000 --seed $seed --out runs/ppo-v2-1m-seed$seed
  python -m objectpilot.evaluate --config configs/wall_approach.json --model runs/ppo-v2-seed$seed/policy.zip --episodes 100 --seed 30000 --out runs/val-ppo-100k-seed$seed
  python -m objectpilot.evaluate --config configs/wall_approach.json --model runs/ppo-v2-1m-seed$seed/policy.zip --episodes 100 --seed 30000 --out runs/val-ppo-1m-seed$seed
done
```

Use new output directories; the commands refuse to overwrite. Tuning is deterministic for a fixed grid and seed range; PPO training is deterministic on CPU for a fixed seed in this environment but may differ across library versions. Evidence files and summaries are in [docs/evidence/control-v2/tuning](../docs/evidence/control-v2/tuning/README.md).

## Limits

- The simulator's response, lag, noise and dropout ranges are assumptions; none of these numbers is a flight result or a detector result.
- The validation seeds selected the gains and compared the PPO seeds, so the selected candidate's validation score is optimistic. The reserved seeds 50000–50099 have not been run and should be run once, at feature freeze, with the frozen gains and checkpoints.
- No nominal-environment PPO training and no checkpoint selection across training were done; each seed's final checkpoint was evaluated.
- Hardware response calibration, firmware freshness, camera delay and the flight gates in [HARDWARE_GATES.md](HARDWARE_GATES.md) are unchanged.
