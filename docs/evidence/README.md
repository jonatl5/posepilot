# objectPilot diagnostic evidence — October 29, 2026

These files describe a Python simulation, not drone flights or detector accuracy.

- `results.json` contains the existing evaluation summaries.
- `*-episodes.csv` contain all 30 episode records for each controller, using seeds 10000–10029.
- `trajectory.json` contains newly recorded deterministic rollouts of the saved initial PPO checkpoint and PID on seed 10000, the first evaluation seed. This scenario was not selected for success. PID succeeds; PPO times out.

PID used its initial gains. PPO used training seed 0, with 8,192 and 100,352 actual transitions respectively. Both controllers passed through the same supervisor. The initial integral gains are zero, so the PID baseline is initially a PD controller.

The model is a fixed-altitude, planar wall-approach simulator with synthetic detections and range measurements, randomized command lag, noise and dropout. The observation has 16 features stacked across four ticks; the actions are normalized forward and yaw commands. Desired stand-off is 0.65 m and the provisional minimum is 0.40 m.

In the trace, `x` is the signed wall-normal position (wall at zero), `y` is the lateral position, `heading` is in radians and `t` is seconds. Wall clearance is `-x`; `range_m` is the simulated sensor reading and can be null. The forward and yaw values are the **supervised** normalized actions. The web player does not execute a policy. The separate supervisor explorer on the homepage is only an interactive illustration.

The 95% success intervals are Wilson intervals. No successful-episode speed ranking is claimed: PPO's failures would bias such a comparison. These inspected seeds are now development data. Final comparisons need tuned baselines, multiple training seeds, held-out evaluation scenarios, measured TT dynamics and hardware trials.

Source code, checkpoints, configuration and the original experiment outputs are in the [October 29 starter package](https://drive.google.com/file/d/1CWB-8wcXvh0cUm5BgMetaEBFtSmdq6WW/view), which may require team access. The local export utility is `scripts/export_site_evidence.py` in the working project; it copies the episode CSVs and replays the existing checkpoint without retraining.
