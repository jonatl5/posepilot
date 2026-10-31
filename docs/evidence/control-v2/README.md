# Control-v2 diagnostic evidence

Recorded October 29, 2026, after the TT contributor/interface audit. Behavior version: `objectpilot-control-v2`; observation layout remains `objectpilot-v1-16x4`.

- `validation.json`: local regression results, environment limitations, required CI checks and provenance.
- `pid-summary.json` / `pid-episodes.csv`: 30 randomized scenarios, seeds 20000–20029; PID 25/30, zero minimum-distance violations. These are previously inspected development scenarios, not a held-out test.
- `demo/`: exact offline PID episode at seed 10000, including HTML playback, summary and JSONL.
- `tuning/`: October 31 PID tuning, tuned-gains validation and control-v2 PPO seed results on the validation scenarios; see its own README.

Run `python -m objectpilot.evaluate --config configs/wall_approach.json --episodes 30 --seed 20000 --out runs/new-v2-evaluation` from a clean environment to reproduce. The new arrival gate, target confirmation and RC quantization change the environment, so earlier success rates and PPO checkpoints are not directly comparable. Original team and integration results remain unchanged in their own directories. Short PPO checks in CI establish interface compatibility only; they do not qualify policy quality or hardware flight.

[Windows and Linux CI](https://github.com/jonatl5/posepilot/actions/runs/40978634314) each passed all 46 tests with no skips, including the nondefault-config PPO workflow. Both also passed the 1024-transition PPO smoke run, dependency checks, English/PRD checks and firmware source generation. The validated code revision is `2b0bfe3`; this follow-up records that evidence and updates documentation only.
