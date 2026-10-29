# Integration evidence

These files describe the additional October 29 integration checks. They are separate from the original team PID/PPO evidence in the parent folder.

- `validation.json`: configuration, environment, test results, marker replay counts and explicit untested areas.
- `pid-summary.json`: new randomized simulator diagnostic, seeds 20000–20029.
- `pid-episodes.csv`: individual outcomes for that run.

No file here describes real flight, a real sensor test, or a newly trained neural model. The previous 11-input marker reference is not part of the deployed control contract.

`demo-pid.html`, `summary.json` and `trajectory.jsonl` retain the seed-10000 offline demo and its embedded recording.
