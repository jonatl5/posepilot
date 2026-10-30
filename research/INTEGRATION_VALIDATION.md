# Integration verification — October 29, 2026

Historical record for `f88abed`, before the control-v2 audit. See [current audit](TT_CONTROL_AUDIT.md) and [new evidence](../docs/evidence/control-v2/README.md).

This record covers the integrated repository, separately from the original team Windows run. It does not establish flight readiness.

| Check | Observed result |
|---|---|
| Python regression suite | 33 passed; one optional SB3 environment check skipped |
| Generated ArUco images through actual OpenCV detector and shared pipeline | 45 logged samples, all observations length 64, no commands transmitted |
| Generated red-marker images through actual OpenCV detector and shared pipeline | 45 logged samples, all observations length 64, no commands transmitted |
| File replay | Unit regression verified ten selected ticks from a one-second 30 FPS recording |
| PID randomized simulator, seeds 20000–20029 | 28/30 success, zero minimum-distance violations |
| Retained offline PID demo, seed 10000 | Success at 16.90 simulated seconds; embedded HTML and JSONL verified |
| Mean successful simulated approach time | 15.09 seconds |
| 95% Wilson interval for simulated success rate | 78.68%–98.15% |

The simulation uses the team's existing environment and untuned starting gains. These additional seeds are now development diagnostics. The 28/30 result must not replace or be pooled with the original 30/30 result from seeds 10000–10029. The two unsuccessful episodes timed out. Ground-truth simulator success also checks velocity and wall clearance; measured camera arrival cannot measure those quantities directly.

The integration tests cover invalid configuration, 11-input schema rejection, unknown-age stock ranges, raw close-range veto, two-second arrival hold/reset, range counter rollover, marker normalization and JSON logging, file timestamp validation, flight-command rejection on the ground transport, and dispatcher heartbeat/stale-command/telemetry/failure handling with fake I/O.

The local integration environment used Python 3.12 on macOS ARM64. Exact installed versions, replay counts and the full configuration are in [validation.json](../docs/evidence/integration/validation.json). An optional dependency installation failed during download; SB3 and PyAV were not available for a new run. No new PPO training, YOLO neural inference or H.264 decoding result is claimed here. The original team's separate report remains [team-validation-2026-10-29.md](team-validation-2026-10-29.md).

## Reproduce

```bash
python -m pip install -e '.[test,markers]'
python -m pytest -q -ra
python -m objectpilot.replay --source synthetic --detector aruco --seconds 5 --out runs/aruco-check.jsonl
python -m objectpilot.replay --source synthetic --detector red --seconds 5 --out runs/red-check.jsonl
python -m objectpilot.evaluate --episodes 30 --seed 20000 --out runs/pid-check
```

Live synthetic sample counts depend on scheduling. They measure interface execution, not tracking accuracy or drone dynamics. The recorded [PID rows](../docs/evidence/integration/pid-episodes.csv) and [summary](../docs/evidence/integration/pid-summary.json) are reproducible simulation evidence. Tests use generated fixtures and fake transport only.

## Pending physical evidence

Actual TT video latency, forward-sensor freshness/geometry, firmware compile/flash, installed payload, RC sign and stopping distance, manual takeover, landing confirmation and onboard link-loss behavior all remain unverified. See [HARDWARE_GATES.md](HARDWARE_GATES.md). The public replay/collection commands do not arm or fly the aircraft.
