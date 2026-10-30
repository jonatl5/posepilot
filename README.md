# objectPilot

Recognize a stationary printed object on a wall, center it in the RoboMaster TT camera, approach slowly and hold a measured stand-off. A PC runs YOLO, a PID baseline or PPO policy, and independent supervision. The working product name is objectPilot; the repository and Pages URL remain posepilot.

[Website](https://jonatl5.github.io/posepilot/) · [PRD](PRD.md) · [TT control audit](research/TT_CONTROL_AUDIT.md) · [Route comparison](research/ALIGNMENT.md) · [Control mapping](research/ARCHITECTURE.md) · [Data plan](research/DATA_COLLECTION.md) · [Hardware gates](research/HARDWARE_GATES.md)

## One implementation route

The repository now includes the implementation from the team's October 29 starter package and aligned additions. The primary target remains a matte wall print detected by YOLO11n. Red cards and ArUco markers are optional bench inputs into the same pipeline; they do not define another policy, simulator or deployment route.

- State: `objectpilot-v1-16x4`, 16 features across four ticks, 64 float32 inputs.
- Control: 10 Hz, forward and yaw only; lateral and vertical commands stay zero.
- Initial simulation settings: 0.65 m stand-off, 0.40 m minimum, forward cap 20, yaw cap 25.
- Range: fresh associated forward ToF for near approach and measured arrival; raw close readings veto motion.
- Arrival: centered within 0.05 frame widths, range within 0.08 m, continuous two-second measured hold.
- First demo: stationary targets, normal indoor lighting, a textured floor and fixed altitude. Person following and night flight are outside this milestone.

These are unqualified simulation parameters, not approved physical clearances. Read the [exact feature/action contract](research/interfaces.md) and [hardware gates](research/HARDWARE_GATES.md) before any flight integration.

## Run before the drone arrives

Use Python 3.10–3.13; Python 3.12 is recommended. Create an environment from this repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test,markers]'
python -m pytest -q
python -m objectpilot.demo --seed 10000 --out runs/demo-pid
python -m objectpilot.replay --source synthetic --detector aruco --seconds 5 --out runs/aruco-bench.jsonl
python -m objectpilot.replay --source synthetic --detector red --seconds 5 --out runs/red-bench.jsonl
python -m objectpilot.evaluate --episodes 30 --seed 20000 --out runs/pid-integration
```

Open `runs/demo-pid/demo.html` to play, pause and scrub the team's offline simulator trace. It embeds the generated recording and works without a server; it does not run YOLO or a policy in the browser. `summary.json` and `trajectory.jsonl` retain the exact episode.

On Windows PowerShell, create the environment with `py -3.12 -m venv .venv`, then set `$opPython = (Resolve-Path .\.venv\Scripts\python.exe).Path` and replace each `python` invocation with `& $opPython`. This does not require changing execution policy. The historical Windows freeze is archived in `research/team-windows-requirements.txt`; it is not an installation file. Use the same `pyproject.toml` extras on every platform. Only install one OpenCV distribution: `opencv-python>=4.10,<5`, which includes the ArUco APIs used here. Start a clean environment if an older setup contains both standard and contrib wheels. Add `--show` for the camera/video window. Synthetic image input exercises OpenCV and control interfaces; it is not a drone dynamics simulation. `objectpilot.evaluate` runs the action-responsive wall simulator. Output logs and evaluation folders must be new so previous evidence is preserved. The optional SB3 environment check is skipped unless the RL extra is installed.

## YOLO and data

```bash
python -m pip install -e '.[vision]'
python -m objectpilot.replay --source 0 --class-name bottle --weights yolo11n.pt --out runs/webcam.jsonl --show
python -m objectpilot.replay --source data/raw/approach.mp4 --class-name bottle --out runs/video.jsonl
```

The first YOLO run may download pretrained weights; do this before joining aircraft Wi-Fi. Start with pretrained detection, collect a held-out wall-print benchmark and fine-tune only if the measured gap warrants it. The [English data guide](research/DATA_COLLECTION.md) covers grouped recording, explicit negatives, annotations, visual-scale calibration and detector training/evaluation. No real target dataset or new detector weights are claimed.

## Grounded TT video and range

```bash
python -m pip install -e '.[markers,video,hardware]'
python -m objectpilot.replay --source tello --detector aruco --range-mode stock --out runs/tt-ground.jsonl --show
python -m objectpilot.bench --seconds 15 --out runs/tof-bench.jsonl
```

Replay and collection never send takeoff, landing or RC commands. `GroundLink` has a strict ground-command allowlist. Stock `EXT tof?` receipts do not establish sample freshness and cannot authorize near approach or arrival. A recently received close stock value can still stop the proposed motion. Use `--range-mode custom` only after the prepared firmware's sequence/validity/age reply is qualified on the delivered hardware.

`dispatch.py` supplies a separately tested command-age/operator-heartbeat watchdog scaffold with an injected sink. It is not connected to a live flight CLI, has no takeoff command and is not hardware qualified. A future adapter must provide bounded I/O, manual takeover, confirmed landing and measured onboard link-loss behavior. Do not replace those checks with a boolean flight flag.

## PPO research

```bash
python -m pip install -e '.[rl]'
python -m objectpilot.train --config configs/wall_approach.json --steps 100000 --seed 0 --out runs/ppo-seed0
python -m objectpilot.evaluate --config configs/wall_approach.json --model runs/ppo-seed0/policy.zip --episodes 100 --seed 30000 --out runs/ppo-validation
python -m objectpilot.demo --config configs/wall_approach.json --model runs/ppo-seed0/policy.zip --out runs/demo-ppo
```

Train multiple seeds and compare tuned PID, nominal PPO and randomized PPO under matched settings. Use validation to select models and reserve new final test scenarios. Train, evaluate, demo and replay all accept `--config`. Use the same JSON file in each command. Policies must match the complete Config, ordered features, action names, actual tensor spaces and `objectpilot-control-v2` behavior version. Older checkpoints require retraining, including previous 64-input checkpoints without this behavior version. Video is not an interactive PPO environment. The default baseline remains PID unless a qualified alternative is more reliable.

## Evidence and provenance

The original team run reported 15 tests and a ten-sample blank-video check. Its 30 diagnostic scenarios produced PID 30/30, initial PPO 14/30 after 100,352 transitions, and smoke PPO 0/30 after 8,192 transitions. Existing [CSV/trajectory evidence](docs/evidence/README.md) is preserved verbatim and is not relabeled as this integration's output. These inspected seeds are development data, not a final unseen test set.

The current [TT audit](research/TT_CONTROL_AUDIT.md) compares both contributors and public projects. New [control-v2 evidence](docs/evidence/control-v2/README.md) is separate from earlier results. Simulation and replay now use the same target confirmation, arrival/stop gate and RC quantization; physical truth audits simulation success without changing the command gate.

Historical integration evidence is recorded in [research/INTEGRATION_VALIDATION.md](research/INTEGRATION_VALIDATION.md) and [machine-readable validation](docs/evidence/integration/validation.json). The team's later repository/demo verification remains in [research/validation.md](research/validation.md). The earlier standalone marker scores are not pooled with the team's results. No actual drone flight, sensor qualification or firmware compilation/flash is claimed.

The imported source is identified in [source-provenance.json](research/source-provenance.json). Original team authorship/license remains in place. The [alignment review](research/ALIGNMENT.md) explains reused components, additions and retired conflicting settings.

## Files

| Path | Role |
|---|---|
| `src/objectpilot/core.py` | Timestamped measurements, 64-value features, PID, supervisor, RC mapping |
| `src/objectpilot/pipeline.py` | Shared dry-run controller and measured arrival hold |
| `src/objectpilot/vision.py` | YOLO and marker adapters; target confirmation |
| `src/objectpilot/replay.py`, `video.py`, `ground_link.py` | File, webcam, synthetic and ground-only Tello input |
| `src/objectpilot/dispatch.py` | Independent watchdog scaffold; fake-I/O tests |
| `src/objectpilot/env.py`, `train.py`, `evaluate.py`, `demo.py` | Team simulator, offline HTML demo and PPO/PID research workflow |
| `src/objectpilot/data.py`, `collect.py`, `calibrate_camera.py` | Collection, labels, splitting, training and calibration |
| `firmware/` | Reviewed extension generator and optional external ToF starting point |
| `configs/wall_approach.json` | Canonical initial Config, not a flight qualification |
| `research/`, `tests/` | English plans, provenance, evidence and regression checks |
| `docs/` | GitHub Pages source, English PRD and published evidence |

## Keep website and PRD synchronized

Node.js 18+ is sufficient; no website dependencies are required.

```bash
npm run build:docs
python scripts/package_source.py
npm run check
python scripts/check_repository.py
npm start
```

Edit `docs/data/prd.json` and regenerate `PRD.md`, `docs/PRD.md` and `docs/prd.html` together. The language check covers published text and code comments. The source archive is built from an explicit allowlist and excludes environments, credentials, local logs, private data and model weights. Open `http://127.0.0.1:4317/posepilot/` to preview. GitHub Pages publishes `/docs` from `main`; an unmerged/local branch does not update the public site.

## License

Original project source and documentation retain the repository [MIT license](LICENSE). Third-party firmware, software, data and model licenses remain separate. The firmware generator reads a pinned upstream checkout and does not redistribute its generated sketch in the source download.
