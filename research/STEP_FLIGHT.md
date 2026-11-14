# Step calibration flight

Status, November 13, 2026: flown once on the delivered TT (cup print, YOLO11s); the flight completed and its numbers are below.

`objectpilot.step_flight` is the only module in the repository that sends takeoff, movement and landing commands. It measures what the stop-and-go planner ([STEPWISE.md](STEPWISE.md)) and its test world assume, so the test world can be re-run with flight numbers before the first full approach.

## What one flight measures

| Measurement | From | Replaces in `objectpilot.stepsim` |
|---|---|---|
| Move scale for forward/back | ToF before and after `forward 20` / `back 20` | `move_scale_sd` |
| Move scale for left/right | Box offset × 2·tan(HFOV/2) × ToF, before and after `left/right 20` and `40` | `move_scale_sd` |
| Cross drift | ToF change during strafes; lateral change during forward/back | `move_cross_sd` |
| Rotation accuracy | State-stream yaw before and after `cw/ccw 10`, `20`, `40` | `yaw_sd_deg`, `yaw_scale_sd` |
| Heading change while translating | State-stream yaw during moves | `move_heading_sd_deg` |
| `error` replies | Replies to all step commands | `fail_prob` |
| ToF against heading | 16-step yaw sweep at 0.8 m, ±40° | `tof_oblique_excess`; checks `fit_wall_normal` in flight |
| Image response signs | Box shift per command (left → box right, cw → box left, up → box down, forward → box larger) | Checks the planner's sign conventions |
| Video HFOV | Box shift per measured rotation | Checks the 54° assumption |
| Command duration, residual speed | Reply time; `vgx/vgy/vgz` after `ok` | Observation timing |

Each command is followed by a 0.5 s pause and an observation: 6 video frames through YOLO, 6 `EXT tof?` replies and the latest state-stream packet. Each observed frame is saved as a JPEG, so a later detector can be re-run on the same flight.

## Sequence (about 3.5 minutes, about 75 step commands)

1. Pre-flight on the ground: `command`, `sdk?`, `battery?` (at least 50%), `speed 30`, video, state stream and ToF replies. The operator confirms the checklist by typing `fly`.
2. `takeoff`, then `up` to 160 cm by downward ToF.
3. Yaw the box to the image center, then `forward 20` until forward ToF reads 0.9–1.1 m. While ToF has no reading, the total forward distance is limited to the taped start distance minus 1.0 m.
4. Trials, twice: 12 opposite pairs (`left 20`/`right 20`, `forward 20`/`back 20`, `cw 10`/`ccw 10`, `left 40`/`right 40`, `cw 20`/`ccw 20`, `cw 40`/`ccw 40`, `down 20`/`up 20`, plus reversed orders). Large pairs start with the command that swings the box toward the center. Before each pair, height, distance (ToF), heading (state yaw, once its sign is measured) and lateral offset (box) are corrected with logged `correction` moves.
5. `forward` to 0.8 m, then the yaw sweep: `cw 10` ×4, `ccw 10` ×8, `cw 10` ×4.
6. `land`.

## Safety rules

- Allowlist: SDK setup and queries, `takeoff`, `land`, `stop`, `speed 10–100`, moves of 20–100 cm, turns of 1–90°. No `rc`, `go`, `flip`, `motoron`, `throwfly` or `emergency`. `GroundLink` stays ground-only.
- The aircraft stops (`stop`, SDK 3.0) and lands on: Ctrl+C; forward ToF under 0.55 m (after `back 20`); battery under 25%; a state packet older than 1.5 s; downward height outside 40–230 cm; the target missing after a second look, a height correction and a turn back to the trial heading; more than 5 `error` replies; 2 reply timeouts; 8 minutes in the air.
- A `forward` step that would end under 0.55 m by ToF is refused.
- A command answered with `error` is observed (no motion expected) and sent once more.
- Ctrl+C is checked at least every 0.2 s while waiting for a reply. Landing ignores further Ctrl+C and retries `land` up to 5 times.
- If the program or laptop dies, the aircraft lands by itself 15 s after its last command (Tello SDK behavior). The aircraft's own Wi-Fi-loss behavior has not been measured.

## Setup

- Indoor room with a lit, textured floor (downward positioning), propeller guards, nobody within 2 m of the path except the operator.
- Cup print on the wall with its center about 1.47 m above the floor (13 cm below the 1.60 m flight height, as in [TT_MEASUREMENTS.md](TT_MEASUREMENTS.md)).
- Tape mark on the floor 1.5 m from the wall, facing the print square. Clear space: 1 m on each side and behind the mark; ceiling above 2.2 m.
- Battery at least 50%; `models/yolo11s.pt` present (download before joining aircraft Wi-Fi); laptop on `RMTT-…` Wi-Fi; Windows firewall prompt for Python allowed for UDP 8889, 8890 and 11111 ([TOF_BENCH.md](TOF_BENCH.md)); the Tello app closed.

```bash
python -m objectpilot.step_flight rehearse --out runs/step-rehearsal
python -m objectpilot.step_flight fly --out runs/step-cal-02 --weights models/yolo11s.pt --class-name cup
python -m objectpilot.step_flight analyze runs/step-cal-02/flight.jsonl
python -m objectpilot.stepsim --episodes 300 --seed 60000 --noise runs/step-cal-02/noise.json --out runs/stepwise-measured
```

Options: `--start-m` (taped distance, default 1.5), `--hover-m` (trial distance, 1.0), `--sweep-m` (0.8), `--height-cm` (160), `--repeats` (2), `--no-sweep`, `--mission-pad` (adds pad-relative position to the state log when a pad is within about 1.2 m below the aircraft), `--no-frames`.

Outputs in the `--out` folder: `flight.jsonl` (every command, reply, observation and state packet), `frames/`, `analysis.json` (per-command table, signs, scales, sweep fit) and `noise.json` (input for `stepsim --noise`).

## First flight, November 13, 2026

Indoor, cup print, 160 cm flight height, default plan. The flight completed: 72 of 72 step commands answered `ok`, no aborts, about 3 minutes from takeoff to landing. Raw log, frames and analysis stay local (`runs/`); the measured noise is kept as `configs/tt_step_noise_measured.json`.

| Measurement | Result |
|---|---|
| Image response signs | All as the planner assumes; state-stream yaw increases with `cw` |
| Move scale (median) | forward 0.89, back 0.91, left 0.92, right 0.99, up 0.80, down 0.85: moves fall about 10% short |
| Cross drift | median 0.7 cm, SD 3.9 cm per move; `left 40` ends about 5 cm closer to the wall, `right 40` about 4 cm farther |
| Rotation | Every turn falls short by about 2° (`cw 10` → 8°, `cw 20` → 18°, `ccw 4` → 2°); SD 1.3° at 10° |
| Heading change while translating | SD 0.37° |
| Command duration | 20 cm moves 1.7–1.9 s, 40 cm 2.0–2.1 s, 10° turns 0.7–0.8 s, 40° turns 1.2–1.3 s; zero residual speed at each observation |
| Video HFOV from turns | Median 54.2° (SD 13° per turn), matching the 54° assumption |
| ToF sweep at 0.76 m | Wall normal 1.0° from the trial heading; readings within ±30° match D/cos to ±4%, with no over-read (bench model: +0.35·(1/cos − 1)) |
| Vertical framing | At 160 cm the box center sat at 0.24–0.31 of the image height; `up 20` moved it to about 0.5. The print center was likely higher than 147 cm |

`noise.json` from this flight: move scale SD 0.133, cross drift 0.11 per metre, rotation SD 2.08°, heading change 0.37°, no `error` replies, no ToF over-read. The fit models the fixed 2° rotation shortfall as random error, which is conservative. With these numbers the planner squares up in 96.7% of the test-world starts ([STEPWISE.md](STEPWISE.md)).

## Rehearsal in the test world

`rehearse` runs the same sequence, guards and analysis against `objectpilot.stepsim`'s world instead of the network. Seeds 60600–60629:

| World noise | Completed | Closest to wall | Time (median) | Step commands (median) | Sweep normal error (median / max) |
|---|---|---|---|---|---|
| Default | 30 / 30 | 0.64 m | 197 s | 75 | 1.2° / 4.9° |
| Step errors doubled | 30 / 30 | 0.59 m | 213 s | 82 | 1.3° / 4.8° |

On one rehearsal (seed 60500), the analysis recovers the world's settings from the log: rotation error 1.02° + 2.4% (world: 1° + 3%), heading change per move 1.0° (1°), ToF oblique excess 0.33 (0.35), HFOV 53.2° ± 3.8° (54°), move scale medians 0.96–1.02 (1.0). Cross drift reads 0.042 per metre against 0.03 in the world. Box and ToF noise inflate it, so it errs on the conservative side. The sweep normal error comes from the assumed 0.3° per command state-yaw drift.

## Limitations

- One flight, one room, one print. Takeoff took 8.8 s and every move replied well inside its timeout. Timeouts are 20 s for takeoff, 15 s for landing, 6 s + 2 × distance / speed for moves and 6 s + angle / 30 for turns.
- Lateral move scale uses the 54° HFOV assumption. The rotation-based HFOV check is reported alongside, so a wrong HFOV shows up as a mismatch.
- The state-stream yaw is in whole degrees.
- The analysis fits one parameter set for all move sizes. The planner re-observes after every step, so it needs repeatability, not exactness.
