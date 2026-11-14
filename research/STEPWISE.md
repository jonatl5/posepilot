# Stop-and-go approach with SDK step commands

Status, November 13, 2026: planner, geometric test world and a step calibration flight tool ([STEP_FLIGHT.md](STEP_FLIGHT.md)). The calibration flight has flown once; the planner itself is not yet connected to the aircraft.

## Idea

Instead of streaming `rc` velocities at 10 Hz, the PC runs a loop: hover, look at about six frames and six `EXT tof?` replies, choose **one** SDK command, let the aircraft execute it and stop, repeat. Every command is a discrete Tello SDK move: `forward/back/left/right/up/down x` (20–500 cm) or `cw/ccw x` (1–360°), checked by `check_command`. Because the aircraft is stationary whenever it is observed, a command's effect on the target box is geometry, not dynamics. Latency, braking and motion blur do not enter the decision.

Code: `src/objectpilot/stepwise.py` (planner, deployable logic) and `src/objectpilot/stepsim.py` (test world and evaluation).

## Phases

| Phase | Trigger | Command |
|---|---|---|
| search | requested label not seen | `cw 40` (below the 54° video FOV, so views overlap); `not_found` after 360° |
| face | label seen, wall normal unknown | yaw by `atan(2·ex·tan 27°)` until \|ex\| ≤ 0.05; then `forward` 50/40/25/20 cm by box height. Before the wall normal is known, strafing would also change the wall distance, so centering uses yaw |
| sweep | ToF along the line of sight reads ≤ 1.10 m, **or** the box is tall (h ≥ 0.26) and ToF still has no reading after one more 20 cm step | yaw in 10° steps toward decreasing ToF until the minimum is bracketed; fit `D / cos(heading − normal)`. With no valid reading, first look ±20° and ±40°. If a neighbor of the minimum is out of range, approach 20 cm and sweep again |
| lateral | normal found | yaw to the normal, then strafe `D·tan(target − normal)` in chunks of at most 40 cm, backing off to 0.75 m first |
| center | box visible, squared | strafe by `ex · 2·tan 27° · ToF` (left if the box is left of center) |
| final | centered | ToF stand-off 0.65 m. Steps under 20 cm use two moves, always away from the wall first (`back 20`, `forward 20+δ`) |
| verify | first time within 5 cm of the stand-off | readings at 0, +10° and −10° refit the normal; re-center |
| arrived | three consecutive hovers in tolerance (about 1.8 s) | none |

Safety rules applied on every step: a ToF reading below 0.45 m commands `back 20`; a forward step may not end closer than 0.48 m by ToF; a tall box without ToF and no sweep left ends with `range_unavailable`; a command answered with `error` is re-sent up to twice.

Strafe drift can carry a squared aircraft past the ToF range during centering. In the final phase, if ToF is missing on two consecutive hovers, the last reading in the lateral, center or final phase was at least 0.95 m, and the box is no taller than 1.05 times its height at that reading, the planner steps `forward 20` (`range_beyond_tof`). It does this at most twice before a valid reading. A 20 cm step from at least 0.95 m ends at least 0.75 m from the wall. Otherwise it waits, then ends with `range_unavailable`.

When ToF is valid only within 1.2 m, the sweep is the hand-over: it runs once at about 1 m, where readings beside the normal are still in range, and once more as a short check at the stand-off.

## Test world

`StepWorld` projects each print's corners through a pinhole camera fitted to the cup stand clips in [TT_MEASUREMENTS.md](TT_MEASUREMENTS.md): 32° × 54° video FOV, 10.5° downward tilt, box height 0.328/d + 0.018. Against the stand clips, the model's box height is within 5% from 0.8 to 3.0 m, its width within 10%, and its vertical offset matches at 0.8, 1.0 and 2.0 m. The ToF model uses the whiteboard bench: 2 cm long head-on, 1.7–3.1 mm noise, readings beyond D/cos at oblique angles, and no reply above 1.22 m.

**Default command accuracy** (assumed before the flight): 8% scale error and 3 cm cross drift per move, 3% scale and 1° per rotation, 1° heading change per move, 2% `error` replies. The November 13 flight measured these (see below); `--noise` replaces them. **Still assumed:** detection falling from 35° to 65° viewing angle, the bottle and third print sizes, and the drift of the state-stream yaw.

## Flight video check

The two manual flights (November 7) were split into hovering and forward-moving segments by the rate of change of the visual distance. At the same distance, the box's vertical offset differs by at most 0.06 between hovering and moving (flight 1 at 0.7–1.0 m: −0.225 hovering, −0.218 moving), and detection confidence is the same (0.88–0.92). The target sits high in flight because of the lens-to-target height, not because of pitch while moving. The flight clips have no stick log, so the effect of a single lateral move or rotation cannot be read from them.

## Results (seeds 60000–60299)

Starts 1.5–3.2 m from the requested print, up to 50° off its normal; 40% start facing a random direction.

| Variant | Success | Final heading error (median) | Final offset (median) | Commands | Time | Below 0.40 m |
|---|---|---|---|---|---|---|
| Square-up (default) | 95.3% | 1.8° | 2.6 cm | 22 | 46 s | 0 |
| Square-up, commanded angles only (no IMU yaw) | 95.3% | 1.9° | 2.3 cm | 22 | 46 s | 0 |
| Yaw-center only (no sweep) | 15.0% | 22.7° | 24.2 cm | 10 | 24 s | 16 |
| Planner FOV 10% wrong | 95.0% | 1.9° | 2.1 cm | 22 | 46 s | 0 |
| Step errors doubled | 95.3% | 2.0° | 2.9 cm | 29 | 60 s | 0 |
| Detection 70% as likely | 89.3% | 1.7° | 2.0 cm | 22 | 47 s | 0 |
| ToF noise ×3, oblique error ×2 | 85.3% | 2.9° | 3.4 cm | 23 | 48 s | 0 |

Success requires measured arrival plus true stand-off within 8 cm, offset within 8 cm, heading within 8° and no pose closer than 0.40 m. All default-variant failures are `not_found`: starts beyond YOLO11s detection range. Yaw-only fails by construction beyond about 10°: it arrives facing the print along the line of sight, so it stops off to the side and at an angle, and its ToF stand-off is a slant distance.

```bash
python -m objectpilot.stepsim --episodes 300 --seed 60000 --example 60024 --plot 60018 60004 60024 60042 --out runs/stepwise
```

Seeds 10000–59999 belong to the PID/PPO splits and are refused.

### With the measured step noise (November 13 flight)

The step calibration flight ([STEP_FLIGHT.md](STEP_FLIGHT.md)) measured move scale spread 0.133, cross drift 0.11 per metre, rotation error 2.1° (in flight a fixed 2° shortfall on every turn), heading change per move 0.37° and no `error` replies, and found no ToF over-read within ±30°. With those numbers (`configs/tt_step_noise_measured.json`) on the same seeds:

| Variant | Success | Final heading error (median) | Final offset (median) | Commands | Time | Below 0.40 m |
|---|---|---|---|---|---|---|
| Square-up (default) | 96.7% | 1.6° | 2.5 cm | 26 | 55 s | 0 |
| Square-up, commanded angles only (no IMU yaw) | 90.3% | 2.0° | 2.8 cm | 25 | 54 s | 2 |
| Yaw-center only (no sweep) | 16.0% | 23.1° | 26.5 cm | 11 | 27 s | 11 |
| Detection 70% as likely | 88.0% | 1.6° | 2.3 cm | 25 | 55 s | 1 |

Before the `range_beyond_tof` rule, square-up reached 95.7%: three runs squared and centered at about 1.24 m after strafe drift and stopped with `range_unavailable`. The remaining ten failures are five `not_found` (two starts beyond 3.1 m, three at 47–49° where the assumed detection falloff loses the print) and five arrivals 9.2–10.6 cm off center with 4–6° heading error, just outside the 8 cm limit. Without the state-stream yaw, measured rotation error costs six points and two clearance violations, so the flight must keep using it.

```bash
python -m objectpilot.stepsim --episodes 300 --seed 60000 --noise configs/tt_step_noise_measured.json --out runs/stepwise-measured
```

## Step calibration flight

One supervised flight facing the cup print measured what the test world assumes: move scale and cross drift, rotation accuracy, heading change per move, `error` frequency, image response signs, video HFOV and ToF against heading. The procedure, safety rules and the November 13 results are in [STEP_FLIGHT.md](STEP_FLIGHT.md) (`python -m objectpilot.step_flight`). Still unmeasured: detection against viewing angle and the bottle and third print sizes.
