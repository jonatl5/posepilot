# Stop-and-go approach with SDK step commands

Status, November 13, 2026: planner, geometric test world, a step calibration flight ([STEP_FLIGHT.md](STEP_FLIGHT.md), flown once) and the planner approach flight ([STEP_APPROACH.md](STEP_APPROACH.md)). The approach was flown four times on November 13. The planner was then changed to reduce over-correction (see [Changes after the November 13 approach flights](#changes-after-the-november-13-approach-flights)); the changed planner has not flown.

## Idea

Instead of streaming `rc` velocities at 10 Hz, the PC runs a loop: hover, look at about six frames and six `EXT tof?` replies, choose **one** SDK command, let the aircraft execute it and stop, repeat. Every command is a discrete Tello SDK move: `forward/back/left/right/up/down x` (20–500 cm) or `cw/ccw x` (1–360°), checked by `check_command`. Because the aircraft is stationary whenever it is observed, a command's effect on the target box is geometry, not dynamics. Latency, braking and motion blur do not enter the decision.

Code: `src/objectpilot/stepwise.py` (planner, deployable logic) and `src/objectpilot/stepsim.py` (test world and evaluation).

## Phases

| Phase | Trigger | Command |
|---|---|---|
| search | requested label not seen | `cw 30`, so the print is within 15° of some view's center; `not_found` after 360°. A box below `confirm_conf` (0.45) is a candidate: yaw it to the center, then require 0.45 within two looks or skip that bearing |
| face | label seen, wall normal unknown | yaw by `atan(2·ex·tan 27°)` until \|ex\| ≤ 0.05; then `forward` 50/40/25/20 cm by box height. Before the wall normal is known, strafing would also change the wall distance, so centering uses yaw |
| sweep | ToF along the line of sight reads ≤ 1.10 m, **or** the box is tall (h ≥ 0.26) and ToF still has no reading after one more 20 cm step | yaw in 10° steps toward decreasing ToF until the minimum is bracketed and the readings reach 20° on both sides of it; fit `D / cos(heading − normal)`. With no valid reading, first look ±20° and ±40°. If a neighbor of the minimum is out of range, approach 20 cm and sweep again |
| lateral | normal found | yaw to the normal, then strafe `D·tan(target − normal)` in chunks of at most 40 cm, backing off to 0.75 m first |
| center | box visible, squared | strafe by `ex · 2·tan 27° · ToF` (left if the box is left of center) when it exceeds 4 cm |
| final | centered | ToF stand-off (0.65 m in this test world; 0.50 m in the approach flight). Steps under 20 cm use two moves, always away from the wall first (`back 20`, `forward 20+δ`) |
| verify | off by default (`verify_deg` 0) | a ±`verify_deg` refit of the normal at the stand-off |
| arrived | three consecutive hovers in tolerance (about 1.8 s) | none |

Safety rules applied on every step:
- A ToF reading below minimum + 5 cm commands `back 20`.
- A forward step must end at least `forward_margin_m` (8 cm) beyond the minimum, and at least 3 cm beyond it even if it overshoots by 30%. This covers queued steps too.
- No strafe closer than `strafe_clearance_m` − 5 cm (0.70 m): back off first. Small strafes overshoot by 20 cm and return, so every strafe has a leg of at least 20 cm.
- A tall box without ToF and no sweep left ends with `range_unavailable`.
- A command answered with `error` is re-sent up to twice.

Strafe drift can carry a squared aircraft past the ToF range during centering. In the final phase, if ToF is missing on two consecutive hovers, the last reading in the lateral, center or final phase was at least 0.95 m, and the box is no taller than 1.05 times its height at that reading, the planner steps `forward 20` (`range_beyond_tof`). It does this at most twice before a valid reading; a 20 cm step from at least 0.95 m ends at least 0.75 m from the wall. Otherwise it waits, then ends with `range_unavailable`.

If the target is lost during centering or the final phase while ToF reads closer than the stand-off, the planner backs off 20 cm (`lost_close_back_off`, at most twice) before the usual waits and reacquire turns. An overshoot there puts the print partly out of the frame, and turning in place cannot find it again.

When ToF is valid only within 1.2 m, the sweep is the hand-over: it runs once at about 1 m, where readings beside the normal are still in range.

## Test world

`StepWorld` projects each print's corners through a pinhole camera fitted to the cup stand clips in [TT_MEASUREMENTS.md](TT_MEASUREMENTS.md): 32° × 54° video FOV, 10.5° downward tilt, box height 0.328/d + 0.018. Against the stand clips, the model's box height is within 5% from 0.8 to 3.0 m, its width within 10%, and its vertical offset matches at 0.8, 1.0 and 2.0 m. The ToF model uses the whiteboard bench: 2 cm long head-on, 1.7–3.1 mm noise per reply, readings beyond D/cos at oblique angles, and no reply above 1.22 m.

**Default command accuracy** (assumed before any flight): 8% scale error and 3 cm cross drift per move, 3% scale and 1° per rotation, 1° heading change per move, 2% `error` replies, no fixed shortfall, no per-hover ToF error. `--noise` replaces them with flight numbers. **Still assumed:** detection falling from 35° to 65° viewing angle, the bottle and third print sizes, and the drift of the state-stream yaw.

## Flight video check

The two manual flights (November 7) were split into hovering and forward-moving segments by the rate of change of the visual distance. At the same distance, the box's vertical offset differs by at most 0.06 between hovering and moving (flight 1 at 0.7–1.0 m: −0.225 hovering, −0.218 moving), and detection confidence is the same (0.88–0.92). The target sits high in flight because of the lens-to-target height, not because of pitch while moving.

## Results (seeds 60000–60299)

Starts 1.5–3.2 m from the requested print, up to 50° off its normal; 40% start facing a random direction. Stand-off 0.65 m, minimum 0.40 m.

| Variant | Success | Final heading error (median) | Final offset (median) | Commands | Time | Below 0.40 m |
|---|---|---|---|---|---|---|
| Square-up (default) | 95.3% | 1.9° | 2.9 cm | 19 | 41 s | 0 |
| Square-up, commanded angles only (no IMU yaw) | 89.0% | 2.2° | 2.5 cm | 19 | 40 s | 0 |
| Yaw-center only (no sweep) | 14.0% | 22.9° | 24.6 cm | 10 | 26 s | 22 |
| Planner FOV 10% wrong | 92.3% | 2.2° | 2.8 cm | 19 | 41 s | 0 |
| Step errors doubled | 91.0% | 2.2° | 3.1 cm | 25 | 52 s | 0 |
| Detection 70% as likely | 91.3% | 1.8° | 2.7 cm | 20 | 45 s | 0 |
| ToF noise ×3, oblique error ×2 | 85.7% | 2.4° | 3.7 cm | 19 | 41 s | 0 |

Success requires measured arrival plus true stand-off within 8 cm, offset within 8 cm, heading within 8° and no pose closer than the minimum. Yaw-only fails by construction beyond about 10°: it arrives facing the print along the line of sight, so it stops off to the side and at an angle, and its ToF stand-off is a slant distance.

```bash
python -m objectpilot.stepsim --episodes 300 --seed 60000 --example 60024 --plot 60018 60004 60024 60042 --out runs/stepwise
```

Seeds 10000–59999 belong to the PID/PPO splits and are refused.

### With the flight-measured noise

`configs/tt_step_noise_measured.json` comes from the November 13 calibration flight ([STEP_FLIGHT.md](STEP_FLIGHT.md)):

- moves fall 8.7% short, with a spread of 10.6% around that, and cross drift of 0.11 per metre
- every turn falls 2.0° short, with ±1.0° + 3.9% around that; heading change per move is 0.37°
- ToF has 1.3 cm of error shared by one hover's replies (sweep residuals), and no over-read within ±30°
- no `error` replies

| Variant | Success | Final heading error (median) | Final offset (median) | Commands | Time | Below 0.40 m |
|---|---|---|---|---|---|---|
| Square-up (default) | 95.0% | 1.4° | 2.5 cm | 24 | 52 s | 0 |
| Square-up, commanded angles only (no IMU yaw) | 76.3% | 3.1° | 4.4 cm | 25 | 52 s | 1 |
| Yaw-center only (no sweep) | 13.0% | 23.5° | 27.4 cm | 12 | 30 s | 13 |
| Detection 70% as likely | 88.0% | 1.5° | 2.9 cm | 25 | 56 s | 0 |

Without the state-stream yaw the planner loses 19 points: it cannot hold the heading or learn the turn shortfall. The flights must keep the state stream.

```bash
python -m objectpilot.stepsim --episodes 300 --seed 60000 --noise configs/tt_step_noise_measured.json --out runs/stepwise-measured
```

## Changes after the November 13 approach flights

In the 30° oblique approach flight, the planner reached the print and then took 17 centering and distance corrections. It strafed in overshoot-and-return pairs that netted about nothing, and stepped back and forth between 0.55 and 0.70 m. Three causes:

1. **The ±10° verify refit was noise.** D/cos changes by 1.5% at 10°, about 1 cm at the stand-off. That equals the 1.3 cm per-hover ToF scatter measured in the calibration sweep. The refits moved the heading by −3° and +8°; strafing 8° off square then changed the wall distance (ToF 0.686 → 0.765).
2. **Tolerances of 4–5 cm matched the step precision.** A 7 cm correction is `right 27`, `left 20`; with about 10% spread per leg it nets 7 ± 4 cm. A hover at 0.70 m inside tolerance drifted out by one reading and restarted the corrections.
3. **The test world was too clean to show it.** It had no fixed move or turn shortfall and no per-hover ToF scatter. With those added (`move_scale_bias`, `yaw_bias_deg`, `tof_obs_sd_m`, estimated by `step_flight analyze`), the old planner needs a median of 18 centering and distance corrections (flight: 17) and reaches 75% (table below).

Planner changes (`StepConfig`):
- **Wider sweep:** the sweep spans ±20° around the minimum before fitting (`sweep_span_deg`).
- **No verify refit:** `verify_deg` is 0.
- **Hysteresis:** once the hold has started, its tolerances widen by 1.4 (`hold_slack`).
- **Correction budget:** after three small corrections on an axis, that axis's tolerance widens by 1.5 (`fine_budget`, `budget_widen`).
- **Step learning:** the planner learns the turn shortfall from the state-stream yaw and the forward/back scale from ToF, and compensates both (`learn_steps`). Forward steps are still capped at the overshoot-safe length.
- **Search:** steps are 30° instead of 40°, with search candidates (see [STEP_APPROACH.md](STEP_APPROACH.md)).
- **Safety:** the strafe-clearance rule and the overshoot-safe forward cap. In the measured-noise test world, a 31 cm overshoot strafe from 0.53 m and a 45 cm step that moved 58 cm had come within 0.27–0.37 m.

Full flight loop (`step_approach simulate`) with the measured noise, 300 starts:

| Stand-off | Planner | Success | Centering and distance corrections (median / 90th percentile) | Commands (median) |
|---|---|---|---|---|
| 0.65 m | Before | 74.7% | 18 / 34 | 35 |
| 0.65 m | After | 93.7% | 6 / 16 | 23 |
| 0.50 m | Before | 71.0% | 19 / 38 | 36 |
| 0.50 m | After | 96.3% | 7 / 17 | 25 |

"Before" keeps the old tolerances, verify, search and no learning, but already includes the two clearance rules. The comparison script is not part of the repository; the variants are `StepConfig` fields.

## Step calibration flight

One supervised flight facing the cup print measured what the test world assumes: move scale and cross drift, rotation accuracy, heading change per move, `error` frequency, image response signs, video HFOV and ToF against heading. The procedure, safety rules and the November 13 results are in [STEP_FLIGHT.md](STEP_FLIGHT.md) (`python -m objectpilot.step_flight`). Still unmeasured: detection against viewing angle and the bottle and third print sizes.
