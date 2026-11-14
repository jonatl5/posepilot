# Planner approach flight

Status, November 13, 2026: flown four times on the delivered TT (results below). The planner was then changed to stop over-correcting, to search in smaller steps with lower-confidence candidates, and to stop 0.50 m from the wall. The changed version is simulated only.

`objectpilot.step_approach` connects `StepPlanner` ([STEPWISE.md](STEPWISE.md)) to the aircraft. It reuses the guarded `FlightSession` of the step calibration flight ([STEP_FLIGHT.md](STEP_FLIGHT.md)): the same allowlist, pre-flight checks, typed `fly` confirmation, takeoff, climb and abort-and-land. After the climb the planner takes over. Each hover gives one observation: 6 frames through YOLO, 6 `EXT tof?` replies and the state-stream yaw. The planner chooses one SDK step command, the aircraft executes it and stops, and the loop repeats until the planner reports `arrived`, `not_found`, `range_unavailable` or `timeout`. Then the aircraft lands.

## Safety layers

| Layer | Rule |
|---|---|
| Allowlist | Moves of 20–100 cm, turns of 1–90°, `takeoff`, `land`, `stop`, `speed`, queries. A planner command outside it aborts the flight (`planner_command_refused`) and lands. |
| Planner (minimum 0.35 m) | ToF under 0.40 m → `back 20`. A forward step must end at least 0.43 m from the wall, and at least 0.38 m if it overshoots by 30%. No strafe closer than 0.70 m: back off first. A target lost inside the stand-off → `back 20` before turning. At most 90 commands and 250 observations. |
| Flight layer | ToF under 0.32 m → `back 20` and land; a `forward` that would end under 0.32 m is refused and lands; battery under 25%; state packet older than 1.5 s; height outside 40–230 cm; more than 5 `error` replies or 2 timeouts; 4 minutes in the air. A takeoff answered with an error (for example `error, high temp`) means the motors never started: the flight ends as `takeoff_refused` without landing commands. |
| Operator | Ctrl+C stops (`stop`) and lands at any time. If the program dies, the aircraft lands by itself 15 s after its last command. Planner waits longer than 8 s send `battery?` so a long hover does not trigger that timeout. |

## Setup

- The same room setup as the calibration flight: lit, textured floor, propeller guards, nobody within 2 m of the path, firewall prompt allowed, the Tello app closed, `models/yolo11s.pt` present.
- **Flight height:** the camera tilts 10.5° down, so at a stand-off of s metres the print center should sit 100·s·tan 10.5° cm below the lens: 9 cm at 0.50 m. The checklist prints the matching print-center height. For the cup print at 148 cm, use `--height-cm 157`. The planner corrects height itself only beyond 0.8 m, and at most twice.
- **Battery:** pre-flight requires 40%. At low charge the battery falls fast: 51% → 24% in 55 s on November 13. An approach takes 35–90 s, and the flight lands at 25%. Starting below about 55% risks a landing before arrival.
- **Heat:** the TT refuses takeoff with `error, high temp` after a long time powered on with video. Power it off for a few minutes or cool it.
- Start on the floor 1.5–3 m from the wall, within about 45° of the print's normal. Clear path; nothing else within 1 m of the line to the print.

```bash
python -m objectpilot.step_approach simulate --episodes 300 --noise configs/tt_step_noise_measured.json --out runs/approach-sim
python -m objectpilot.step_approach fly --class-name cup --weights models/yolo11s.pt --height-cm 157 --out runs/approach-01
python -m objectpilot.step_approach summary runs/approach-01/flight.jsonl
```

Options:
- both modes: `--stand-off-m` (0.50) and `--minimum-m` (0.35); the flight aborts 3 cm inside the minimum, and the stand-off must be at least 12 cm beyond the minimum. Also `--max-seconds` (240).
- `fly`: `--confidence` (0.25: the detector threshold for search candidates; the planner still confirms at 0.45), `--imgsz` (960, the TT stream width) and `--no-frames`.
- `simulate`: `--episodes`, `--seed` (60000 or higher), `--noise`, `--height-cm` (154 for the test world's 145 cm prints) and `--keep-logs`.

`fly` writes:
- `flight.jsonl`: every command with the planner's phase and reason, every planner decision, observation and state packet
- the observed frames
- `summary.json`: outcome, commands by phase and reason, waits, the closest ToF reading, and the final box, range and yaw

## Flights on November 13, 2026

Cup print center at 148 cm, flight height 160 cm, stand-off 0.65 m, the planner as first written.

| Flight | Start | Result |
|---|---|---|
| 1 | 1.5 m straight in front, facing the print | Takeoff refused: `error, high temp`. The script then sent `land` five times and warned that landing was not confirmed; fixed (see Safety layers). |
| 1, repeat | Same | **Arrived** in 38 s, 13 planner commands: face, sweep, one 38 cm strafe, one forward step, verify, 0.66 m. |
| 2 | 1.8 m from the print, 30° off its normal | **Arrived** in 76 s, 25 commands, 17 of them centering and distance corrections (see [STEPWISE.md](STEPWISE.md)). |
| 3 | 1.8 m in front, turned away; other prints on the wall as distractors | Found the cup after two `cw 40` turns and squared up; landed at 25% battery during the verify refit (51% at takeoff). |
| 3, repeat | Same; three backpack prints beside the cup and a person in view | 9 × `cw 40` covered 360° (state yaw 0, 38, 77, …, −14), then `not_found`. |

Three more attempts stopped at pre-flight on the old 50% battery threshold (40%, 17% and 45%), and one with no SDK reply.

**Why the last search missed.** The saved frames show the cup print sharp and complete at yaw 77° (at the right edge, cx 0.83) and at 115° (left edge, cx 0.13). It sat about halfway between two 40° views. These are not motion-blurred or stale frames: every observation's six frames match each other. YOLO11s found it in each frame, but below the 0.45 threshold:

| Frame | Detected at 640 px (as flown) | Detected at 960 px |
|---|---|---|
| yaw 77°, cup at the right edge | 0.43, 0.16 | 0.36, 0.39 |
| yaw 115°, cup at the left edge | 0.19, 0.20 | 0.60, 0.54 |

SDK rotations have a fixed rate. Every observation happens after the turn has stopped, so a slower turn would not have changed the frames. Changes:
- 30° search steps, so the print is within 15° of some view's center.
- Inference at the 960 px stream width.
- A 0.25 detector threshold. A box below 0.45 is a candidate: the planner turns it to the image center and requires 0.45 within two looks. Otherwise it skips that bearing and searches on.

## Simulation through the full flight loop

`simulate` runs `ApproachFlight` against the geometric test world from the `stepsim.scenario` starts: 1.5–3.2 m away, up to 50° off the print's normal, 40% facing a random direction. It runs the same code as `fly`, including takeoff, climb, the flight-layer guards and landing. It is scored with the same rule as stepsim: arrival with the true stand-off within 8 cm, offset within 8 cm, heading within 8°, and never inside the minimum. Seeds 60000–60299, stand-off 0.50 m, height 154 cm:

| Noise | Success | 0–15° / 15–30° / 30–50° | Heading error (median) | Offset (median) | Commands | Closest to wall |
|---|---|---|---|---|---|---|
| Default (assumed) | 98.0% | 100 / 99.0 / 95.2% | 1.7° | 2.3 cm | 20 | 0.42 m |
| Flight-measured | 96.3% | 96.7 / 99.0 / 93.3% | 1.5° | 2.5 cm | 25 | 0.41 m |

With the flight-measured noise, the planner as flown on November 13 reaches 71.0% here, with a median of 19 centering and distance corrections; the changed planner needs 7. Remaining failures with that noise: five `not_found` (starts beyond 3 m, where YOLO11s cannot detect the print, or 45–50° off the normal, where the assumed detection falloff applies) five arrivals 8.1–9.2 cm off center, and one that stopped at 0.42 m. Simulated time is not flight time: a simulated observation takes 0.6 s, a real one about 1 s.

## Limitations

- The changed planner has not flown.
- Detection confidence is not modeled in the test world (every simulated detection scores 0.9), so the candidate search is covered by unit tests and the flight frames only.
- Detection against viewing angle is assumed (falling from 35° to 65°). Measure it on the stand before trusting oblique starts.
- Only the cup print is measured. The bottle and third prints, and their sizes, are unscreened.
