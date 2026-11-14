# TT camera measurements, November 5–7, 2026

These are the first measurements on the delivered aircraft. The aircraft sat on a stand or was held by hand, motors off. Clips were recorded with the Tello app (1280×720) and analyzed with pretrained YOLO11 on a Windows laptop CPU. Raw clips stay local under `data/` and are ignored by Git. No flight, ToF or latency result is included.

## Setup

- Target prints on letter-size matte paper, taped to the whiteboard: a red backpack (Nov 5) and a yellow cup (Nov 5 and Nov 7).
- Camera lens about 1.60 m above the floor. Print object center about 1.40 m.
- Distances are taped from the wall to the lens. The Nov 5 backpack clips were handheld; all cup clips used a stand.
- App recordings were 1280×720 at an average of 11–30 fps, below the nominal 30p. Clips forwarded through a chat app can be recompressed, so copy originals over USB.

## Detection

Share of frames with the target class at confidence ≥ 0.45 (`confidence_min`):

| Print, model | 0.5 | 0.65 | 0.8 | 1.0 | 1.2 | 1.5 | 2.0 | 2.5 | 3.0 m | CPU per frame |
|---|---|---|---|---|---|---|---|---|---|---|
| Backpack, YOLO11n | – | – | – | – | – | 0% | 0% | 0% | 0% | 19–35 ms |
| Backpack, YOLO11s | – | – | – | – | – | 0% | 0% | 0% | 0% | 43–68 ms |
| Backpack, YOLO11m | – | – | – | – | – | 20% | 0% | 0% | 0% | 106–172 ms |
| Cup, YOLO11n | 100% | 100% | 100% | 100% | 100% | 100% | 100% | 39% | 0% | 19 ms |
| **Cup, YOLO11s** | 100% | 100% | 100% | 100% | 100% | 100% | 100% | 100% | 96% | **43 ms** |
| Cup, YOLO11m | – | – | – | – | – | 100% | 100% | 100% | 100% | 106 ms |

The backpack print is split between handbag, suitcase and backpack at low confidence; it is dropped as a target. The cup print works with **YOLO11s**, which also leaves time inside the 100 ms control period. YOLO11m does not.

## Visual scale k (cup print, YOLO11s box)

| Taped distance (m) | 0.8 | 1.0 | 1.2 | 1.5 | 2.0 | 2.5 | 3.0 |
|---|---|---|---|---|---|---|---|
| Box height fraction | 0.4285 | 0.3433 | 0.2832 | 0.2364 | 0.1869 | 0.1497 | 0.1277 |
| distance × height | 0.343 | 0.343 | 0.340 | 0.355 | 0.374 | 0.374 | 0.383 |

- All valid points fit `height ≈ 0.328 / distance + 0.018` within about 1%. The constant term is roughly a fixed box margin.
- The single-parameter controller model `distance = k / height` uses **k = 0.343**, fitted on 0.8–1.2 m. That is the visual-to-ToF handover region. It reads 0.80, 1.00 and 1.21 m at the three fit points, 1.45 m at 1.5 m, and underestimates distance by 8–10% at 2–3 m. Underestimating far away only slows the approach.
- 0.5 m and 0.65 m are excluded: the cup touches the bottom image edge there (box bottom ≥ 0.99) and the box is truncated.
- A clip labeled 1.5 m from Nov 7 reads as about 1.25 m with this k and was not used.

## Vertical geometry

Target vertical offset `ey = cy − 0.5` (negative = above center), YOLO11s:

| Distance (m) | 0.65 | 0.8 | 1.0 | 1.2 | 1.5 | 2.0 | 2.5 | 3.0 |
|---|---|---|---|---|---|---|---|---|
| ey | +0.24 (cut off) | +0.11 | +0.03 | −0.10 | −0.06 | −0.15 | −0.22 | −0.24 |

Fitting these points gives a camera tilted down by about **10.5° ± 1°**. With the paper assumed to be letter size, the recorded video's vertical field of view is about 32° (horizontal about 53°), much narrower than the 82.6° quoted for stills. Repeated stand placements differ by about ±2° in pitch, which is ±0.07 in `ey`.

No fixed height difference keeps the target within the original `vertical_gate` of 0.15 from 0.65 m to 3 m; the best case is about ±0.21. The supervisor blocks forward motion whenever |ey| exceeds `vertical_gate`, and the controller has no vertical action.

## Configuration decision: `configs/tt_cup_measured.json`

A copy of `configs/wall_approach.json` with two changes:

| Field | Was | Now | Reason |
|---|---|---|---|
| `visual_scale_m` | 0.22 (placeholder) | 0.343 | Cup print, YOLO11s, 720p TT video |
| `vertical_gate` | 0.15 | 0.30 | Fixed-altitude geometry plus hover pitch noise; 0.15 blocks the approach beyond about 2 m |

Physical setup that goes with it: lens and flight height 1.60 m, cup center about **1.47 m** (13 cm below the lens). This keeps the target near the image center from 0.65 to 1.2 m, where arrival and ToF association happen. Far away the target sits above center, within the relaxed gate.

Checks on November 7:

- **Dry-run replay** of the cup clips with YOLO11s. The original configuration blocked forward motion at 2.5 and 3.0 m (`centering`). At 1.2 m it latched the near-range requirement early, because k = 0.22 read 0.78 m. With the measured configuration, 1.2–3.0 m is `tracking` with forward RC 17–20. From 1.0 m and closer it waits for ToF (`near_range_unavailable`), which a video file cannot supply.
- **Simulator**: tuned PID on validation seeds 30000–30099 still scores 92/100 with zero minimum-distance violations. The scale-invariant simulator is unaffected by k.

`configs/wall_approach.json` stays the canonical simulator and PPO configuration, so the committed tuning evidence still reproduces. Control-v2 PPO checkpoints match it, not the measured file. Retrain them for the measured configuration after the simulator is updated with flight measurements. The measured values apply only to this cup print, YOLO11s and 720p TT video. Re-run the stand ladder after changing any of them.

## Forward ToF bench, whiteboard (November 7)

Stock factory firmware `esp32v1.0.0.22` after activating the expansion controller in the Tello app. `python -m objectpilot.tof_bench`: aircraft level on a stand, motors off, 10 s per point, queried at up to 10 Hz. The tape distance runs from the board to the ToF window. A desk fan cooled the aircraft. Without it, the TT powered itself off at a board temperature (`temph`) of 99 °C after about 15 minutes idle. Raw sessions are under `runs/tof/2026-11-08-whiteboard*`.

| Tape distance, angle | Valid | Median | Bias | Noise (SD) |
|---|---|---|---|---|
| 0.25 m, 0° | 99% | 0.248 m | −0.2 cm | 0.8 mm |
| 0.40 m, 0° | 99% | 0.426 m | +2.6 cm | 0.9 mm |
| 0.50 m, 0° | 100% | 0.508 m | +0.8 cm | 1.4 mm |
| 0.65 m, 0° | 100% | 0.662 m | +1.2 cm | 1.7 mm |
| 0.80 m, 0° | 100% | 0.831 m | +3.1 cm | 1.8 mm |
| 1.00 m, 0° | 99% | 1.019 m | +1.9 cm | 3.1 mm |
| 1.10 m, 0° | 100% | 1.116 m | +1.6 cm | 2.8 mm |
| 1.20 m, 0° | 100% | 1.222 m | +2.2 cm | 3.5 mm |
| 1.30 m and 1.50 m, 0° | 0% | `tof 8190` on every reply | – | – |
| 0.65 m, 20° | 99% | 0.725 m | +7.5 cm | 2.4 mm |
| 1.00 m, 20° | 96% | 1.084 m | +8.4 cm | 5.4 mm |
| 0.65 m, 40° | 98% | 0.949 m | +29.9 cm | 4.0 mm |

- **Range:** valid up to 1.2 m and out of range by 1.3 m on the whiteboard. This matches `forward_range_max_m` = 1.2. The cup print and the dark print are still to be measured.
- **Out-of-range reply is 8190, not the documented 8192.** Before the November 7 fix, `parse_ext_tof` accepted it as 8.19 m. The control gate never used it, because valid range is capped at 1.2 m, but the parser now rejects 0 and every value ≥ 8190.
- **Accuracy:** head-on readings are linear (slope 1.014, residual RMS 0.8 cm) and read about 2 cm long from 0.4 to 1.2 m. The spread from point to point (+0.8 to +3.1 cm) points to stand placement and to a sensor zero slightly behind the window. Correct with a fixed offset after the remaining surfaces are measured.
- **Oblique:** the beam measures the slant distance, not the perpendicular clearance. At 20° it over-reads the clearance by 7–8 cm, and at 40° by about 30 cm, with occasional out-of-range replies. This confirms the oblique-ray effect in [simulator finding 3](CONTROLLER_TUNING.md#three-simulator-findings-that-gains-cannot-fix). Near the wall the heading must stay close to perpendicular before the reading can be trusted as clearance.
- **Freshness:** values change 4–5 times per second at 0.5–1.2 m, consistent with the stock 200 ms timing budget. The longest run of an identical value was 0.3–1.2 s; it was longer at short range, where noise is below 1 mm. Stock replies still carry no sample age, so the current gate does not accept them for near approach. Choosing between the `EXT oprange?` firmware and an explicit stock-age policy is open.

## First manual flights (November 7)

Two approach clips were recorded in the Tello app while flying manually, then forwarded through a chat app, so they are recompressed. Each was analyzed frame by frame with YOLO11s, k = 0.343, and a dry replay with `configs/tt_cup_measured.json`.

| Clip | Length | Cup detected (conf ≥ 0.45) | Longest miss after first lock | Vertical offset `ey` | Final hover (visual distance) |
|---|---|---|---|---|---|
| 1 | 26.6 s | 92% overall, 94% after first lock | 34 frames (about 1.2 s) at about 2.2 m | −0.37 at about 3 m, −0.30 to −0.33 at 1.2–1.6 m, −0.16 to −0.26 at 0.8 m | 0.77–0.82 m |
| 2 | 34.4 s | 42% overall, 89% after first lock at 18 s | 24 frames | −0.31 at 2.6 m, −0.15 to −0.20 at 1.0 m | 1.03–1.05 m |

- **Detection in flight** is reliable inside about 2.8 m, with confidence around 0.9 near the wall. Beyond about 2.8 m, and with the cup near the top edge of the frame, confidence falls below 0.45. Clip 2's first 18 s, at about 3.4 m, has no detection.
- **The visual distance from k holds in flight.** Hovers planned at the 0.8 m and 1.0 m lines read 0.77–0.82 m and 1.03–1.05 m.
- **In flight the cup sits higher in the image than on the stand.** At 0.8 m the stand gave +0.11; the flights gave −0.16 to −0.26. That corresponds to the lens only about 6 cm above the cup center, plus extra nose-down pitch while moving forward. In the dry replay of clip 1, the supervisor reports `centering` from 2.3 to 12.7 s, because |ey| ≥ 0.30, so the approach would stall at 1.3–2.6 m.
- **Action:** increase the lens-above-target height to about 15 cm. Lowering the print by about 10 cm is simpler than flying higher. Start the demo approach at about 2.5 m instead of 3 m. Copy future clips over USB.
