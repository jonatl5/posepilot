# TT camera recording plan

Team decision, November 5, 2026: detector footage is recorded with the delivered RoboMaster TT's own camera, using the Tello app's built-in recorder while a pilot flies the aircraft manually. Phone-camera clips are no longer the main dataset. This page gives the shot list, the duration of every clip, what each clip must cover, and the session order. Platform facts behind these choices are in [TT_PLATFORM.md](TT_PLATFORM.md). Labeling, splitting and training follow [DATA_COLLECTION.md](DATA_COLLECTION.md).

## Status after the first measurements (November 7)

The first stand and handheld recordings ([TT_MEASUREMENTS.md](TT_MEASUREMENTS.md)) changed parts of this plan. Where this section and the sections below differ, this section applies.

- **Target and detector:** the cup print is the first target class, detected with **YOLO11s** (`--weights models/yolo11s.pt --class-name cup`). The backpack print was dropped: no pretrained model detects it reliably.
- **Height:** lens and flight height 1.60 m; target object center about **1.47 m**. The camera tilts down about 10.5°, so the target sits slightly below the lens.
- **Calibration ladder (C1):** stand, motors off, aircraft level, distance taped from the wall to the lens. **One 6–15 s clip per distance** at 0.65, 0.8, 1.0, 1.2, 1.5, 2.0, 2.5 and 3.0 m, named `handheld_<distance>m_<class>.mp4`. Points at 0.65 m and closer only check framing; k uses 0.8–1.2 m. Done for cup print A at the old 1.40 m mount.
- **Handheld moving clips** may replace flight clips for detector coverage, with `device=tt_app_handheld`. Hover dropout still needs real flights (F4).
- Copy originals over USB, not through a chat app. Video files under `data/` are ignored by Git.

**Next recordings, in order:**

1. **ToF bench**, hardware gate 3, following [TOF_BENCH.md](TOF_BENCH.md). Three short sessions, about 30 min in total.
2. **Re-mount check:** cup center at 1.47 m, stand clips at 0.65, 1.2 and 3.0 m, about 8 s each. Confirms the target is fully in frame at 0.65 m and `ey` stays within ±0.3 at 3 m.
3. **Cup A detector coverage, handheld at 1.60 m, about 5 min.**
   - Walk approach 3.0 → 0.8 m (30 s).
   - Yaw sweep with the target leaving the frame on both sides, at 2.0 m and 1.2 m (40 s).
   - Offset approaches from ±0.5 m (2 × 30 s).
   - Fast yaw for blur (20 s).
   - Occlusion with a sheet (40 s).
   - Approach under dimmed or side light (30 s).
4. **Negatives and distractors** (handheld, 30 s each).
   - Empty whiteboard.
   - Wall with other posters, including a bottle and a different-looking mug.
   - Two cup prints 0.5 m apart.
5. **Held-out cup B:** a different mug picture. Stand ladder at 0.8, 1.0, 1.2, 2.0 and 3.0 m, plus one approach and one yaw sweep. Tests detector generalization and how much k changes between prints.
6. **After the flight sink exists:** F1 and F4 flights at 1.60 m for hover dropout and vibration.

## What these recordings establish

| Can establish | Cannot establish |
|---|---|
| The real TT image domain at 1280×720 / 30p: print appearance, blur, flight vibration and Wi-Fi artifacts | Forward ToF behavior. The app does not query or log `EXT tof?`. |
| Pretrained YOLO detection rate by distance, offset and lighting; false positives on distractors and empty walls | Camera-to-host latency. The MP4 timeline is created on the phone, not when the frame was exposed. |
| Labeled frames for an optional fine-tune, plus a held-out test print family | RC command response, braking or stopping drift. Manual stick input is not logged. |
| Visual-scale calibration per print (handheld ladder) | Any flight-performance or clearance claim |
| Per-frame detection dropout while hovering, an input to [simulator finding 1](CONTROLLER_TUNING.md#three-simulator-findings-that-gains-cannot-fix) | |

Range, latency and response come from separate laptop SDK sessions ([hardware gates](HARDWARE_GATES.md) 2–6) with the app closed.

## Set up each room once

| Item | Setting |
|---|---|
| Height H | Superseded on Nov 7: lens 1.60 m, target center about 1.47 m (see the status section). Originally 1.0 m with the print center at H. |
| Prints | Matte paper. Start with A3; in Session 0, confirm the depicted object is fully in frame at 0.65 m and visible at 3.0 m. Give every print an ID such as `bottle-A`. A letter is a print family: a different source picture. |
| Distance reference | Measured horizontally from the wall to the **camera lens** at the aircraft's nose. Write this in the log. |
| Floor marks | Tape lines parallel to the wall at 0.4, 0.5, 0.65, 0.8, 1.0, 1.2, 1.5, 2.0, 2.5 and 3.0 m. One center line runs perpendicular to the wall through the print center. Add lateral marks ±0.5 m from the center line at 2.5 m and 3.0 m. |
| Floor texture and light | A non-repeating, textured floor under the whole path, such as a patterned rug or randomly placed tape. No glossy, monochrome or tiled floor. At least 300 lux, no sunlight in the frame and no lights switching during a clip. |
| App | Expansion-controller switch in direct-connection mode (SSID `RMTT-XXXXXX`). Video mode, landscape, no zoom. Slow speed mode. Note the app version and phone model. |
| Safety | Propeller guards on. A pilot and a spotter. Nobody between the aircraft and the wall. **Flights stay at or beyond the 0.8 m line.** Shots closer than 0.8 m, and any shot with a person near the print, are handheld with the motors off. Land on a low-battery warning, an Attitude-mode warning or any uncommanded drift. These are data-collection rules, not qualified clearances. |
| Battery | DJI rates about 8 min with the kit mounted. Plan **at most 5 min of recording per battery**; positioning, takeoff and landing use the rest. Optionally remove the expansion kit for recording flights only (13 min rated). The camera is unchanged, but log `kit=off` and reinstall it for SDK sessions. |

## Clip conventions

- One clip is one press of Record and produces one MP4. Durations are recording time, ±5 s. Start recording once the aircraft hovers at the start mark. Takeoff and landing stay outside the clip.
- Keep motion gentle: about 0.2–0.3 m/s, so the 3.0 → 0.8 m approach takes 8–12 s. Yaw slowly unless the shot says otherwise.
- Clip ID: `<class>-<family>-<room><day>-<shot>-<take>`, for example `bottle-A-R1d1-F1-1`.
- The app names files by time. Before each battery's takeoff, record a 3 s ground clip of a card showing the session and battery number. The sheet below keeps the clip order and the phone clock time when each clip started.
- `device` in the manifest is `tt_app_flight` or `tt_app_handheld`. `trajectory` is the shot code.

## Shot catalogue

| Code | Mode | Duration | Procedure | Covers |
|---|---|---:|---|---|
| Q0 · Screening pass | Flight | 40 s | Hover centered at 3.0 m for 3 s. Approach along the center line, holding 3 s each at 2.0, 1.5, 1.2 and 1.0 m. Hold 5 s at 0.8 m. Yaw left until the print leaves the frame, return; yaw right until it leaves, return. | A quick pretrained-detector check of each candidate print |
| C1 · Calibration ladder | Handheld on a stand at H, motors off | 90 s | Lens over each line, centered. Hold 5 s at 3.0, 2.5, 2.0, 1.5, 1.2, 1.0, 0.8, 0.65, 0.5 and 0.4 m, about 4 s per move. Note any deviation from this fixed order. | Visual scale `k` for this print and camera (`split=calibration`) |
| F1 · Centered approach | Flight | 30 s | Hover centered at 3.0 m for 3 s. Approach slowly along the center line to 0.8 m. Hold 5 s. | The standard approach and continuous scale change |
| F2 · Offset approach | Flight | 30 s | Start at 2.5–3.0 m on a ±0.5 m lateral mark: left in the first room, right in the second. The print starts near the frame edge. Yaw to center while approaching to 1.0 m. Hold 3 s. | Off-center views, edge crops, yaw correction |
| F3 · Yaw sweep | Flight | 40 s | At 2.0 m, yaw slowly until the print leaves the frame on the left, return, then leave on the right and return (about 15 s). Move to 1.2 m and repeat. | Partial crops, target loss and reacquisition, target-absent frames |
| F4 · Hover hold | Flight | 30 s | Hold centered at 1.0 m for 15 s, then at 0.8 m for 15 s, with no stick input. | Arrival-like view, hover drift and vibration, per-frame dropout |
| F5 · Fast yaw | Flight | 20 s | At 1.5 m, swing yaw quickly through the print three times in each direction. | Motion blur and fast reacquisition |
| N1 · Empty wall | Flight | 30 s | The Q0 movements in front of wall space with no target. | Explicit negatives |
| N2 · Distractor wall | Flight | 30 s | The Q0 movements in front of non-target posters and objects of similar color and shape. | False positives |
| N3 · Same-class pair | Flight | 30 s | Two prints of one class, 0.5 m apart at H. Approach the left one to 1.0 m, back off to 2.0 m, then approach the right one. | Wrong-target selection and `TargetLock` behavior |
| N4 · Occlusion | Handheld at H, motors off | 40 s | At 1.5 m and then 1.0 m, a helper slides a sheet over half the print, then all of it, then away. Three cycles at each distance. | Occlusion and reappearance |

Lighting is a property of the session (see below), not a separate shot.

## Sessions

Session 1 records candidates of up to three classes with three prints each (nine prints). After screening, keep two or three classes. For each kept class, assign family **A** and **B** to training and validation and family **C** to the held-out test.

### Session 0 — bring-up check

About 15 min, one battery.

1. Activate the aircraft and update its firmware in the Tello app. Record the aircraft firmware, app version and expansion-controller mode. The laptop identity queries are in [TT_PLATFORM.md](TT_PLATFORM.md#sensor-and-telemetry-interfaces).
2. With the aircraft on the stand at 2.0 m, record a 10 s clip. Copy it to the laptop. Confirm 1280×720, about 30 fps, a readable duration, and that it opens in OpenCV.
3. Hover at H at 2.0 m for 1 min while recording. The print center must sit near the vertical middle of the frame, within the 0.15 `vertical_gate`. If it does not, move the print, not H.
4. Run the pretrained detector on both clips with `objectpilot.replay`. Proceed when the print is detected at 3.0 m and fully in frame at 0.65 m (handheld).

### Session 1 — screening

Room R1, normal light. Record Q0 for every candidate print: 9 clips × 40 s = 6 min, about 2 batteries. Run the pretrained detector on each clip. Compare, per print: detection rate during each hold, confidence against distance, wrong-class detections and the longest gap. Keep the two or three most reliable classes and choose families A, B and C for each.

### Session 2 — calibration

No flight. Record C1 for every kept print: up to 9 × 90 s = 13.5 min. This session can run while batteries charge. **The print used for the first PID trials needs its C1 first**, because PID trials need its visual scale.

### Session 3 — main set, room R1

Normal light; becomes the training split.

- Families A and B of each class: F1, F2-left, F3, F4 and F5, which is 150 s per print. Six prints give 30 clips and 15 min, about 3–4 batteries.
- Negatives: N1 ×3 and N2 ×3, at different wall spots and camera headings, plus N3 ×1 per class. That is 9 clips × 30 s = 4.5 min, about 1 battery.
- N4 ×2 (handheld): 80 s.

### Session 4 — main set, room R2

A different room, or R1 on another day with side lighting; becomes the validation split. Repeat Session 3 with F2-right: 30 + 9 + 2 clips.

### Session 5 — held-out test

R2 or a third room, ideally on a new day. Family C of each class, full F1–F5 set: 15 clips, 7.5 min, about 2 batteries. These clips are never used for training, threshold choice or model selection.

## Totals and priorities (three kept classes)

| Session | Flight clips | Handheld clips | Footage | Batteries (≤5 min each) |
|---|---:|---:|---:|---:|
| 0 bring-up | 1 | 1 | 1.2 min | 1 |
| 1 screening | 9 | 0 | 6 min | 2 |
| 2 calibration | 0 | 9 | 13.5 min | 0 |
| 3 main R1 (train) | 39 | 2 | 20.8 min | 4–5 |
| 4 main R2 (validation) | 39 | 2 | 20.8 min | 4–5 |
| 5 test | 15 | 0 | 7.5 min | 2 |
| **Total** | **103** | **14** | **about 70 min** | **13–15** |

Each kept class gets 25 positive flight clips: four F-sets from families A and B in two rooms, plus one from family C. The 22 negative and distractor clips (N1–N4) are about 23% of the 97 clips in the training, validation and test splits. This meets the 15–25 clips per class and 20–30% negative targets in the [data guide](DATA_COLLECTION.md). With two kept classes, Sessions 3–5 shrink by roughly a third.

The schedule is set by battery count. With one battery, 13–15 cycles take several days of charging, so record in this order:

1. **By November 8, before PID trials:** Sessions 0–2, then Session 3 for family A of every class. This is enough for the pretrained benchmark and the visual scale of the print used in the trials.
2. **November 8–14:** the rest of Session 3, then Session 4.
3. **November 15–19:** Session 5. Label every test frame and run the held-out benchmark.
4. **Only if the benchmark shows a gap:** label a 1,000–2,000 image training batch from Sessions 3–4 and fine-tune.

## Per-clip log

Fill one row per clip during the session, on paper or in a shared sheet, then copy it into `data/clips.csv`.

```text
clip_id, date, phone_start_time, session, battery_no, battery_pct_start, room, lighting,
print_id, target_class, family, split, shot, mode, height_cm, kit, start_line_m, end_line_m,
app_filename, notes
```

`clip_id`, `print_id`, `room`, `target_class`, `lighting` and `notes` map directly to [data/clips.example.csv](../data/clips.example.csv). `split` is `train` for Session 3, `val` for Session 4, `test` for Session 5 and `calibration` for C1. Set `group_id` to `<room><day>-<family>`, for example `R1d1-A`, so a whole session-and-family group stays in one split. Put `height_cm`, `kit` and any deviation into `notes`. Write down every aborted clip and why.

## After each session

1. Export the MP4 files from the phone to `data/raw/tt-app/<YYYY-MM-DD>/`, which Git ignores. Keep the originals; copy them to their clip IDs.
2. Check the resolution, frame rate and duration. Note visible Wi-Fi artifacts (gray or smeared blocks). Drop a clip only if more than a few seconds are corrupted, and record why.
3. Run the pretrained benchmark on each clip:

   ```bash
   python -m objectpilot.replay --source data/raw/tt-app/2026-11-06/bottle-A-R1d1-F1-1.mp4 --class-name bottle --out runs/bench/bottle-A-R1d1-F1-1.jsonl
   ```

4. Calibration: from each C1 replay log, take the median detected box-height fraction for each hold window. Write `distance_m,box_height_fraction,split` rows with `split=calibration` to `data/calibration/visual_scale.csv`, then run `python -m objectpilot.data calibrate --csv data/calibration/visual_scale.csv`.
5. Extract frames for labeling: `python -m objectpilot.data extract --manifest data/clips.csv --out data/frames/batch01 --fps 2`.

Replay on these clips produces dry-run PID proposals from manually flown footage. They are perception checks of an open-loop recording, not closed-loop controller results.
