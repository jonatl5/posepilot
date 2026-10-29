# Data collection and training plan

## What can be done before delivery

Start with pretrained YOLO11n and a small set of matte wall prints. Choose two or three COCO classes from observed detection reliability, for example bottle, backpack or cup. Try three to five source pictures per class. Print size, image content, cropping and camera field of view affect the detected box; the paper dimensions are not the object-box dimensions.

Record 20–40 second phone or webcam clips at the intended camera height. Cover centered approaches, yaw, lateral offsets, pauses, motion blur, partial crops, occlusion/reappearance, distractor posters, two objects of the same class and empty walls. Vary rooms, dates and lighting. A starting collection of 15–25 clips per class and 20–30% negative/distractor clips is a planning estimate, not a guarantee of sufficient data.

Use a tape measure for stationary reference points at 0.4, 0.5, 0.65, 0.8, 1.0, 1.2, 1.5, 2.0, 2.5 and 3.0 m. Near points are handheld calibration measurements, not permitted flight distances. Record whether distance is measured from the camera, sensor or body, plus the mounting offsets. Hold at each point for two or three seconds. A moving clip must not assign a single constant ground-truth distance to every frame.

Capture landscape video with fixed resolution and no digital zoom. Record camera model, lens, stabilization, exposure/HDR settings and print ID. Do not treat a manufacturer's diagonal field of view as horizontal field of view. Recalibrate after changing the camera, resolution, crop, lens or print dimensions.

## Files and labels

Use `data/clips.example.csv` as the clip manifest. Share a `group_id` across related recording sessions, rooms and print families. Split groups, not adjacent frames. The default hash split approximates 70/15/15 over many groups; with few groups, assign splits manually so each has enough classes and examples. Hold out new print families when claiming generalization to unseen images.

```bash
python -m objectpilot.data extract --manifest data/clips.csv --out data/frames/batch01 --fps 2
# Annotate object boxes, review every label and fill label_path / label_reviewed=yes.
python -m objectpilot.data prepare --frames data/frames/batch01/frames.csv --out data/dataset/v1 --classes bottle,backpack,cup
```

Label the depicted object, not the entire paper, unless a separate paper class is explicitly intended. An empty label file means a manually verified negative image. A missing file is not a negative. Preparation rejects unreviewed/missing labels, invalid or out-of-image boxes, duplicate image bytes and cross-split groups. Group registration and manual review must still catch visually similar frames and scene leakage.

Direct webcam or grounded TT collection also produces the same review manifest:

```bash
python -m objectpilot.collect --source 0 --seconds 20 --fps 2 --out data/frames/session01 --group-id roomA-session01-printFamilyA --print-id bottle01 --split train
python -m objectpilot.collect --source tello --seconds 20 --out data/frames/tt01 --group-id roomA-tt01-printFamilyB --print-id bottle02 --split test
```

Neither command starts flight or sends RC. No real collection was performed during integration. Local images, logs and models are ignored by Git; review any dataset before sharing it.

## Decide whether to train

Benchmark pretrained weights first. If failures mainly come from the printed-image domain, fine-tune on reviewed training data. A first annotation batch of 1,000–2,000 diverse images can be a useful budget; add data based on failure cases and learning curves. Do not train from scratch for a red card or ArUco marker.

```bash
python -m pip install -e '.[vision]'
python -m objectpilot.data train --data data/dataset/v1/dataset.yaml --weights yolo11n.pt --epochs 50 --device cpu
python -m objectpilot.data evaluate --data data/dataset/v1/dataset.yaml --weights runs/detector/printed_targets/weights/best.pt --split val
# Freeze model and threshold before the final test evaluation.
python -m objectpilot.data evaluate --data data/dataset/v1/dataset.yaml --weights runs/detector/printed_targets/weights/best.pt --split test
```

Report per-class precision/recall and mAP, recall by distance/lighting, longest dropout, wrong-target selections and median/p95 inference timing. Do not select a model on the final test set. Public aerial datasets are not direct substitutes for indoor, eye-level printed-wall footage. No annotated target dataset or newly trained detector is supplied here.

## Two separate calibrations

For the operational visual proxy, fit `k = median(distance_m * box_height_fraction)` on a separate calibration set for each print/camera setup. CSV columns are `distance_m,box_height_fraction,split`; every row must have `split=calibration`.

```bash
python -m objectpilot.data calibrate --csv data/calibration/visual_scale.csv
# Put the returned value in a copied Config or pass --visual-scale to replay.
python -m objectpilot.replay --source data/raw/approach.mp4 --visual-scale 0.22 --out runs/calibrated-replay.jsonl
```

The value 0.22 is an uncalibrated example. Range sidecars use video seconds, meters, source/status and optional sequence; unknown sample age cannot be relabeled as verified fresh range.

Camera intrinsics are a separate geometric check:

```bash
python -m objectpilot.generate_marker --kind checkerboard --output assets/checkerboard.png
python -m objectpilot.calibrate_camera --images 'data/calibration/*.jpg' --square-m 0.025 --output configs/camera-local.json
```

Use the measured printed square size and at least ten sharp, varied views. Review per-view errors and verify held-out geometry. The current wall controller uses the per-print visual scale; it does not silently consume the intrinsic calibration or claim metric ArUco pose accuracy.

## PPO is not trained from prerecorded video

Video tests perception and helps estimate sensor noise/delay. PPO requires the interactive `ApproachEnv`, whose next state depends on its actions. Train nominal and randomized candidates with multiple seeds, select checkpoints on validation scenarios, then freeze them. Use matched detector settings, action limits and supervisor when comparing with tuned PID. The already-published seeds 10000–10029 are development diagnostics. Reserve new final scenarios and report failures/interventions as well as successes.

After delivery, measure the actual TT image domain, video delay, sensor cadence, command response, stopping drift and battery effects. Update simulation assumptions before making any flight-performance claim. Use the most reliable qualified controller for the demonstration, even if that remains PID.
