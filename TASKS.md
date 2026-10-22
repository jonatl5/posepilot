# PosePilot task list

Work to do, grouped by area. Tick a box (`- [x]`) when it's done, and note the week on the [progress page](https://jonatl5.github.io/posepilot/log.html).

**Goal:** the RoboMaster TT recognizes an object (one of a few classes), flies up to it and hovers in front of it. Objects are stationary; daylight / indoor lighting only.

**Now** = can start before the RoboMaster TT arrives. **After TT** = needs the drone.

## Done

- [x] Decide the initial project plan (week 1)
- [x] Research components; switch to RoboMaster TT, laptop compute over Wi-Fi (week 2)
- [x] Update the website and README for the new design
- [x] Compare IR cameras ([research/ir-cameras.md](research/ir-cameras.md)), then drop night operation and the IR camera
- [x] Switch the goal from following a person to approaching an object

## 1. Object detection on a webcam — Now

Everything here carries over to the TT unchanged.

- [ ] Set up a Python environment (OpenCV, NumPy, a detector) and pin versions in `requirements.txt`
- [ ] Pick a detector: Ultralytics YOLO (nano model, COCO classes) or MediaPipe Object Detector; check the license fits
- [ ] Pick the target classes (e.g. backpack, bottle, chair) and test each on a laptop webcam
- [ ] Compute box center errors `ex`, `ey` and size error `es` (box height vs. hover size); add EMA smoothing
- [ ] Pick the target object when several are visible (highest confidence, or closest to center)
- [ ] Add a latest-frame buffer and an observation-age timeout
- [ ] Log per-frame timing: capture → detection → features
- [ ] Measure box size vs. real distance for each class, to choose the hover size

## 2. Drone control code — Now (write), After TT (test)

- [ ] Pick a library: `djitellopy` or DJI's RoboMaster Python SDK
- [ ] Write connect, video stream, `rc` command and land functions
- [ ] Write the supervisor: speed limits, minimum distance, stale-frame hold, slow search turn when the object is lost, land on Wi-Fi loss, manual override key
- [ ] Add a "dry run" mode that prints commands instead of sending them
- [ ] After TT: test the video stream and measure Wi-Fi delay
- [ ] After TT: run the detector on the TT stream and check accuracy at 720p
- [ ] After TT: measure the response to `rc` commands (props off first, then hover)
- [ ] After TT: measure real flight time

## 3. Simulation + baseline — Now

The longest piece of work on the roadmap.

- [ ] Build a 2D Gymnasium environment: a stationary object, a drone with forward speed + yaw, random start position and heading
- [ ] Output the same observation features as the real pipeline (section 1)
- [ ] Add delay, box noise, missed detections and false detections
- [ ] Reward reaching the hover spot (centered, right size) quickly and smoothly; penalize overshoot and getting too close
- [ ] Write a PID controller and tune it in the simulator (baseline)
- [ ] First PPO run with Stable-Baselines3 to check the setup works
- [ ] Later: update delay / response numbers with real TT measurements

## 4. Docs + planning — Now

- [ ] Write the observation spec: feature list, order, units, normalization
- [ ] Write the first-flight test plan: location, stop procedure, props-off checks
- [ ] Decide where the code lives (this repo or a separate one)
- [ ] Fill in each week on the progress page

## Later stages (from the roadmap)

- [ ] Weeks 7–8: train PPO with randomized start positions, delay and detection noise; compare with PID on held-out seeds
- [ ] Weeks 9–10: connect the policy to `rc` commands, props-off integration tests
- [ ] Weeks 11–12: slow fixed-altitude indoor flights to real objects; publish logs and results
