# PosePilot task list

Work to do, grouped by area. Tick a box (`- [x]`) when it's done, and note the week on the [progress page](https://jonatl5.github.io/posepilot/log.html).

**Now** = can start before the RoboMaster TT arrives. **After TT** = needs the drone.

## Done

- [x] Decide the initial project plan (week 1)
- [x] Research components; switch to RoboMaster TT + IR camera, laptop compute over Wi-Fi (week 2)
- [x] Update the website and README for the new design

## 1. IR camera — Now

The biggest open decision. Order early so it arrives close to the TT. Comparison: [research/ir-cameras.md](research/ir-cameras.md).

- [x] List candidate cameras: thermal (e.g. FLIR Lepton, MLX90640) and night vision (no-IR-filter camera + 850 nm LEDs)
- [x] Compare weight, power draw, resolution, frame rate and connection (TT 14-pin port: I2C / UART / SPI)
- [ ] Find out how the IR frames get to the laptop (through the TT's ESP32, or a separate link)
- [ ] Run MediaPipe Pose on sample IR / thermal footage from online datasets to see if it finds people
- [ ] Choose a camera and order it
- [ ] Plan how to mount it (weight budget, balance, cable routing)

## 2. Pose pipeline on a webcam — Now

Everything here carries over to the TT unchanged.

- [ ] Set up a Python environment (MediaPipe, OpenCV, NumPy) and pin versions in `requirements.txt`
- [ ] Run MediaPipe Pose Landmarker on a laptop webcam, `num_poses = 1`
- [ ] Compute torso center from shoulders + hips, and shoulder-width scale
- [ ] Compute errors `ex`, `ey`, `es` and their rates; add EMA smoothing
- [ ] Add a latest-frame buffer and an observation-age timeout
- [ ] Log per-frame timing: capture → landmarks → features
- [ ] Test in a dim / dark room and note where detection fails

## 3. Drone control code — Now (write), After TT (test)

- [ ] Pick a library: `djitellopy` or DJI's RoboMaster Python SDK
- [ ] Write connect, video stream, `rc` command and land functions
- [ ] Write the supervisor: speed limits, stale-frame hold, land on Wi-Fi loss, manual override key
- [ ] Add a "dry run" mode that prints commands instead of sending them
- [ ] After TT: test the video stream and measure Wi-Fi delay
- [ ] After TT: measure the response to `rc` commands (props off first, then hover)
- [ ] After TT: measure real flight time, with and without the IR camera

## 4. Simulation + baseline — Now

The longest piece of work on the roadmap.

- [ ] Build a 2D Gymnasium environment: moving person, drone with forward speed + yaw
- [ ] Output the same observation features as the real pipeline (section 2)
- [ ] Add delay, landmark noise and dropped frames
- [ ] Write a PID follower and tune it in the simulator (baseline)
- [ ] First PPO run with Stable-Baselines3 to check the setup works
- [ ] Later: add low-light noise, IR-only observations and camera switching
- [ ] Later: update delay / response numbers with real TT measurements

## 5. Docs + planning — Now

- [ ] Write the observation spec: feature list, order, units, normalization
- [ ] Write the first-flight test plan: location, stop procedure, props-off checks
- [ ] Decide where the code lives (this repo or a separate one)
- [ ] Fill in each week on the progress page

## Later stages (from the roadmap)

- [ ] Weeks 7–8: train PPO with randomized delay, noise and lighting; compare with PID on held-out seeds
- [ ] Weeks 9–10: mount the IR camera, connect the policy to `rc` commands, props-off integration tests
- [ ] Weeks 11–12: slow fixed-altitude indoor trials, good light then dim light; publish logs and results
