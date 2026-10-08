# PosePilot

**Onboard single-person visual pursuit**

[Project website](https://jonatl5.github.io/posepilot/)

PosePilot explores how human pose perception, compact state estimation and reinforcement learning can help a drone follow one person. Perception and policy inference are planned to run entirely on an onboard Raspberry Pi 5, with high-level setpoints sent to PX4 over a wired MAVLink connection.

**Status: system design and project website.** This repository contains the website source and research plan. Flight-control software, a trained PPO policy and measured flight results are pending. The interactive visualization uses synthetic landmarks and runs no camera, MediaPipe model or PPO policy.

## Latest architecture

| Layer | Proposed configuration | Role |
| --- | --- | --- |
| Flight platform | Holybro X500 V2 development kit, Pixhawk 6C configuration | Open research airframe and PX4 flight controller |
| Onboard compute | Raspberry Pi 5, with 4 GB as a starting point | Pose, state estimation, policy inference and supervision |
| Vision input | Standard Camera Module 3 | Direct CSI-2 connection; Pi 5 requires a Standard–Mini 15-pin to 22-pin camera cable |
| Perception | MediaPipe Pose Landmarker, `num_poses = 1` | Extract 33 body landmarks |
| State estimation | EMA / Kalman, to be compared | Center, apparent scale, rates and observation validity |
| Learned policy | Small PPO Actor MLP, to be trained | Forward velocity and yaw rate |
| Flight-control interface | MAVSDK / MAVLink over UART | Send setpoints to PX4 Offboard mode |
| Ground station | Workstation / laptop | Training, debugging, configuration, telemetry and analysis |

```text
Camera Module 3 ──CSI──> Raspberry Pi 5
                         ├─ MediaPipe Pose
                         ├─ State estimator
                         ├─ PPO Actor (inference)
                         └─ Supervisor + altitude loop
                                  │ UART / MAVLink
                                  ▼
                            Pixhawk 6C / PX4
                                  │
                              ESCs / motors

Workstation: simulation + training + analysis
Ground link: configuration / telemetry; outside the visual-control loop
```

## Research scope

- A single visible person in the scene; identity matching across multiple people is outside the initial scope.
- The first policy learns forward velocity and yaw rate, with altitude maintained by a separate loop. A zero vertical-velocity setpoint alone is not a substitute for a verified altitude loop.
- Shoulders and hips provide the torso center and shoulder-width scale. Candidate features include rates, landmark quality, observation age and previous commands.
- Apparent size is a distance proxy affected by body rotation and occlusion. Hip-relative world landmarks do not directly measure camera-to-person range.
- `min_tracking_confidence` is a configuration threshold, not an assumed independent result score. Landmark visibility / presence can inform quality checks.
- `LIVE_STREAM` ignores new input while busy. Buffering and observation age still need explicit handling.
- Training happens on a workstation; the aircraft runs inference. Pi 5 compatibility, frame rate, latency and thermal behavior require bench validation.

## Proposed training pipeline

MediaPipe remains pretrained. The learned component is the small PPO motion policy; training happens on a workstation, while the Pi runs actor inference.

1. **Measure the real pipeline.** Benchmark camera / pose timing, landmark quality and command-response lag. Calibrate camera geometry, choose a desired torso scale, and define the feature order, filtering and normalization.
2. **Build the simulation.** Use a fast Gymnasium environment to project a simulated torso into camera coordinates and perturb those landmarks with measured noise, delays and missed frames. Keep privileged target coordinates out of the actor's observations. Validate the same policy in PX4 SITL / Gazebo with camera frames and the actual pose pipeline; first check that the rendered person can be detected.
3. **Specify episodes and rewards.** Randomize initial bearing, range and target trajectories. Begin with slow, clean pursuit, then add turns, stops, delay, frame loss, body rotation and response lag. For valid observations, a candidate reward is `r = -wx * ex² - ws * es² - wΔ * ||a_t - a_(t-1)||²`. Handle missing observations explicitly, and test separate target-loss and boundary penalties. End on sustained loss or proximity / boundary violations; mark time limits as truncations.
4. **Train PPO.** Collect fresh parallel rollouts, estimate advantages with the critic, then update the actor using PPO's clipped objective. Record seeds, complete configurations, reward components and checkpoints. A small MLP can be trained on a workstation CPU; profile before assuming GPU acceleration is necessary.
5. **Evaluate independently.** Select checkpoints on validation scenarios and freeze them before held-out testing. Compare multiple PPO training seeds with tuned PID under matched observations, limits and supervision. Report tracking error, retention, smoothness and failures rather than training reward alone.
6. **Package and transfer.** Ship actor weights, normalization statistics, feature schema and action scaling together. Verify workstation / Pi action agreement on recorded observations, then progress through SITL, bench tests and supervised fixed-altitude trials. Use deterministic actor inference on the Pi; keep the critic and optimizer off the aircraft.

This is a proposed workflow, not an implemented trainer. Reward coefficients, network size, update interval and randomization ranges remain experimental choices. Record them for every run and freeze final evaluation settings in advance.

## Proposed roadmap and exit gates

All stages in this suggested ten-week sequence remain planned.

1. **Weeks 1–2: perception bench tests.** Validate the camera, ARM64 environment, latency distribution, landmark validity, temperature and power stability.
2. **Weeks 3–4: simulation and baseline.** Build a PX4 SITL / Gazebo pursuit task with matched observations and action limits, then tune a PID baseline.
3. **Weeks 5–6: PPO training.** Randomize visual noise, delay, target motion and missed observations. Evaluate on held-out seeds.
4. **Weeks 7–8: onboard integration.** Deploy inference and test the serial link, command limits, timeouts, manual takeover and PX4 failsafes with propellers removed.
5. **Weeks 9–10: controlled trials.** Start with slow motion at fixed altitude, compare repeated runs, and publish logs, configurations and limitations.

Compare PPO with tuned PID using center / scale error, capture-to-command latency (median and p95), command smoothness, target retention and recovery time. Performance and PPO advantages have not yet been established.

## Supervision and integration

The design includes action limits, observation-age checks, target-loss handling, independent altitude control and RC takeover. Hold requires a valid position estimate. The actual Offboard-loss action must be configured and tested for the operating area. Setpoint heartbeats maintained by MAVSDK do not replace perception-failure monitoring.

Power design starts with a stable 5 V / 5 A rail, a suitable Pi 5 USB-C supply arrangement and active cooling. Mounting, balance, camera calibration, serial electrical compatibility, onboard positioning and voltage transients under motor load each require validation.

## Website preview

Use Node.js 18 or later. No third-party dependencies are required:

```sh
npm start
```

Open `http://127.0.0.1:4317/posepilot/`. Run `npm run check` to check the interactive script's syntax.

Website files live in `docs/`. GitHub Pages publishes the `/docs` directory from the `main` branch. Pushing changes to these files triggers a Pages update.

The English-language website includes responsive layouts, keyboard-accessible controls, motion playback / pause and target-occlusion simulation. It uses no analytics, forms or external scripts.

## Primary references

- [Holybro X500 V2](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2)
- [MediaPipe Pose Landmarker for Python](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker/python)
- [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)
- [Camera Module 3](https://www.raspberrypi.com/products/camera-module-3/)
- [Pi camera cable](https://www.raspberrypi.com/products/camera-cable/)
- [MAVSDK Offboard](https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html)
- [PX4 Offboard and failsafes](https://docs.px4.io/main/en/flight_modes/offboard)
- [Gymnasium custom environments](https://gymnasium.farama.org/introduction/create_custom_env/)
- [Stable-Baselines3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html)
- [PPO paper](https://arxiv.org/abs/1707.06347)
- [PX4 Gazebo simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)

## License

The original website source and project documentation in this repository are licensed under [MIT](LICENSE). Referenced hardware, software, models and trademarks retain their respective owners' licenses and rights.
