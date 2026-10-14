# PosePilot

**Day-and-night single-person visual pursuit**

[Project website](https://jonatl5.github.io/posepilot/) · [Weekly progress](https://jonatl5.github.io/posepilot/log.html)

PosePilot explores how human pose perception, compact state estimation and reinforcement learning can help a small drone follow one person, in good light and in low light. The platform is a DJI RoboMaster TT: its built-in camera covers good light, and an added infrared camera keeps the person visible as light drops. The TT streams video over Wi-Fi to a ground laptop, which runs perception, the policy and the supervisor and sends velocity commands back through the Tello SDK.

**Status: system design and project website.** This repository contains the website source and research plan. Flight-control software, a trained PPO policy, the IR camera choice and measured flight results are pending. The interactive visualization uses synthetic landmarks and runs no camera, MediaPipe model or PPO policy.

## Latest architecture

| Layer | Proposed configuration | Role |
| --- | --- | --- |
| Flight platform | DJI RoboMaster TT (87 g), Tello SDK 3.0 | Flight stabilization and downward vision positioning; accepts `rc` velocity commands over Wi-Fi |
| Visible camera | Built-in camera, 720p30, 82.6° FOV | Main input in good light, streamed to the laptop |
| Infrared camera | Model to be chosen, on the TT's 14-pin expansion port (I2C, UART, SPI, GPIO, PWM) | Low-light and night input |
| Compute | Ground laptop, Python | Pose, camera fusion, state estimation, policy inference and supervision |
| Perception | MediaPipe Pose Landmarker, `num_poses = 1` | Extract 33 body landmarks |
| State estimation | EMA / Kalman, to be compared | Center, apparent scale, rates, observation validity and source camera |
| Learned policy | Small PPO Actor MLP, to be trained | Forward velocity and yaw rate |
| Control interface | Tello SDK `rc` commands over Wi-Fi | Send bounded velocity commands to the TT |

```text
RoboMaster TT
├─ Built-in camera ─┐
└─ IR camera ───────┤ Wi-Fi video
                    ▼
              Ground laptop
              ├─ MediaPipe Pose
              ├─ Camera fusion + state estimator
              ├─ PPO Actor (inference)
              └─ Supervisor + altitude loop
                    │ Wi-Fi / Tello SDK rc
                    ▼
        RoboMaster TT flight controller
                    │
                  motors

Laptop / workstation: simulation + training + analysis
Wi-Fi link: inside the visual-control loop; delay and dropouts must be measured
```

### IR camera options

The IR camera is not chosen yet. The two main options trade off differently:

- **Thermal (LWIR)** works in full darkness but has low resolution, and MediaPipe Pose is not trained on thermal images.
- **Night vision with IR LEDs** produces images closer to normal grayscale video, so MediaPipe is more likely to work, but range is short and the LEDs add weight and draw power.

Either way, weight, power draw and data rate must fit the TT, and MediaPipe's accuracy on the chosen IR images must be tested.

## Research scope

- A single visible person in the scene; identity matching across multiple people is outside the initial scope.
- The first policy learns forward velocity and yaw rate, with altitude maintained by a separate loop.
- Shoulders and hips provide the torso center and shoulder-width scale. Candidate features include rates, landmark quality, observation age, the source camera and previous commands.
- Whether to switch between the visible and IR cameras or fuse both is an open question.
- Apparent size is a distance proxy affected by body rotation and occlusion. Hip-relative world landmarks do not directly measure camera-to-person range.
- `min_tracking_confidence` is a configuration threshold, not an assumed independent result score. Landmark visibility / presence can inform quality checks.
- Video arrives over Wi-Fi and `LIVE_STREAM` ignores new input while busy. Buffering and observation age still need explicit handling.
- DJI rates the TT's flight time at about 8 minutes, so test sessions are short. Flight time with the IR camera mounted must be re-measured.

## Proposed training pipeline

MediaPipe remains pretrained. The learned component is the small PPO motion policy; training happens on a workstation, and the ground laptop runs actor inference.

1. **Measure the real pipeline.** Benchmark frame timing, Wi-Fi stream delay, pose inference time, landmark quality and the TT's response lag to `rc` commands, in good light and low light, with both cameras. Calibrate and align both cameras, choose a desired torso scale, and define the feature order, filtering and normalization.
2. **Build the simulation.** Use a fast Gymnasium environment to project a simulated torso into camera coordinates and perturb those landmarks with measured noise, delays and missed frames. Keep privileged target coordinates out of the actor's observations. The TT has no official simulator, so the drone model must match its measured response to `rc` commands. Validate the policy in a higher-fidelity simulator such as Gazebo with rendered frames and the actual pose pipeline; first check that the rendered person can be detected.
3. **Specify episodes and rewards.** Randomize initial bearing, range and target trajectories. Begin with slow, clean pursuit, then add turns, stops, delay, frame loss, body rotation and response lag, and finally low-light noise, IR-only observations and camera switches. For valid observations, a candidate reward is `r = -wx * ex² - ws * es² - wΔ * ||a_t - a_(t-1)||²`. Handle missing observations explicitly, and test separate target-loss and boundary penalties. End on sustained loss or proximity / boundary violations; mark time limits as truncations.
4. **Train PPO.** Collect fresh parallel rollouts, estimate advantages with the critic, then update the actor using PPO's clipped objective. Record seeds, complete configurations, reward components and checkpoints. A small MLP can be trained on a workstation CPU; profile before assuming GPU acceleration is necessary.
5. **Evaluate independently.** Select checkpoints on validation scenarios and freeze them before held-out testing. Compare multiple PPO training seeds with tuned PID under matched observations, limits and supervision. Report tracking error in good and low light, retention, smoothness and failures rather than training reward alone.
6. **Package and transfer.** Ship actor weights, normalization statistics, feature schema and action scaling together. Verify that the deployed actor reproduces the training commands on recorded observations, then progress through simulation, props-off bench tests and supervised fixed-altitude trials. Run the actor deterministically on the ground laptop.

This is a proposed workflow, not an implemented trainer. Reward coefficients, network size, update interval and randomization ranges remain experimental choices. Record them for every run and freeze final evaluation settings in advance.

## Proposed roadmap and exit gates

All stages in this suggested ten-week sequence remain planned. Week-by-week progress is recorded on the [progress page](https://jonatl5.github.io/posepilot/log.html).

1. **Weeks 1–2: perception bench tests and IR camera choice.** Bring up the TT video stream and Pose on the laptop. Measure frame age, Wi-Fi delay, inference time, landmark validity and dropped frames in good and poor light, and bench-test IR camera options.
2. **Weeks 3–4: simulation and baseline.** Build a pursuit simulation matched to the TT's measured `rc` response, observations and action limits, then tune a PID baseline.
3. **Weeks 5–6: PPO training.** Randomize visual noise, delay, lighting, target motion and missed observations. Evaluate on held-out seeds.
4. **Weeks 7–8: full-system integration.** Mount the IR camera, connect the policy to `rc` commands and test timestamps, command limits, stale observations, Wi-Fi loss and manual takeover with propellers removed.
5. **Weeks 9–10: controlled trials.** Start with slow motion at fixed altitude in a clear indoor area, first in good light and then in dim light. Compare repeated runs and publish logs, configurations and limitations.

Compare PPO with tuned PID using center / scale error, capture-to-command latency (median and p95), command smoothness, target retention and recovery time, in both good and low light. Performance and PPO advantages have not yet been established.

## Supervision and safety

The design includes action limits, observation-age checks, target-loss handling and an independent altitude loop. On stale or missing observations the supervisor holds or hands over; on Wi-Fi loss or an operator command the TT lands. Operator takeover and the TT's own link-loss behavior stay independent of the learned policy and must be tested on the aircraft.

Hovering in place relies on the TT's downward vision positioning, which needs a lit, textured floor. Night and low-light flights therefore need their own hold and landing tests. Mounting the IR camera changes mass and balance, so re-check stability and flight time after assembly.

## Future platform

The earlier design, a Holybro X500 V2 with a Raspberry Pi 5 companion computer, Camera Module 3 and Pixhawk 6C running PX4, remains a possible scale-up for running perception and the policy fully onboard.

## Website preview

Use Node.js 18 or later. No third-party dependencies are required:

```sh
npm start
```

Open `http://127.0.0.1:4317/posepilot/`. Run `npm run check` to check the interactive script's syntax.

Website files live in `docs/`. GitHub Pages publishes the `/docs` directory from the `main` branch. Pushing changes to these files triggers a Pages update. The weekly progress page is `docs/log.html`; fill in each week's fields and set its status pill as described in the comment at the top of that file.

The English-language website includes responsive layouts, keyboard-accessible controls, motion playback / pause and target-occlusion simulation. It uses no analytics, forms or external scripts.

## Primary references

- [DJI RoboMaster TT](https://www.dji.com/support/product/robomaster-tt)
- [RoboMaster developer guide](https://robomaster-dev.readthedocs.io/en/latest/)
- [RoboMaster TT expansion kit](https://store.dji.com/product/robomaster-tt-expansion-kit)
- [MediaPipe Pose Landmarker for Python](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker/python)
- [Gymnasium custom environments](https://gymnasium.farama.org/introduction/create_custom_env/)
- [Stable-Baselines3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html)
- [PPO paper](https://arxiv.org/abs/1707.06347)
- [Gazebo](https://gazebosim.org/docs)
- [Holybro X500 V2](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2) (future platform)

## License

The original website source and project documentation in this repository are licensed under [MIT](LICENSE). Referenced hardware, software, models and trademarks retain their respective owners' licenses and rights.
