# PosePilot

**Spot the object. Fly to it.**

[Project website](https://jonatl5.github.io/posepilot/) · [Weekly progress](https://jonatl5.github.io/posepilot/log.html) · [Task list](TASKS.md)

PosePilot is a DJI RoboMaster TT that recognizes an object, flies up to it and hovers in front of it. A pretrained object detector finds the object in the TT's camera feed; a small reinforcement-learning (PPO) policy turns the object's position and size into steering commands. Everything runs on a laptop connected to the TT over Wi-Fi.

**Status: design stage.** This repository holds the website and the plan. Detection code, a trained policy and flight results are still to come. The website demo is synthetic: no camera, detector or policy runs.

## Scope

- **Targets:** any of a few common object classes (e.g. backpack, bottle, chair) from a pretrained detector.
- **Task:** the object stays still. The drone turns to center it, flies forward and stops at a set distance, facing it.
- **Lighting:** daylight / normal indoor lighting only.
- **Actions:** two outputs, forward speed and turn rate. Altitude is held by a separate loop.

## How it works

```text
RoboMaster TT camera ──Wi-Fi video──> Laptop
                                      ├─ Object detector (e.g. YOLO) → bounding box
                                      ├─ State: box center + size, smoothed
                                      ├─ PPO policy → forward speed, turn rate
                                      └─ Supervisor: limits, min distance, search, land
                                            │ Wi-Fi / Tello SDK rc
                                            ▼
                                      RoboMaster TT
```

| Part | Choice | Job |
| --- | --- | --- |
| Drone | DJI RoboMaster TT (87 g, ~8 min flights) | Flies and takes `rc` velocity commands over Wi-Fi |
| Camera | Built-in, 720p30, 82.6° FOV | Sees the object |
| Detector | Pretrained, e.g. Ultralytics YOLO or MediaPipe Object Detector | Finds the object and its bounding box |
| Policy | Small PPO MLP, trained in simulation | Chooses forward speed and turn rate |
| Laptop | Python | Runs detection, policy and supervisor |

## What the policy sees

| Feature | Meaning |
| --- | --- |
| `ex`, `ey` | How far the box center is from the image center |
| `es` | Box size vs. the size at hover distance (distance proxy) |
| rates | How fast those are changing |
| `c` | Detection confidence |
| `a(t-1)` | Last commands sent |

The goal is reached when the box is centered and at hover size. Box size depends on the object class and viewing angle, so the hover size is calibrated per class.

## Training

1. **Measure** the TT: Wi-Fi delay, detection time and response to `rc` commands.
2. **Simulate** a stationary object and a drone with forward speed + turn, starting from random positions, with measured delay, box noise and missed detections.
3. **Train** PPO with Stable-Baselines3. The reward favors reaching the hover spot quickly and smoothly and penalizes overshoot and getting too close.
4. **Test** against a tuned PID controller on unseen start positions.
5. **Fly** supervised, slow, indoors, at fixed altitude.

## Roadmap

Weeks 1–2 covered planning and research. Progress is logged on the [progress page](https://jonatl5.github.io/posepilot/log.html).

| Weeks | Stage |
| --- | --- |
| 3–4 | Test object detection on the TT camera |
| 5–6 | Build the simulator and PID baseline |
| 7–8 | Train PPO and compare with PID |
| 9–10 | Integrate everything, props off |
| 11–12 | Fly indoors to real objects |

## Research question

Can a learned policy approach an object better than PID? Measured by final distance / centering error, time to arrive, smoothness (no jerks or overshoot) and success rate.

## Safety

The supervisor runs separately from the policy: speed limits, a minimum distance to the object, hold and slow search turn when the object is lost, and land on Wi-Fi loss or operator command. An operator can always take over. The TT's position hold uses its downward camera, which needs a lit, textured floor.

## Earlier designs

- **Person following with day/night cameras.** Replaced by object approach. The IR camera research is kept in [research/ir-cameras.md](research/ir-cameras.md).
- **Holybro X500 V2 + Raspberry Pi 5 + Pixhawk 6C / PX4.** A possible future platform for running everything onboard.

## Website preview

Use Node.js 18 or later. No third-party dependencies are required:

```sh
npm start
```

Open `http://127.0.0.1:4317/posepilot/`. Run `npm run check` to check the demo script's syntax.

Website files live in `docs/`. GitHub Pages publishes `/docs` from `main`, so pushing changes there updates the site. The weekly progress page is `docs/log.html`; fill in each week's fields and set its status pill as described in the comment at the top of that file.

## References

- [DJI RoboMaster TT](https://www.dji.com/support/product/robomaster-tt)
- [RoboMaster developer guide](https://robomaster-dev.readthedocs.io/en/latest/)
- [Ultralytics YOLO](https://docs.ultralytics.com/)
- [MediaPipe Object Detector](https://ai.google.dev/edge/mediapipe/solutions/vision/object_detector)
- [Gymnasium custom environments](https://gymnasium.farama.org/introduction/create_custom_env/)
- [Stable-Baselines3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html)
- [PPO paper](https://arxiv.org/abs/1707.06347)

## License

The original website source and project documentation in this repository are licensed under [MIT](LICENSE). Referenced hardware, software, models and trademarks retain their respective owners' licenses and rights.
