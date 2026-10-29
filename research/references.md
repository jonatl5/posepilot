# objectPilot sources and reuse decisions

Checked 2026-10-29. Read, locally executed and validated on TT are different evidence levels. No TT validation has occurred. The initial research used website checkout `a2a04e5eb30fd4f2b339ea2e24e229c7f386166b`; subsequent repository updates add the pipeline and offline demo.

| Primary source | Reuse | Limits |
|---|---|---|
| [DJI SDK 3.0](https://dl.djicdn.com/downloads/RoboMaster%20TT/Tello_SDK_3.0_User_Guide_en.pdf) | UDP command/state/video and `EXT tof?` contract | `rc` values are channels, not m/s. EXT support depends on ESP32 firmware. No-radio land cannot be guaranteed by laptop code. |
| [DJI TT manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf) | Expansion-controller architecture and ToF envelope | 1.2 m maximum is specified indoors against a white wall. Needs measurement on prints. |
| [RoboMaster RMTT_Libs](https://github.com/RoboMaster/RMTT_Libs) | ESP32 parser, ToF task and query callback | Pinned b8ece60ec31eeca3350209a2559f549864e91dce; locally inspected and extension generated, not compiled/flashed. Preserve license. |
| [DJITelloPy](https://github.com/damiafuentes/DJITelloPy) and [API](https://djitellopy.readthedocs.io/en/latest/tello/) | Python bench connection and range query | Library installed; live SDK/forward sensor tests await hardware. `get_distance_tof()` is not the expansion forward sensor. |
| [DIT Tello RL YOLO](https://github.com/DIT-Tello-Drone/tello-rl-yolo) | Prior Tello detector plus learned bounding-box controller architecture | DDPG, older stack; not a PPO model or a source of TT-to-wall dynamics. Not executed. |
| [gym pybullet drones](https://github.com/learnsyslab/gym-pybullet-drones) | Optional future physics study | Existing aircraft models do not make a calibrated TT. Fast planar SDK-response model chosen first. Not executed. |
| [Gymnasium custom environments](https://gymnasium.farama.org/introduction/create_custom_env/) | reset/step/seed and terminated/truncated contract | Custom environment validated locally; synthetic sensor generation does not validate real perception. |
| [Stable Baselines3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html) | CPU MLP training, deterministic evaluation and checkpoints | No expectation that PPO must beat PID; smoke training is not convergence evidence. |
| [YOLO11](https://docs.ultralytics.com/models/yolo11/) and [training](https://docs.ultralytics.com/modes/train/) | A small supported pretrained detector, selected for reproducibility | Not a claim of being the newest or best model. Benchmark prints before fine-tuning; upstream software/weight terms apply separately. |
| [COCO](https://cocodataset.org/) and [API](https://github.com/cocodataset/cocoapi) | Pretrained common-object categories and evaluation conventions | Indoor printed-target domain still needs own data. Check individual image rights before redistributing prints/datasets. |
| [VisDrone](https://github.com/VisDrone/VisDrone-Dataset) | Drone-domain reference data | Aerial vehicles/people differ from the present task; not the primary training set. |
| [ST VL53L1X datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf) and [Pololu driver](https://github.com/pololu/vl53l1x-arduino) | Optional external infrared laser ToF backend | Up to 4 m is not a universal accuracy guarantee. Confirm voltage, bus address, FoV, timing and mounting. |

Implementation decisions are project choices, not vendor guarantees. Hardware qualification, camera calibration, actual video collection, trained detector validation, PID tuning and multi-seed PPO evaluation remain necessary.
