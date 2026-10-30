# RoboMaster TT control and contributor alignment audit

Reviewed October 29, 2026. Hardware: DJI RoboMaster TT (Tello Talent), including its ESP32 expansion. This is a source/interface audit and software validation, not a flight qualification.

## Outcome

Keep the shared goal: recognize stationary wall prints in normal light, center the selected target, approach at fixed altitude and hold a measured stand-off. YOLO supplies perception, PID is the default baseline, and PPO is a simulation-trained alternative evaluated under the same constraints. ArUco/red cards remain bench fixtures. Neither moving-person tracking nor night flight becomes a requirement because another project implements it.

The observation layout remains `objectpilot-v1-16x4`. Behavior is now explicitly versioned as `objectpilot-control-v2`. Existing checkpoints without that behavior version must be retrained and reevaluated; renaming metadata does not migrate a policy.

## Official interface and public projects

Sources were inspected at the revisions linked below. Public examples demonstrate interfaces, not validated gains or stopping distances for our aircraft.

| Reference | Detection / controller input | Output and useful lesson |
|---|---|---|
| [Official TT SDK 3.0](https://dl.djicdn.com/downloads/RoboMaster%20TT/Tello_SDK_3.0_User_Guide_en.pdf), pp. 2, 5, 6, 8 | PC enters SDK mode with `command`, then enables video with `streamon`. Default UDP ports: command 8889, state 8890, video 11111. | `rc a b c d` means roll, pitch, throttle, yaw; integer channels in [-100,100], with no acknowledgement. `EXT tof?` returns millimeters; 8192 denotes out of range. Do not wait for an `ok` after each RC packet or treat RC magnitudes as measured speeds. |
| [AyuDwi1996 TT YOLO project](https://github.com/AyuDwi1996/Yolo-Object-Detection-in-Drone/tree/a6b234077288e06600e6aa3dc4871471b693c9da) (`new_main.py`, `utils.py`) | Tello frames to OpenCV DNN YOLOv3; converts normalized model boxes to pixel rectangles and returns class names. | Detected classes select `EXT mled` graphics. The inspected loop does not implement an autonomous approach controller. Useful evidence for PC perception plus TT expansion commands, not a flight-control baseline. |
| [Tianbot RoboMaster TT ROS](https://github.com/tianbot/rmtt_ros/tree/2f5afd56efff8e93aec7557d46fbc9e8d4a25504) (`rmtt_tracker/scripts/`) | Face tracker uses pixel center and square root of box area. AprilTag tracker uses pose/transform measurements and bearing. | Publishes ROS `geometry_msgs/Twist` on `cmd_vel`. Face outputs are clipped and divided by 100 before publishing; tag controllers use different limits. The driver imports `rmtt_core`, whose conversion is not visible in the inspected Python entrypoint. The repository restricts support to Tianbot-authorized devices. A Twist is not our SDK RC tuple; do not infer identical units or copy axis signs. |
| [dronefreak Tello tracking](https://github.com/dronefreak/dji-tello-target-tracking/blob/ffeb824430dde06b8935cbe8c6c7e3086d270a14/src/drone_controller.py) | Tracked pixel box, pixel center errors and box area relative to a target area; PID control. | Sends four channels through DJITelloPy. It includes lateral and vertical tracking, unlike our fixed-height two-action milestone. Adopt the perception/control separation, not its gains or coupled axis mapping. This is a Tello example, not proof of TT calibration. |
| [DIT Tello RL + YOLO](https://github.com/DIT-Tello-Drone/tello-rl-yolo/tree/f8c7fef32cfce9e696adc5f94c62483dada9fd8c) (`agents/drone_real_env.py`, `main.py`) | YOLOv3 person-box center in a 960 x 720 pixel space; legacy Gym/Keras-RL DDPG with two actions in [-60,60]. | Learned outputs control yaw and vertical motion; separate area thresholds choose forward/backward RC values. It is DDPG, not PPO, and its two actions have different meanings from ours. Its checkpoint and environment cannot be reused as our 64-input forward/yaw policy. |
| [harleylara TT/Tello wrapper](https://github.com/harleylara/tello-python/blob/f37769a13311eaa824057fee455cc4864c866a24/tello/tello.py) and [DJITelloPy API](https://djitellopy.readthedocs.io/en/latest/tello/) | Named joystick/channel arguments, separate from vision output. | Both construct SDK RC commands. The harleylara method's argument order differs from the wire order; use explicit adapters and named semantics. Our boundary remains `(left_right, forward_back, up_down, yaw)`. |

The official [RMTT expansion source](https://github.com/RoboMaster/RMTT_Libs/tree/b8ece60ec31eeca3350209a2559f549864e91dce) is the basis for the existing firmware preparation script. `EXT oprange?` is our custom freshness protocol, not an official SDK command. Firmware routing, sample timing and invalid readings require bench qualification. The aircraft's downward range is not the expansion board's forward range. An infrared distance sensor does not supply a nighttime object-recognition image.

## Exact interface used here

1. Decode a full camera frame to BGR `uint8[H,W,3]`. TT decoding requests PyAV `bgr24`; webcam/file capture uses OpenCV. Keep the source frame ID and host receipt time.
2. YOLO returns class, confidence and `boxes.xywhn`: center x/y and width/height normalized by the **original image dimensions**, not the model's letterboxed 640-pixel tensor. [Ultralytics prediction API](https://docs.ultralytics.com/modes/predict/)
3. Build `Detection(cx, cy, width, height, confidence, timestamp, frame_id, label)`. Confirm three distinct same-class frames. Convert `ex=cx-0.5`, `ey=cy-0.5`; do not multiply errors by two or mix pixels with normalized fractions.
4. Combine selected detection, forward range and previous safe actions in `FeatureBuilder`. Four chronological 16-feature records produce one `float32[64]` vector. Feature order, masks, age scaling and zero padding are listed in [interfaces.md](interfaces.md).
5. PID or PPO proposes `float32[2] = [forward, yaw]` in [-1,1]. `ControlGate` applies validity, centering, proximity, no-reverse, slew and measured arrival rules. Default cadence is 10 Hz; arrival requests zero during a continuous two-second window and latches until reset.
6. Convert the supervised proposal to `(0, round(20*forward), 0, round(25*yaw))` with configurable caps. For example, safe `[0.4,-0.2]` becomes `rc 0 8 0 -5`. A right-side target requests positive yaw in our image convention; confirm the physical sign, image mirroring and stopping response before flight.

PPO receives features, not raw video, YOLO labels or simulator truth. It changes the action proposal only. Detector weights and PPO weights are different artifacts. Replay is an observation test, not an action-responsive training environment. [Stable-Baselines3 PPO supports the continuous Box interface used here](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html).

## Contributor comparison and resolutions

Compared the pipeline/PPO source publication by **jonatl5**, [50d18b8](https://github.com/jonatl5/posepilot/commit/50d18b8446a799b8f5ac984acf1f912b1a31f100), with the integration by **Torres Zeng / TorresZ314**, [f88abed](https://github.com/jonatl5/posepilot/commit/f88abed2e955f1bb5c419380cf37a88fec01554e). Earlier website commits are background; these are the two commits that introduced the active implementation.

| Area | Finding before this audit | Resolution |
|---|---|---|
| Goal and state | Both use wall prints, YOLO, fixed height, forward/yaw, 10 Hz and 16 x 4 state. | Retained; no change to the primary demonstration goal. |
| Target confirmation | Replay confirmed three frames; the simulator bypassed `TargetLock`. | Both now use the same selection implementation before feature construction. Synthetic detection quality still does not establish YOLO accuracy. |
| Arrival / stopping | Replay zeroed during a measured hold and latched arrival; simulation continued controller proposals and used a separate truth-dependent hold counter. | Both now execute `ControlGate` on the same observations. Simulation separately audits true clearance, centering and residual velocity at measured arrival. A failed audit terminates as `arrival_unverified`, never a success. Truth remains outside policy inputs and command decisions. |
| RC response | Simulator applied continuous normalized actions independently of RC cap changes. | Dynamics now receive the same integer RC mapping. Gains are still provisional, referenced to RC=20 forward / RC=25 yaw. Lower caps reduce simulated command response. |
| Configuration | Replay accepted JSON calibration; train/evaluate/demo always created default Config. Evaluation also subtracted a hardcoded 0.65 m. | All four entrypoints accept the same `--config`; metrics use that stand-off. Custom range parsing also uses its configured age limit. |
| Checkpoints | Loading checked schema label/config only. | Verify behavior version, feature order, action names, full config and actual Box shapes/bounds/dtypes. Old behavior checkpoints are rejected even if they contain 64 inputs. |
| Dependencies | The old Windows freeze pinned OpenCV 5, while current extras required OpenCV <5. Vision installation could bring both standard and contrib wheels into the same `cv2` namespace. | Keep the exact old freeze only in `research/team-windows-requirements.txt` as historical provenance. Use `pyproject.toml` extras as the active installation source, with one `opencv-python>=4.10,<5` distribution. Our ArUco APIs are in modern OpenCV's objdetect module. [OpenCV documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) |
| Platform evidence | Team evidence came from Windows; local integration used macOS and CI used Linux. | CI now exercises Python 3.12 on Windows and Linux, checks dependency consistency, requires a single OpenCV distribution, tests nondefault-config PPO train/load/evaluate/demo/replay, and exports resolved package versions with evaluation metadata. |
| Results | Historical metrics refer to older behavior. | Preserve original CSVs and recordings. New diagnostics live in `docs/evidence/control-v2/`; do not pool success rates across behavior versions. |

## Reproduce one configuration end to end

Create a clean Python 3.12 virtual environment, then run from the repository root:

```bash
python -m pip install -e '.[vision,rl,test,hardware,video]'
python -m pip check
python -m pytest -q
python -m objectpilot.train --config configs/wall_approach.json --steps 100000 --seed 0 --out runs/v2-policy
python -m objectpilot.evaluate --config configs/wall_approach.json --model runs/v2-policy/policy.zip --episodes 100 --seed 30000 --out runs/v2-evaluation
python -m objectpilot.demo --config configs/wall_approach.json --model runs/v2-policy/policy.zip --out runs/v2-demo
python -m objectpilot.replay --config configs/wall_approach.json --model runs/v2-policy/policy.zip --source synthetic --detector aruco --seconds 5 --out runs/v2-replay.jsonl
python -m pip freeze > runs/v2-policy/environment-resolved.txt
```

To evaluate PID under matched conditions, omit `--model`. Change calibration in a copied JSON file and use that file in every command; retrain for the new configuration. The checkpoint metadata records the resolved numeric values, not just a filename. Use new output paths to preserve earlier evidence.

## Remaining measured work

The control code is aligned at the software interface. The simulator is still a planar command-response approximation: it does not validate aerodynamic motion, wall effects, video exposure delay or independent sensor cadence. Qualify delivered TT firmware, forward sensor orientation, camera intrinsics/visual scale, RC signs/gain/lag/braking, fresh range transport and operator takeover. Ground-only capture stays available; the command watchdog remains an injected-I/O scaffold, not an autonomous flight launcher. See [hardware gates](HARDWARE_GATES.md).
