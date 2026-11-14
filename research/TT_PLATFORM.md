# RoboMaster TT platform: control paths and sensor interfaces

Checked November 5, 2026, after the aircraft was delivered. This page summarizes published DJI/Ryze documentation and a few community reports. It is source reading, not a measurement of our unit. Record the delivered unit's firmware and behavior in [Session 0 of the recording plan](RECORDING_PLAN.md#session-0--bring-up-check) and in [hardware gates](HARDWARE_GATES.md) 1–4.

## Hardware summary

| Item | Published value | Consequence for objectPilot |
|---|---|---|
| Aircraft | 87 g with propellers, guards and battery; removable 1.1 Ah / 3.8 V battery. [Manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf) | Small, light aircraft. Wall effects and drafts can matter near the stand-off. |
| Flight time | About 8 min with the expansion kit, 13 min without it. Measured by DJI in still air at a constant 15 km/h. [Manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf), [DJI support](https://www.dji.com/support/product/robomaster-tt) | Plan at most 5 min of recording per battery with the kit mounted. Hovering and indoor maneuvering differ from DJI's test. |
| Camera | 5 MP stills (2592×1936). Video and recording: 1280×720 at 30p, MP4. FOV 82.6°; the document does not say which axis. [Manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf) | Detector data and visual-scale calibration must come from this camera. Do not reuse phone-camera scale values. |
| Vision Positioning System (VPS) | Downward camera plus infrared height sensing. Effective at 0.3–10 m, best at 0.3–6 m. The aircraft drops to Attitude mode (no position hold) in several conditions: below 300 lux; over monochrome, textureless, repeating, reflective or moving surfaces; when lighting changes; and at high speed below 0.5 m. [Manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf) | The floor under the approach path needs light and texture. In Attitude mode the aircraft drifts; land as soon as possible. |
| Failsafe | Tello app: auto-landing after 50 s of weak or lost mobile signal, or if the app crashes. SDK mode: auto-landing if no command arrives within 15 s. [Manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf), [SDK 3.0](https://dl.djicdn.com/downloads/RoboMaster%20TT/Tello_SDK_3.0_User_Guide_en.pdf) | Neither limit is a fast link-loss stop near a wall. Operator takeover remains required (gate 5). |
| Open-source controller | ESP32-D2WD, dual-core, 160 MHz. Wi-Fi 2.4/5.8 GHz and Bluetooth. A switch selects direct-connection (AP) or router (station) mode. In direct mode the SSID changes to `RMTT-XXXXXX`. In router mode the Tello app cannot connect. Micro-USB port for firmware and offline debugging. [Manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf) | Keep the switch in direct mode for every session described here. |
| 14-pin expansion port | I2C, UART, SPI, GPIO, PWM, 5 V (4.8 ± 0.2 V) and 3.3 V. The listed pins are IO2, IO4, IO5, IO12–IO15, IO21, IO26 and IO27. [Manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf) | This is where an optional external sensor would attach; see [firmware/README.md](../firmware/README.md). |
| Dot-matrix and distance module | 8×8 red/blue matrix plus a forward infrared ToF sensor. Maximum 1.2 m indoors against a white target. The kit adds 12.5 g. [Manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf) | This is the only forward range sensor. 1.2 m matches `forward_range_max_m`; usable range on our prints must be measured. |
| Box contents (single pack) | One battery, four mission pads, propeller guards, spare propellers, expansion kit. [Quick start](https://www.manualslib.com/manual/2239196/Ryze-Robomaster-Tt.html) | With one battery, battery cycles limit flight recording. See the budget in the recording plan. |

## Ways to control the TT

| Path | Where the code runs | What it can do | Use in this project |
|---|---|---|---|
| Tello app (phone) | Phone | Activation, manual flight with virtual sticks, photo/video recording to the phone, firmware updates. DJI's download center lists the app as no longer updated. [Manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf), [downloads](https://www.dji.com/downloads/products/robomaster-tt) | **Detector-data flights**, per the [recording plan](RECORDING_PLAN.md). |
| Tello EDU app | Phone | Block programming; controls the LEDs with factory firmware. | Not used. |
| Text SDK 3.0 over UDP | Laptop | Send `command` to 192.168.10.1:8889. State arrives on port 8890 and H.264 video on port 11111 after `streamon`. `rc a b c d` sets roll, pitch, throttle and yaw in −100…100 and gets no reply. `EXT …` is forwarded to the ESP32 (factory firmware only). [SDK 3.0](https://dl.djicdn.com/downloads/RoboMaster%20TT/Tello_SDK_3.0_User_Guide_en.pdf) | **Primary control path.** Already used by `GroundLink`, `objectpilot.bench` and the replay tools, and the target for the future flight sink. |
| DJITelloPy | Laptop | Python wrapper around the text SDK. One forum report says it does not interpret the `EXT tof?` reply. [Forum](https://tellopilots.com/threads/rm-tello-tt-collision-avoidance.6370/), [SDK 3.0 issue](https://github.com/damiafuentes/DJITelloPy/issues/92) | Used for the bench connection. objectPilot parses the range reply itself. |
| RoboMaster Python SDK (`robomaster`) | Laptop | Official wrapper for EP and TT. The official Windows setup requires 64-bit Python 3.6.6–3.8.9. [Environment setup](https://robomaster-dev.readthedocs.io/en/latest/code_env_setup.html) | **Not adopted.** It conflicts with the project's Python 3.10–3.13 requirement, and the text SDK covers the same commands. |
| Mind+ | Laptop | Graphical programming. Real-time mode needs the factory ESP32 firmware; upload mode runs on the controller. [Mind+ wiki](https://mindplus.dfrobot.com/RMTT) | Not used. |
| Arduino / MicroPython on the ESP32 | Expansion controller | Offline programs. The controller can send `[TELLO] <command>` to the aircraft over serial; replies carry an `ETT ` prefix. RM Assistant (DJI Education Hub) restores factory firmware. [SDK 3.0](https://dl.djicdn.com/downloads/RoboMaster%20TT/Tello_SDK_3.0_User_Guide_en.pdf), [manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf) | Only for the optional `EXT oprange?` freshness extension in `firmware/`. Flashing replaces the factory firmware, which the stock `EXT` commands, Mind+ real-time mode and the Bluetooth controller rely on. |
| ROS (`tianbot/rmtt_ros`) | Laptop | ROS driver. Its repository limits support to Tianbot-authorized devices. | Reference only; see the [TT control audit](TT_CONTROL_AUDIT.md). |

Project operating rule, not a vendor statement: use one controlling client at a time. Close the Tello app before any laptop SDK session, and do not run laptop tools while the app is flying the aircraft.

## Sensor and telemetry interfaces

| Signal | How to read it | Units and range | Notes |
|---|---|---|---|
| Forward ToF (expansion module) | `EXT tof?` → `tof xxxx` | mm; the guide says 8192 when out of range, but **our unit replies 8190** (Nov 7 bench, parser now rejects ≥ 8190); maximum 1.2 m against a white target indoors | Sample age is unknown with stock firmware. The stock example keeps the previous value when a read fails, and its 200 ms timing budget limits fresh samples to about 5 per second ([firmware/README.md](../firmware/README.md)). One forum user needed aircraft firmware v02.05.01.19 before this command answered ([forum](https://tellopilots.com/threads/rm-tello-tt-collision-avoidance.6370/)). |
| Downward ToF | State field `tof` | cm | Height above the floor. **Never forward clearance.** |
| Height | State `h` (relative to takeoff), `baro` | cm; m | Use to hold and verify the recording height H. |
| Attitude | State `pitch`, `roll`, `yaw` | degrees | Yaw helps when measuring yaw response. |
| Velocity | State `vgx`, `vgy`, `vgz` | dm/s | The aircraft's own estimate. Compare it with floor ground truth before trusting it for braking. |
| Acceleration | State `agx`, `agy`, `agz` | the guide says cm/s² | Units should be checked on the unit. |
| Battery, temperatures, motor time | State `bat`, `templ`/`temph`, `time`; `battery?` | %, °C, s | Log the battery level at the start and end of every session. |
| Mission pads | `mon`, `moff`, `mdirection 0/1/2`; state `mid`, `x`, `y`, `z`, `mpry` | cm relative to the pad. 20 Hz with one camera direction, 10 Hz each when both are enabled. Recognizable 0.3–1.2 m above the pad, over an area from 0.4×0.4 m to 1×1 m. [Mission pad guide](https://dl.djicdn.com/downloads/RoboMaster+TT/RoboMaster_TT_Mission_Pad_and_Flight_Map_User_Guide_en.pdf) | Not detected: `mid` −1 and x/y/z −100. Detection disabled: −2 and −200. A pad on the floor near the wall can serve as independent position ground truth for response and braking tests (gate 6). |
| Video stream settings | `setresolution high/low`, `setfps high/middle/low`, `setbitrate 0–5` | 720p/480p; 30/15/5 fps; auto or 1–5 Mbps | SDK 3.0 commands. One forum user found the TT/EDU needed aircraft firmware v02.05.01.17 or newer for SDK 3.0 ([forum](https://tellopilots.com/threads/sdk-3-0-for-tello-edu.6013/)). |
| Identity | `sdk?`, `sn?`, `hardware?` (`RMTT` or `TELLO`), `wifi?` (SNR), `wifiversion?`, `ssid?`, `EXT version?` | text | Record these once for the delivered unit. `GroundLink` currently allows only `command`, `streamon`, `streamoff` and the EXT range and version queries. |
| LEDs | `EXT led r g b`, `EXT mled …` | 0–255 per channel | Optional operator status display, for example a visible autonomous-mode indicator. |

The state string's update rate is not given in the guide; measure it on the delivered unit.

## Decisions for objectPilot

1. Keep the laptop text SDK over UDP as the only programmatic control path, from the existing Python 3.12 environment. No new dependency is needed.
2. Detector footage comes from the TT camera, recorded by the Tello app during manual flights ([recording plan](RECORDING_PLAN.md)). The app does not log ToF, telemetry or commands. Those clips are perception data only. Latency, range and response evidence come from separate laptop SDK sessions.
3. Recording and flight height: lens at 1.60 m, target center about 1.47 m. The camera tilts down about 10.5°; see [TT_MEASUREMENTS.md](TT_MEASUREMENTS.md). This replaces the first choice of 1.0 m. It stays inside the VPS's best range, but above the 1.2 m mission-pad recognition limit, so pad ground truth for response tests needs a separate lower flight.
4. The forward ToF stops at 1.2 m, the configured `forward_range_max_m`. The [near-entry finding](CONTROLLER_TUNING.md#three-simulator-findings-that-gains-cannot-fix) can only be decided after measuring that range on the actual prints (gate 3).
5. Any future flight sink must send a command at least every 15 s, or the aircraft lands. The 10 Hz loop easily satisfies this, but a stalled loop also lands the aircraft. Treat that as a backstop, not as the command-age watchdog.
