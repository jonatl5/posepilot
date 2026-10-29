# ESP32 range firmware preparation

The TT expansion controller is programmable; the aircraft flight controller is not replaced. Laptop detection and PPO remain on the ground computer. The factory `EXT tof?` path should be measured first. The expansion-kit maximum of 1.2 m is specified for an indoor white wall, not guaranteed for every printed surface.

## Built-in sensor integration

Reviewed upstream: [RoboMaster/RMTT_Libs](https://github.com/RoboMaster/RMTT_Libs), commit `b8ece60ec31eeca3350209a2559f549864e91dce`. The stock default example uses `Wire.begin(27, 26)` and `Serial1.begin(1000000, SERIAL_8N1, 23, 18)`. These are board-specific facts from that sketch, not generic ESP32 pin defaults. Its 200 ms sensor timing budget means polling at 10 Hz does not guarantee 10 fresh readings/sec.

The example retains the previous `tof_range` when a read returns 65535. A fresh network response can therefore contain old sensor data. `range_snapshot.h` records every read, including invalid ones, and exposes sequence, sample time, validity and age. The patch preserves upstream command parsing and flight bridging.

```powershell
git clone https://github.com/RoboMaster/RMTT_Libs.git artifacts/references/RMTT_Libs
git -C artifacts/references/RMTT_Libs checkout b8ece60ec31eeca3350209a2559f549864e91dce
python firmware/prepare_firmware.py --upstream artifacts/references/RMTT_Libs --out artifacts/firmware/objectpilot_default
```

The generated sketch retains upstream license obligations. Install DJI's TT board package and RMTT_Libs per the official instructions, compile for the actual board, and retain a factory restore method before flashing. No firmware has been flashed or compiled for a TT in this task. The local environment does not have the TT Arduino board toolchain.

The **application-defined** query `EXT oprange?` is intended to route to the new callback. It is not an official SDK command and must be verified end-to-end on hardware. Reply contract:

```text
oprange <sequence> <sampled_ms> <millimeters> <valid_0_or_1> <sample_age_ms>
```

Host computes a conservative sample timestamp as `request_start - sample_age_ms/1000`; it must reject invalid/status errors, repeated sequence, counter resets and excessive request RTT. MCU milliseconds and host monotonic seconds are not directly comparable. Sequence rollover must be accounted for in a persistent flight adapter. A sensor timeout must never be relabeled as a fresh valid range.

## Optional external infrared ToF

`external_vl53l1x.h` supplies a nonblocking Pololu VL53L1X adapter as a bench integration starting point. It is not enabled in the generated default sketch. For the printed-wall MVP, measure the built-in sensor before selecting an upgrade. VL53L1X is an infrared laser ToF sensor, not radio radar. ST lists up to 4 m and 27-degree typical FoV, but range depends on timing, reflectance and lighting; Short mode in this adapter favors near-range testing.

1. Check the exact breakout voltage and logic-level requirements. The bare chip and a regulated breakout have different supply requirements; never infer 5 V tolerance.
2. Both built-in and many external ToF sensors use address 0x29. Disconnect/disable the built-in sensor, or use verified XSHUT wiring and readdress on every boot. Do not invent a free GPIO. Confirm address ownership with an I2C scan.
3. Replace the stock `tt_tof.Init` and range-read path with one external backend. Keep a single I2C owner or a bus mutex when using the matrix. Do not call both sensor readers against 0x29.
4. Feed each completed reading into `op_record_range(valid ? mm : 65535)`. Keep the current 1.2 m software gate until the new operating envelope has been measured; increasing hardware range alone does not update the policy.
5. Measure payload, mounting, optical axis and vibration. Do not extrapolate a single beam into full obstacle avoidance.

Bench grid: 0.25, 0.4, 0.5, 0.65, 0.8, 1.0, 1.2 and 1.5 m; matte white, dark print, glossy print; head-on and oblique views; sensor unplug, blocked beam, repeated samples, MCU restart. Log raw response, sequence, age, tape distance, validity and query RTT. Report MAE, p95 absolute error, invalid fraction and fresh-sample rate per condition. Accept or revise the 0.65 m stand-off and 0.40 m minimum only after clearance/braking tests.

Sources: [DJI SDK 3.0](https://dl.djicdn.com/downloads/RoboMaster%20TT/Tello_SDK_3.0_User_Guide_en.pdf), [TT manual](https://dl.djicdn.com/downloads/RoboMaster%20TT/RoboMaster_TT_Tello_Talent_User_Manual_en.pdf), [Pololu driver](https://github.com/pololu/vl53l1x-arduino), [ST datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf).
