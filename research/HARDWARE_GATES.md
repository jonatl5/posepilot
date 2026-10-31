# Hardware bring-up and first-flight gates

The aircraft was delivered by November 5, 2026. Until the gates below are recorded, no real flight, SDK stream, sensor freshness or board compilation is established. The 0.65 m stand-off and 0.40 m minimum are simulation settings until measured braking, body offsets and sensing error justify physical clearances. Published interfaces, units and limits are summarized in [TT_PLATFORM.md](TT_PLATFORM.md).

Two kinds of session use the aircraft and never overlap:

- **Tello app recording sessions** fly manually and record detector footage on the phone ([RECORDING_PLAN.md](RECORDING_PLAN.md)). They log no range, telemetry or commands, so they supply perception data for PRD gate G1 only and cannot pass any gate below.
- **Laptop SDK sessions** use `objectpilot.replay`, `objectpilot.bench` and, later, the qualified flight sink. Close the app first and keep the expansion-controller switch in direct-connection mode.

1. **Identify the platform.** Confirm the delivered TT model, aircraft firmware (Tello app), `sdk?`, `sn?`, `hardware?` (expects `RMTT`), `EXT version?`, expansion board, forward-sensor direction, voltage, payload and restoring procedure (RM Assistant restores factory controller firmware). Follow the official documentation linked in `references.md`, `TT_PLATFORM.md` and `firmware/README.md`.
2. **Grounded video.** Install dependencies/weights before joining aircraft Wi-Fi. Confirm 720p capture as delivered, BGR/RGB order, newest-frame behavior, Wi-Fi loss and camera-to-host timing. Do not infer exposure time from receipt time.
3. **Range bench.** Compare built-in forward ToF against measured distances on white, dark and glossy surfaces and at oblique angles with the guided `python -m objectpilot.tof_bench` session ([TOF_BENCH.md](TOF_BENCH.md)). Downward aircraft ToF is not forward clearance. Stock SDK replies may cache old samples; use them for bench measurements first.
4. **Sample freshness.** Compile and inspect the prepared ESP32 extension for the actual board. Verify `EXT oprange?` routing, sequence wrap/restart, invalid readings, sample age and query RTT end to end. The command is application-defined, not an official DJI SDK command. Nothing here flashes the board automatically.
5. **Flight interface.** Integrate the command dispatcher with one qualified sink, explicit operator takeoff/hold/landing, an independent command-age watchdog, fresh telemetry and a confirmed landing state. The provided scaffold has no takeoff method and is not connected to the ground-only replay CLI.
6. **Response and clearance.** In a clear supervised indoor area with a lit, textured floor, verify channel signs, low-speed motion, yaw response, stopping drift and low-battery behavior. Floor tape marks and a side video are the baseline ground truth; a mission pad under the path (`mon`, `mdirection 0`, recognizable 0.3–1.2 m above the pad) can add 20 Hz pad-relative position in the state stream, after checking it against the tape. Use physical measurements to set clearances before approaching a wall. Confirm onboard radio-loss behavior independently: the PC cannot guarantee delivery after Wi-Fi failure.
7. **PID before policy.** Begin with short supervised baseline trials, then repeat from matched starts. Record minimum clearance, final range/centering, time, failure and manual intervention. Update and validate the simulator before comparing any PPO candidate in flight.

Ground-only commands:

```bash
python -m pip install -e '.[vision,video,hardware]'
python -m objectpilot.replay --source tello --range-mode stock --out runs/ground-stream.jsonl --show
python -m objectpilot.bench --seconds 15 --out runs/stock-range.jsonl
# Only after custom firmware and query routing have been verified:
python -m objectpilot.replay --source tello --range-mode custom --out runs/custom-range-stream.jsonl --show
```

An external VL53L1X is optional. Verify breakout voltage/logic levels, I2C address ownership, XSHUT wiring, optical axis and payload. The built-in and external sensors can both use 0x29; do not enable both blindly. Keep one range backend and the measured operating envelope. IR time-of-flight ranging is not IR imaging and does not solve dark-camera detection or downward position hold. Night flight remains outside the first demonstration.
