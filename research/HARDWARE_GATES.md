# Hardware bring-up and first-flight gates

The aircraft is still in shipping. No real flight, SDK stream, sensor freshness or board compilation is established by the software checks. The 0.65 m stand-off and 0.40 m minimum are simulation settings until measured braking, body offsets and sensing error justify physical clearances.

1. **Identify the platform.** Confirm the delivered TT/Tello model, firmware, expansion board, sensor direction, voltage, payload and restoring procedure. Follow the official hardware documentation linked in `references.md` and `firmware/README.md`.
2. **Grounded video.** Install dependencies/weights before joining aircraft Wi-Fi. Confirm 720p capture as delivered, BGR/RGB order, newest-frame behavior, Wi-Fi loss and camera-to-host timing. Do not infer exposure time from receipt time.
3. **Range bench.** Compare built-in forward ToF against measured distances on white, dark and glossy surfaces and at oblique angles. Downward aircraft ToF is not forward clearance. Stock SDK replies may cache old samples; use them for bench measurements first.
4. **Sample freshness.** Compile and inspect the prepared ESP32 extension for the actual board. Verify `EXT oprange?` routing, sequence wrap/restart, invalid readings, sample age and query RTT end to end. The command is application-defined, not an official DJI SDK command. Nothing here flashes the board automatically.
5. **Flight interface.** Integrate the command dispatcher with one qualified sink, explicit operator takeoff/hold/landing, an independent command-age watchdog, fresh telemetry and a confirmed landing state. The provided scaffold has no takeoff method and is not connected to the ground-only replay CLI.
6. **Response and clearance.** In a clear supervised indoor area with a lit, textured floor, verify channel signs, low-speed motion, yaw response, stopping drift and low-battery behavior. Use physical measurements to set clearances before approaching a wall. Confirm onboard radio-loss behavior independently: the PC cannot guarantee delivery after Wi-Fi failure.
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
