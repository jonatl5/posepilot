# Forward ToF bench procedure

This is hardware gate 3: measure the stock forward ToF (`EXT tof?`) against tape distances, with the motors off. `python -m objectpilot.tof_bench` guides the operator point by point. It labels every reply with the planned distance, surface and angle, and writes raw samples and per-point summaries. The tool talks through the ground-only `GroundLink` allowlist, so takeoff, landing, motor and RC commands cannot be sent. Interfaces are summarized in [TT_PLATFORM.md](TT_PLATFORM.md).

## What it answers

- Accuracy (bias, MAE, p95 error) and validity per distance on the whiteboard, the cup print and a dark print.
- The usable head-on range per surface. This decides `forward_range_max_m` and `near_entry_m` ([simulator finding 2](CONTROLLER_TUNING.md#three-simulator-findings-that-gains-cannot-fix)).
- Freshness: how often the stock value actually changes and the longest run of identical replies. Stock replies have no sample age, and the factory example caches the last value.
- Query round-trip time, oblique-angle behavior, the out-of-range reply in open space, and a blocked sensor.

## Before the session (with internet)

1. On this laptop, the repository environment already has the hardware extras: Python 3.12 `.venv`, `djitellopy`, `pyserial` and `av`, with `pip check` clean. On another laptop:

   ```bash
   py -3.12 -m venv .venv
   .venv\Scripts\python.exe -m pip install -e ".[vision,video,hardware,test]"
   .venv\Scripts\python.exe -m pytest -q tests/test_tof_bench.py
   ```

2. Charge the battery and update the aircraft firmware in the Tello app. A forum report says `EXT tof?` needed aircraft firmware v02.05.01.19.
3. Leave the open-source controller on its **factory firmware**. The stock `EXT tof?` command is answered by it.
4. Prepare the surfaces: a spare cup print and a sheet printed solid black, each taped flat **at the height of the ToF window** so the beam lands on them, plus the bare whiteboard. Also find a direction with nothing within 3 m, and keep a small card ready.
5. Stand: the aircraft sits level (use a phone level), sensor window facing the wall squarely. Distances are measured horizontally **from the surface to the front window of the ToF sensor on the top module**. Write down once the horizontal and vertical offsets between that window and the camera lens.

## Connect

1. Expansion-controller switch in **direct-connection** mode. Power on the aircraft.
2. Close the Tello app; one controlling client at a time.
3. Connect the laptop's Wi-Fi to `RMTT-XXXXXX`. The laptop loses internet while connected.
4. On the first run, Windows may ask whether Python may use the network. Allow it for the network type the RMTT Wi-Fi is assigned (usually Public). Without that, no reply or state packet arrives. The tool needs UDP 8889 for replies and 8890 for the state stream. Only you should change firewall settings.

## Run

List the points first (no aircraft needed): 31 points, 10 s each.

```bash
.venv\Scripts\python.exe -m objectpilot.tof_bench --dry-list
```

Run it in three short sessions. The aircraft warms up while idle, so power it off for about 10 minutes between sessions if it feels hot. The tool records the board temperature (`temph`) and battery whenever the state stream arrives.

```bash
.venv\Scripts\python.exe -m objectpilot.tof_bench --surfaces whiteboard --out runs\tof\2026-11-08-whiteboard
.venv\Scripts\python.exe -m objectpilot.tof_bench --surfaces cup_print --out runs\tof\2026-11-08-cup
.venv\Scripts\python.exe -m objectpilot.tof_bench --surfaces dark_print,open_space,blocked --out runs\tof\2026-11-08-other
```

At each point the tool prints the surface, the tape distance and angle, and an instruction.

1. Place the stand on the mark with the window over the line, then step out of the beam.
2. Press **Enter**. Keep everyone still for 10 s.
3. Read the summary line: valid %, out-of-range %, median, bias, p95 error, value changes per second and RTT.
4. Press **Enter** to accept, or **r** to redo if the stand moved, someone crossed the beam or the numbers look wrong. `s` skips a point and `q` ends the session; accepted points are already saved.

For the 20° and 40° points, rotate the aircraft about its vertical axis on the same mark. A phone compass or a protractor on the floor is enough.

## Output

Each `--out` directory contains:

| File | Content |
|---|---|
| `session.json` | Start/end time, aircraft identity (`sdk?`, `sn?`, `hardware?`, `battery?`, `wifi?`, `EXT version?`), plan reference, git revision, end battery, usable range per surface |
| `points.csv` | One row per accepted point: validity, out-of-range and failure fractions, median, bias, MAE, p95 error, standard deviation, changes per second, longest identical run, RTT median/p95, battery, board temperature |
| `samples.jsonl` | Every query of every attempt: time, raw reply, status, measured distance, raw millimeters, RTT, state snapshot (`bat`, `temph`, `templ`, downward `tof`, `h`) |

The "usable range" line is the largest head-on distance from 0.40 m upward where every tested point has at least 90% valid replies, |bias| ≤ 3 cm and p95 error ≤ 5 cm. It is a summary for the team decision, not a flight clearance.

## Troubleshooting

| Symptom | Check |
|---|---|
| `Address already in use` | Another tool (replay, bench, a second terminal) holds UDP 8889 or 8890. Close it, or add `--no-state` if only 8890 is taken. |
| Identity shows `unavailable` and nothing answers | Laptop on `RMTT-…` Wi-Fi? Tello app closed? Firewall prompt allowed? Aircraft on? |
| `hardware?` says `TELLO` | The expansion controller is not detected; reseat it. |
| `EXT tof?` times out or answers `error` | Run `--probe --out runs\tof\probe-1`: it sends a few queries and saves every line the aircraft returns, with arrival times, to `probe.json`. If replies arrive after more than 1.5 s, raise `--timeout`. Otherwise check the factory controller firmware, aircraft firmware version and direct-connection mode. Every sample also stores late replies (`late_lines`) and unexpected lines (`other_lines`). |
| Expansion-controller RGB light blinks blue | Normal with factory firmware: it is waiting for a Bluetooth gamepad and says nothing about Wi-Fi. The SDK link shows on the aircraft status light (slow purple = SDK connected, fast purple = no command for 15 s). |
| Replies `unknown command: keepalive` | The controller forwards replies to its own keepalive traffic. Since November 7, `GroundLink` skips these lines; earlier runs counted them as failures, so repeat such points. |
| Aircraft powers itself off | Thermal shutdown. On November 7 the TT switched off at `temph` 99 °C, idle on the stand, after roughly 15 minutes powered on. The tool asks before measuring at 90 °C or above (`--max-temp`), stops a point at 97 °C (`--stop-temp`) and prints the `--from` point to resume with. Start each session cool (below about 70 °C), aim a desk fan at the aircraft, and power it off between sessions. |
| Dot-matrix shows "WiFi UNACTIVE" | The expansion controller is not activated. Connect the phone to `RMTT-…` with mobile data on and activate it in the Tello app (1.6.0 or later); `EXT tof?` does not work before that. |
| Almost all replies `out_of_range` at short distances | The beam misses the surface: check the window height against the taped sheet. |

## After the session

Keep the three `runs/tof/…` folders; they stay local because `runs/` is ignored. Send or point to them, together with the window-to-lens offsets. The next steps are:

- set `forward_range_max_m` and `near_entry_m` from the measured usable range on the cup print;
- decide from the freshness numbers whether the stock reply is enough or the `EXT oprange?` firmware ([firmware/README.md](../firmware/README.md)) is needed;
- feed the noise and invalid rates into the simulator.

`python -m objectpilot.bench` remains a minimal free-running logger; `tof_bench` is the labeled procedure for gate 3.
