# Planner approach flight

Status, November 16, 2026: the first planner flew on November 13; it reached the print but over-corrected and missed a print during one search. The changed planner flew on November 15 and 16 and arrived four times from starts facing away: it no longer over-corrects, searches in 30° steps with lower-confidence candidates, and stops 0.50 m from the wall. Two 2.5 m searches found no cup; detection now also searches image crops (see November 15, evening). Spoken target selection (`--listen`) first worked in flight on November 16 (`approach-voice-07`, second attempt), after three kinds of failure: a gated microphone, a keyboard click ending the recording, and a raw capture that dropped a third of its samples. The recorder now reads through a callback and recognizes the whole recording.

`objectpilot.step_approach` connects `StepPlanner` ([STEPWISE.md](STEPWISE.md)) to the aircraft. It reuses the guarded `FlightSession` of the step calibration flight ([STEP_FLIGHT.md](STEP_FLIGHT.md)): the same allowlist, pre-flight checks, typed `fly` confirmation, takeoff, climb and abort-and-land. After the climb the planner takes over. Each hover gives one observation: 6 frames through YOLO, 6 `EXT tof?` replies and the state-stream yaw. The planner chooses one SDK step command, the aircraft executes it and stops, and the loop repeats until the planner reports `arrived`, `not_found`, `range_unavailable` or `timeout`. Then the aircraft lands.

## Safety layers

| Layer | Rule |
|---|---|
| Allowlist | Moves of 20–100 cm, turns of 1–90°, `takeoff`, `land`, `stop`, `speed`, queries. A planner command outside it aborts the flight (`planner_command_refused`) and lands. |
| Planner (minimum 0.35 m) | ToF under 0.40 m → `back 20`. A forward step must end at least 0.43 m from the wall, and at least 0.38 m if it overshoots by 30%. No strafe closer than 0.70 m: back off first. A target lost inside the stand-off → `back 20` before turning. At most 90 commands and 250 observations. |
| Flight layer | ToF under 0.32 m → `back 20` and land; a `forward` that would end under 0.32 m is refused and lands; battery under 25%; state packet older than 1.5 s; height outside 40–230 cm; more than 5 `error` replies or 2 timeouts; 4 minutes in the air. A takeoff answered with an error (for example `error, high temp`) means the motors never started: the flight ends as `takeoff_refused` without landing commands. |
| Aircraft | If the aircraft descends below 40 cm on its own (no `down` sent), the flight ends as `aircraft_descending_on_its_own` with the battery level and lands. On November 15 the TT did this at 32% battery: it started descending by itself and answered the next turn with `error Not joystick`. |
| Operator | Ctrl+C stops (`stop`) and lands at any time. If the program dies, the aircraft lands by itself 15 s after its last command. Planner waits longer than 8 s send `battery?` so a long hover does not trigger that timeout. |

## Setup

- The same room setup as the calibration flight: lit, textured floor, propeller guards, nobody within 2 m of the path, firewall prompt allowed, the Tello app closed, `models/yolo11s.pt` present.
- **Flight height:** the camera tilts 10.5° down, so at a stand-off of s metres the print center should sit 100·s·tan 10.5° cm below the lens: 9 cm at 0.50 m. The checklist prints the matching print-center height. For the cup print at 148 cm, use `--height-cm 157`. The planner corrects height itself only beyond 0.8 m, and at most twice.
- **Battery:** pre-flight requires 40%. At low charge the battery falls fast: 51% → 24% in 55 s on November 13. An approach takes 35–90 s, and the flight lands at 25%. Starting below about 55% risks a landing before arrival.
- **Heat:** the TT refuses takeoff with `error, high temp` after a long time powered on with video. Power it off for a few minutes or cool it.
- Start on the floor 1.5–3 m from the wall, within about 45° of the print's normal. Clear path; nothing else within 1 m of the line to the print.

```bash
python -m objectpilot.step_approach simulate --episodes 300 --noise configs/tt_step_noise_measured.json --out runs/approach-sim
python -m objectpilot.step_approach fly --class-name cup --weights models/yolo11s.pt --height-cm 157 --out runs/approach-01
python -m objectpilot.step_approach summary runs/approach-01/flight.jsonl
```

Options:
- both modes: `--stand-off-m` (0.50) and `--minimum-m` (0.35); the flight aborts 3 cm inside the minimum, and the stand-off must be at least 12 cm beyond the minimum. Also `--max-seconds` (240).
- `fly`: `--confidence` (0.25: the detector threshold for search candidates; the planner still confirms at 0.45), `--imgsz` (960, the TT stream width) and `--no-frames`.
- `simulate`: `--episodes`, `--seed` (60000 or higher), `--noise`, `--height-cm` (154 for the test world's 145 cm prints) and `--keep-logs`.

`fly` writes:
- `flight.jsonl`: every command with the planner's phase and reason, every planner decision, observation and state packet
- the observed frames
- `summary.json`: outcome, commands by phase and reason, waits, the closest ToF reading, and the final box, range and yaw

## Spoken target

The TT has no microphone, so the operator names the target into the laptop. With `--listen`, the flight asks after the typed `fly` confirmation and before takeoff:

1. Press Enter, then say the target: a single word such as "cup", or a sentence, in English or Chinese. Or type it.
2. The laptop microphone records from Enter until 1 s of silence after speech, at most 6 s. Speech counts as started on **voiced** sound only: two 30 ms blocks with a pitch (normalized autocorrelation at least 0.5 for 80–400 Hz) among the last five loud ones. Keyboard clicks and taps have no pitch (none of 25 Enter-click blocks reached 0.5; 44% of the operator's voice blocks did), so they neither start nor end a recording. The endpointer only decides when to stop: the **whole recording** goes to recognition, including when no voiced start was found.
   - Capture uses **WASAPI exclusive mode** on the default microphone (48 kHz). That is the raw signal: Windows voice processing is bypassed, so its noise gate cannot cut the start of a word.
   - The stream is read through a **callback**. Blocking reads in exclusive mode silently dropped about a third of the samples (see November 16).
   - Downsampling to 16 kHz low-passes first, so the raw stream's 8–24 kHz noise does not fold into the speech band.
   - If exclusive mode is not available, the recorder uses the shared device and warns when its input is gated.
3. **Vosk keyword recognition** runs offline in grammar mode, with a small English and a small Chinese model (about 40 MB each). Each model can only output the vocabulary words it knows or `[unk]`, so a short word is chosen among the targets instead of being transcribed freely. Words at confidence 0.5 or more from either model are kept. Both models together take about 0.1 s.
4. The recognized words are matched against `configs/voice_targets.json`: words and synonyms per detector label (`cup`, `bottle`, `handbag`), in English and Chinese.
   - English matches whole words with plurals, plus near misspellings when typed.
   - Words of two labels in one utterance count as ambiguous, unless the two language models disagree and one label's summed confidence is at least 0.25 ahead.
   - Traditional forms and homophones in the file only matter for typed text; the grammars use the words each model knows.
5. The flight prints what each model heard, with confidences, and which label that is. It asks up to three times. With no target it ends as `no_target` and does not take off. With a target it retargets the detector and planner and takes off.

Each attempt is logged as a `voice` row in `flight.jsonl`: recognized words per language, capture path, noise floor and gating, whether a voice was detected (`heard`), seconds recorded against seconds elapsed (`captured_s`, `wall_s`; the recorder warns when audio went missing), and result. Every spoken attempt's whole recording is saved as `voice-N.wav`.

The Chinese words in the vocabulary file are stored as JSON `\u` escapes, because the repository check keeps tracked files ASCII. `python -m objectpilot.voice words` prints them decoded.

Setup, while the laptop still has internet (on the aircraft's Wi-Fi it has none). The models go to `models/vosk/`, which Git ignores like the YOLO weights.

```bash
python -m pip install -e '.[voice]'
python -m objectpilot.voice download
python -m objectpilot.voice devices
python -m objectpilot.voice listen
python -m objectpilot.step_approach fly --listen --weights models/yolo11s.pt --height-cm 157 --out runs/approach-voice-07
```

- `voice devices` lists the microphones and the capture path the recorder will use.
- `voice listen` is a microphone test without the aircraft.
- `voice file clip.wav` recognizes a saved clip.
- `voice parse "find the bottle"` checks the matching alone.

### Why not Whisper (November 15)

The first version transcribed freely with faster-whisper (`base`). In flights `approach-voice-03` and `-04` the operator said "cup" and once Chinese "cup" twice. It reached no target:
- **Transcripts:** `''`, `Up!`, `Mark.`, `Up!`, `Up.`, and `bag, <Chinese cup>`, which counted as ambiguous.
- **What the clips show:** each saved clip had 85–100% exact-zero samples around the word, against 0.3% in exclusive mode. Windows noise suppression on the Intel microphone array was gating the input, probably reacting to the drone's fan. A gate that opens on loudness drops the quiet /k/, which leaves "up".
- **Whisper `small` could not recover it either:** `Hot`, `Hop`, `Pop`. Free transcription is weakest on isolated short words.

Same six clips plus eight Windows text-to-speech clips (Zira, English; Huihui, Mandarin):

| Recognizer | November 15 microphone clips | Synthesized clips | Time per command |
|---|---|---|---|
| faster-whisper `base` (first version) | 1 of 6 | 8 of 8 | about 2.2 s |
| faster-whisper `small` | 1 of 6 | 8 of 8 | 3–7 s |
| **Vosk grammar, English + Chinese** | **6 of 6** | **8 of 8** | **about 0.1 s** |

The Vosk results for the six recorded clips: `cup` five times (confidence 0.65–1.0), once `mug` (0.79, also cup), and the Chinese word from the Chinese model (0.93–1.0). The synthesized set includes a sentence with no target, which both versions refused. The same clips also pass when fed through the 48 kHz raw-capture path at a tenth of their level. The new capture path itself has not been tried with a real voice yet.

## November 16: the raw capture dropped a third of its samples

Flight `approach-voice-06` (battery 87%) **arrived** 80 s after takeoff with 22 planner commands, after two search turns. One 26 cm strafe timed out and was sent again. The target came from typing: both spoken attempts failed.
- **Attempt 1:** "no voiced speech heard". 11 loud blocks were rejected as clicks; no block had a pitch.
- **Attempt 2:** 28 loud blocks were rejected. Speech started on two pitched blocks of low room hum after the word; the saved clip holds only the word's last 0.3 s, and neither model heard anything.

Cause, found before changing anything:
1. **The operator's voice reached the recorder without pitch.** In the saved clip the word is a smeared band below 1.5 kHz without harmonics, voicing 0.1–0.4. The same voice in the shared-mode clips of November 15 shows clear harmonics.
2. **The raw stream lost samples.** Every exclusive-mode clip (`approach-voice-05` and `-06`) has its largest sample-to-sample jump exactly at the 30 ms read boundary, 5–8 times the median; the six shared-mode clips have none. Live on the laptop, 2 s of blocking reads returned 1.32 s of audio (6 s: 3.96 s) with no overflow flag. The same device read through a callback returned 1.98 s (6 s: 6.0 s) and no boundary jump. Speech cut into pieces every few milliseconds has no pitch, so the November 15 endpointer filed it with the clicks.
3. **The endpointer threw the audio away.** It returned only what came after a voiced start, so a missed start lost the word for good. My November 15 tests fed audio files to the endpointer and never used the live stream.

Fixes: callback capture, the whole recording to recognition, low-pass before downsampling, the `heard`/`captured_s`/`wall_s` log fields, and a stop 1 s after speech once speech started (it had required a third pitched block, and a clipped "cup" can have only two).

Check on the operator's six November 15 clips (shared mode, intact), scaled to raw-microphone level and fed as a 48 kHz stream of raw room noise from the live callback capture, the real Enter click from `approach-voice-05`, then the word 0.9 s or 2.9 s later:
- 11 of 12 recognized as cup; the miss is a short unclear clip at one delay, recognized at the other
- every short word ended the recording about 1.5 s after it
- before the stop fix, two late words were never marked as started and ran to the 6 s limit; whole-recording recognition still got both

A test word played through the laptop speakers did not reach the microphone (speakers silent).

Flight `approach-voice-07` (battery 90%), the same afternoon with the new recorder: **the spoken target worked and the flight arrived.**
- Attempt 1 recorded 1.80 s in 1.81 s, so nothing was lost, but it held no word. A quiet pitched sound, about a seventh of the word's level, counted as speech; 1 s of quiet after it ended the recording before the operator spoke. Both models correctly returned nothing.
- Attempt 2: `cup` at confidence 1.0 from the English model. The clip shows the word with clear harmonics.
- The planner searched 11 turns (about 350°), found the cup and arrived 127 s after takeoff: 32 commands, no errors.

## November 15, evening: voice and a 2.5 m search

Three more flights failed:

| Flight | Start | What happened | Cause |
|---|---|---|---|
| `approach-voice-05` | `--listen`, raw capture | Three attempts, nothing recognized. Both models returned no word at all, not even `[unk]`. | Each saved clip is one 0.15–0.25 s broadband burst with no pitch: the Enter key, which the raw microphone now hears. The recorder took it as speech and stopped 0.8 s later, about 1.4 s after Enter, before the word was spoken. Windows noise suppression had removed such clicks before. My earlier check used clean speech and Gaussian noise, so it never contained one. |
| `approach-2026-11-14/06`, `07` | 2.5 m, facing away, cup among distractor prints, `--class-name cup` | 12 search turns over 360° without one cup detection at the 0.25 threshold; 06 stopped by Ctrl+C, 07 `not_found`. | The cup print was in view at two headings (frames saved). At that distance it is about 50 px wide in the 960×720 stream, and YOLO11s scores it erratically. At imgsz 960 (the setting since November 13) it scored 0.04–0.07; at 1280 it scored 0.23–0.66; on 2/3 crops at 640 it scored a median of 0.37, seen in 5 of 6 frames. In run 06 the drone stood farther away and no setting scored it above 0.19. |

Fixes:
1. **Voice:** endpointing starts on voiced sound only (step 2 above). Tested on composites of the real click and raw room noise from `approach-voice-05`, followed by spoken words at raw-microphone level:
   - all four were captured from the word, not the click, and recognized
   - the click-only clips recorded nothing
   - the five earlier spoken clips are all recognized, including the 4 s clip with a pause before the second word
2. **Detection:** while the target is unseen or small (box under 20% of the frame height), each frame is searched on the full frame and on four overlapping 2/3 crops at 640 (`objectpilot.vision.detect_tiled`). Otherwise the full frame alone is searched at 640. Replaying the saved search frames:
   - run 07 gets the cup at −67° (0.29, 4 of 6 frames) and −35° (0.38, 5 of 6), where 960 had nothing
   - the earlier finds score 0.82–0.89 instead of 0.63–0.89
   - none of the 41 search headings in five flights produced a false cup (people, chairs, backpack and bottle prints)
   - run 06 still finds nothing
   - cost: about 0.25 s per frame with crops, 0.06 s without
3. **Planner:** a centered search candidate that is small (box height under 0.26) and below the 0.45 confirmation gets up to two 40 cm forward steps toward it before the two confirming looks (`candidate_approach`). Detector confidence rises as the print grows: about 0.3 at 2.5 m, 0.8–0.9 at 1.8 m.

The pretrained detector's range for this cup print in the live stream is about 2.5 m. Beyond that a detector trained on TT frames of the prints is needed; the flight logs now hold several hundred such frames.

## Flights on November 15, 2026

Log folder `runs/approach-2026-11-14`. Cup print with distractor prints; start 1.8 m out, facing away so the print was out of view. Changed planner, stand-off 0.50 m, flight height 157 cm.

| Flight | Battery at takeoff | Result |
|---|---|---|
| 01 | 45% | Searched four 30° turns. Then the battery read 32%, the aircraft started descending by itself (vertical speed −6 dm/s) and answered the next turn with `error Not joystick`. At 12 cm the flight layer reported `height_out_of_bounds` and sent `land`. Now reported as `aircraft_descending_on_its_own`. |
| 02 | 7% | Stopped at pre-flight. |
| 03 | 69% | **Arrived** in 49 s, 14 commands: two search turns, face, sweep, one centering strafe, two final steps. ToF 0.54 m. |
| 04 | 45% | **Arrived** in 53 s, 15 commands, one centering strafe. ToF 0.46 m; battery 28% at landing. |
| 05 | 26% | Stopped at pre-flight. |

The over-correction of November 13 is gone: one centering strafe per arrival, against ten in the 30° flight. Battery: 45% at takeoff led once to a self-landing after 18 s and once to an arrival at 28%. Takeoff at 60% or more leaves margin.

## Flights on November 13, 2026

Cup print center at 148 cm, flight height 160 cm, stand-off 0.65 m, the planner as first written.

| Flight | Start | Result |
|---|---|---|
| 1 | 1.5 m straight in front, facing the print | Takeoff refused: `error, high temp`. The script then sent `land` five times and warned that landing was not confirmed; fixed (see Safety layers). |
| 1, repeat | Same | **Arrived** in 38 s, 13 planner commands: face, sweep, one 38 cm strafe, one forward step, verify, 0.66 m. |
| 2 | 1.8 m from the print, 30° off its normal | **Arrived** in 76 s, 25 commands, 17 of them centering and distance corrections (see [STEPWISE.md](STEPWISE.md)). |
| 3 | 1.8 m in front, turned away; other prints on the wall as distractors | Found the cup after two `cw 40` turns and squared up; landed at 25% battery during the verify refit (51% at takeoff). |
| 3, repeat | Same; three backpack prints beside the cup and a person in view | 9 × `cw 40` covered 360° (state yaw 0, 38, 77, …, −14), then `not_found`. |

Three more attempts stopped at pre-flight on the old 50% battery threshold (40%, 17% and 45%), and one with no SDK reply.

**Why the last search missed.** The saved frames show the cup print sharp and complete at yaw 77° (at the right edge, cx 0.83) and at 115° (left edge, cx 0.13). It sat about halfway between two 40° views. These are not motion-blurred or stale frames: every observation's six frames match each other. YOLO11s found it in each frame, but below the 0.45 threshold:

| Frame | Detected at 640 px (as flown) | Detected at 960 px |
|---|---|---|
| yaw 77°, cup at the right edge | 0.43, 0.16 | 0.36, 0.39 |
| yaw 115°, cup at the left edge | 0.19, 0.20 | 0.60, 0.54 |

SDK rotations have a fixed rate. Every observation happens after the turn has stopped, so a slower turn would not have changed the frames. Changes:
- 30° search steps, so the print is within 15° of some view's center.
- Inference at the 960 px stream width.
- A 0.25 detector threshold. A box below 0.45 is a candidate: the planner turns it to the image center and requires 0.45 within two looks. Otherwise it skips that bearing and searches on.

## Simulation through the full flight loop

`simulate` runs `ApproachFlight` against the geometric test world from the `stepsim.scenario` starts: 1.5–3.2 m away, up to 50° off the print's normal, 40% facing a random direction. It runs the same code as `fly`, including takeoff, climb, the flight-layer guards and landing. It is scored with the same rule as stepsim: arrival with the true stand-off within 8 cm, offset within 8 cm, heading within 8°, and never inside the minimum. Seeds 60000–60299, stand-off 0.50 m, height 154 cm:

| Noise | Success | 0–15° / 15–30° / 30–50° | Heading error (median) | Offset (median) | Commands | Closest to wall |
|---|---|---|---|---|---|---|
| Default (assumed) | 98.0% | 100 / 99.0 / 95.2% | 1.7° | 2.3 cm | 20 | 0.42 m |
| Flight-measured | 96.3% | 96.7 / 99.0 / 93.3% | 1.5° | 2.5 cm | 25 | 0.41 m |

With the flight-measured noise, the planner as flown on November 13 reaches 71.0% here, with a median of 19 centering and distance corrections; the changed planner needs 7. Remaining failures with that noise: five `not_found` (starts beyond 3 m, where YOLO11s cannot detect the print, or 45–50° off the normal, where the assumed detection falloff applies) five arrivals 8.1–9.2 cm off center, and one that stopped at 0.42 m. Simulated time is not flight time: a simulated observation takes 0.6 s, a real one about 1 s.

## Limitations

- The changed planner has flown twice (one start, one room). Oblique starts have not flown with it.
- The spoken target has worked in one flight, one operator, one room, before takeoff (no propeller noise). A quiet pitched sound before the word can end a recording early; the operator then repeats (`approach-voice-07`, attempt 1).
- Pretrained YOLO11s finds the cup print to about 2.5 m in the live stream (with crops); farther starts need a fine-tuned detector.
- Detection confidence is not modeled in the test world (every simulated detection scores 0.9), so the candidate search is covered by unit tests and the flight frames only.
- Detection against viewing angle is assumed (falling from 35° to 65°). Measure it on the stand before trusting oblique starts.
- Only the cup print is measured. The bottle and third prints, and their sizes, are unscreened.
