import sys
import threading
import time
import types

import numpy as np
import pytest

from objectpilot.step_approach import ApproachFlight, MemoryRecorder, approach_plan
from objectpilot.step_flight import SimAircraft
from objectpilot.voice import (Endpointer, ask_target, examples, load_vocabulary, load_wav, main, match_target,
                               record_utterance, resample, resolve_by_confidence, save_wav, voicing)

VOCABULARY = load_vocabulary()
# Chinese test sentences as escapes: the repository check keeps tracked files ASCII.
HELP_FIND_CUP = "\u5e2e\u6211\u627e\u4e00\u4e0b\u676f\u5b50"        # "help me find the cup"
WANT_BOTTLE = "\u6211\u8981\u627e\u74f6\u5b50"                       # "I want the bottle"
SCHOOLBAG_TRADITIONAL = "\u53bb\u627e\u66f8\u5305"                   # "go find the school bag", traditional
QUILT = "\u88ab\u5b50"                                               # homophone of "cup" (bei zi)
WATER_CUP = "\u6c34\u676f"                                           # "water cup": contains "cup" (bei)
WEATHER = "\u4eca\u5929\u5929\u6c14\u4e0d\u9519"                     # "nice weather today"


@pytest.mark.parametrize("text, label", [
    ("Find the cup please.", "cup"), ("go to the BOTTLES", "bottle"), ("fly to the backpack", "handbag"),
    (HELP_FIND_CUP, "cup"), (WANT_BOTTLE, "bottle"), (SCHOOLBAG_TRADITIONAL, "handbag"), (QUILT, "cup"),
    (WATER_CUP, "cup"), ("find the bottel", "bottle"), ("Ｆｉｎｄ the cup", "cup"),
])
def test_spoken_sentences_map_to_one_label(text, label):
    assert match_target(text, VOCABULARY).label == label


@pytest.mark.parametrize("text, reason", [
    ("the cupboard", "no_target_word"), (WEATHER, "no_target_word"), ("", "empty"), ("  ...  ", "empty"),
    ("the cup or the bottle", "ambiguous"),
])
def test_unclear_sentences_name_no_label(text, reason):
    match = match_target(text, VOCABULARY)
    assert match.label is None and match.reason == reason


def test_vocabulary_labels_are_detector_classes():
    assert set(VOCABULARY) == {"cup", "bottle", "handbag"}          # COCO names used by YOLO11


def tone(seconds, rate, amplitude):
    t = np.arange(int(seconds*rate))/rate
    return (amplitude*np.sin(2*np.pi*220*t)).astype(np.float32)


def test_endpointer_keeps_the_utterance_and_stops_after_silence():
    rate, block = 16000, 480
    rng = np.random.default_rng(0)
    signal = np.concatenate([rng.normal(0, .002, rate), tone(1., rate, .1), rng.normal(0, .002, 2*rate)])
    end, state = Endpointer(rate), None
    for k in range(0, len(signal), block):
        state = end.feed(signal[k:k+block])
        if state == "done":
            break
    assert state == "done" and end.heard
    assert 2.9 <= len(end.audio())/rate <= 3.1           # the whole recording: 1 s room, 1 s tone, 1 s silence
    quiet = Endpointer(rate)
    for k in range(0, 3*rate, block):
        quiet.feed(rng.normal(0, .002, block))
    assert not quiet.heard and len(quiet.audio()) == 3*rate and not quiet.gated
    gated = Endpointer(rate)                              # Windows noise suppression: exact zeros
    for k in range(0, rate, block):
        gated.feed(np.zeros(block, np.float32))
    assert gated.gated


def test_resample_and_wav_round_trip(tmp_path):
    audio = tone(.5, 44100, .3)
    assert len(resample(audio, 44100)) == 8000
    save_wav(tmp_path/"a.wav", resample(audio, 44100))
    assert np.allclose(load_wav(tmp_path/"a.wav"), resample(audio, 44100), atol=1e-3)


def test_ask_target_retries_then_takes_voice_or_typed_answer():
    info = dict(capture="test", gated=False, noise=0.)
    replies = iter(["", "", "the cup or the bottle", "bottle please"])
    heard = iter([(None, info), (np.zeros(800, np.float32), info)])
    recognize = lambda audio: ("bottle", {"en": [["[unk]", 1.], ["bottle", .9]], "zh": [["[unk]", 1.]]})
    label, records = ask_target(VOCABULARY, lambda: next(heard), recognize,
                                input_fn=lambda prompt: next(replies), echo=lambda *a: None, attempts=4)
    assert label == "bottle" and [r["reason"] for r in records] == ["no_audio", "matched"]
    assert records[-1]["source"] == "voice" and "audio" in records[-1] and records[-1]["capture"] == "test"
    label, records = ask_target(VOCABULARY, lambda: (None, info), lambda audio: ("", {}),
                                input_fn=lambda prompt: "the cup or the bottle", echo=lambda *a: None, attempts=2)
    assert label is None and [r["reason"] for r in records] == ["ambiguous", "ambiguous"]


def test_prompt_examples_name_every_label():
    text = examples(VOCABULARY)
    assert text.startswith("cup/") and "bottle/" in text and "bag/" in text


def voice_flight(answer):
    plan = approach_plan(height_cm=154)
    aircraft = SimAircraft(61500, plan, pose=(-1.6, -.9, .3))
    recorder = MemoryRecorder(aircraft.clock)
    flight = ApproachFlight(aircraft, plan, recorder, clock=aircraft.clock, echo=lambda *a, **k: None,
                            choose_target=lambda: (answer, [dict(attempt=1, source="voice", text=str(answer),
                                                                 label=answer, reason="matched")]))
    return flight, aircraft, recorder


def test_spoken_target_is_searched_after_confirmation_and_before_takeoff():
    flight, aircraft, recorder = voice_flight("bottle")
    assert "you name" in flight.checklist()
    assert flight.run(confirm=lambda text: True) == "arrived"
    events = [r["event"] if r["event"] != "command" else r["command"] for r in recorder.rows]
    assert events.index("voice") < events.index("takeoff")
    assert flight.planner.c.label == aircraft.target.label == "bottle"
    assert abs(aircraft.world.y-aircraft.target.y) < .1           # in front of the bottle print, not the cup


def test_no_spoken_target_means_no_takeoff():
    flight, aircraft, recorder = voice_flight(None)
    assert flight.run(confirm=lambda text: True) == "no_target"
    assert "takeoff" not in [r["command"] for r in recorder.rows if r["event"] == "command"]


def test_parse_command_line(capsys):
    assert main(["parse", "please find the bottle"]) == 0 and capsys.readouterr().out.startswith("bottle")
    assert main(["parse", "nothing here"]) == 1


def test_voicing_separates_a_pitched_sound_from_a_click():
    rate = 48000
    rng = np.random.default_rng(1)
    assert voicing(tone(.03, rate, .1), rate) > .8
    assert voicing(rng.normal(0, .05, 1440).astype(np.float32), rate) < .4


def test_a_click_before_speaking_neither_starts_nor_ends_the_recording():
    rate, block = 48000, 1440
    rng = np.random.default_rng(3)
    room = lambda s: rng.normal(0, .0005, int(s*rate)).astype(np.float32)
    click = (rng.normal(0, .03, int(.2*rate))*np.hanning(int(.2*rate))).astype(np.float32)   # Enter key: no pitch
    signal = np.concatenate([room(.4), click, room(1.2), tone(.6, rate, .05), room(1.3)])
    end, state = Endpointer(rate), None
    for k in range(0, len(signal), block):
        state = end.feed(signal[k:k+block])
        if state == "done":
            break
    assert state == "done" and end.ignored > 0
    assert abs(end.start*.03-1.8) < .1                     # starts at the tone (1.8 s), not at the click (0.4 s)
    only_click = Endpointer(rate)
    for k in range(0, len(signal[:int(1.6*rate)]), block):
        only_click.feed(signal[k:k+block])
    assert not only_click.heard and len(only_click.audio()) > 0


def test_a_recording_without_a_detected_voice_is_still_recognized():
    info = dict(capture="test", gated=False, noise=0., heard=False)
    label, records = ask_target(VOCABULARY, lambda: (np.zeros(16000, np.float32), info),
                                lambda audio: ("cup", {"en": [["cup", .9]], "zh": []}),
                                input_fn=lambda prompt: "", echo=lambda *a: None, attempts=1)
    assert label == "cup" and records[0]["heard"] is False and "audio" in records[0]


def test_downsampling_removes_sound_above_the_new_nyquist_frequency():
    t = np.arange(48000)/48000
    hiss = np.sin(2*np.pi*12000*t).astype(np.float32)          # would fold to 4 kHz without the low-pass
    assert np.sqrt(np.mean(resample(hiss, 48000)[200:-200]**2)) < .01
    speech_band = np.sin(2*np.pi*1000*t).astype(np.float32)
    assert np.sqrt(np.mean(resample(speech_band, 48000)[200:-200]**2)) == pytest.approx(.707, abs=.01)


def fake_sounddevice(signal, block_s=.03):
    """A sounddevice stand-in whose stream pushes `signal` (48 kHz) to the callback."""
    class Stream:
        def __init__(self, samplerate, channels, dtype, blocksize, device, extra_settings=None, callback=None):
            assert callback is not None, "blocking reads lost samples on the real microphone"
            self.callback, self.block, self.running = callback, blocksize or int(samplerate*block_s), True

        def __enter__(self):
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
            return self

        def run(self):
            for k in range(0, len(signal)-self.block+1, self.block):
                if not self.running:
                    return
                self.callback(signal[k:k+self.block].reshape(-1, 1), self.block, None, None)
                time.sleep(.0005)

        def __exit__(self, *exc):
            self.running = False
            self.thread.join()

    return types.SimpleNamespace(InputStream=Stream, PortAudioError=OSError,
                                 query_devices=lambda device=None, kind=None: {"default_samplerate": 48000})


def test_recorder_keeps_every_delivered_sample_and_stops_after_the_word(monkeypatch):
    rate = 48000
    rng = np.random.default_rng(4)
    room = lambda s: rng.normal(0, .0005, int(s*rate)).astype(np.float32)
    click = (rng.normal(0, .03, int(.2*rate))*np.hanning(int(.2*rate))).astype(np.float32)
    signal = np.concatenate([room(.4), click, room(1.), tone(.6, rate, .05), room(5.)])
    monkeypatch.setitem(sys.modules, "sounddevice", fake_sounddevice(signal))
    audio, info = record_utterance(device=0, echo=lambda *a: None)
    assert info["heard"] and info["ignored_clicks"] > 0
    assert 3.1 <= info["captured_s"] <= 3.3                 # 2.2 s to the end of the tone + 1 s silence
    assert len(audio) == pytest.approx(info["captured_s"]*16000, abs=2)   # nothing dropped, the click included
    start = int(.4*16000)
    assert np.abs(audio[start:start+3200]).max() > .02      # the click is in the recording


def test_disagreeing_language_models_resolve_to_the_clearly_more_confident_label():
    cup_word = "\u676f\u5b50"
    assert resolve_by_confidence({"en": [["bag", .71]], "zh": [[cup_word, 1.0]]}, VOCABULARY) == "cup"
    assert resolve_by_confidence({"en": [["bag", .9]], "zh": [[cup_word, 1.0]]}, VOCABULARY) is None
