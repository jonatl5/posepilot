"""Spoken target selection: laptop microphone, offline keyword recognition, fixed vocabulary.

The TT has no microphone. Before takeoff the operator presses Enter and says what to find
("cup", "find the bottle", or the Chinese words) into the laptop, or types it.

Recognition is Vosk in grammar mode, one small model per language (English, Chinese). Each
model can only output the vocabulary words it knows or [unk], so a single short word is chosen
from the targets instead of being transcribed freely. On the November 15 clips (the operator
saying "cup" with the first consonant cut off by Windows noise suppression) free Whisper
transcription wrote "Up!", "Hop", "Mark"; grammar mode returned cup for 6 of 6, in 0.1 s.

Capture prefers WASAPI exclusive mode on the default microphone: it bypasses Windows voice
processing, whose noise gate zeroed the room and the quiet start of each word. The words and
synonyms per detector label are in configs/voice_targets.json. The laptop has no internet on
the aircraft's Wi-Fi, so download the models first.

    python -m objectpilot.voice download                  # fetch the models while online
    python -m objectpilot.voice devices                   # microphones and the capture path used
    python -m objectpilot.voice words                     # print the vocabulary
    python -m objectpilot.voice parse "find the bottle"   # text only, no model
    python -m objectpilot.voice listen                    # microphone test, no aircraft
    python -m objectpilot.voice file clip.wav             # recognize a WAV file
"""
from dataclasses import dataclass, field
import argparse
import difflib
import json
from pathlib import Path
import re
import sys
import time
import unicodedata
import urllib.request
import wave
import zipfile

import numpy as np

SAMPLE_RATE = 16000
ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VOCABULARY = ROOT/"configs"/"voice_targets.json"
DEFAULT_MODEL_DIR = ROOT/"models"/"vosk"
VOSK_MODELS = {"en": "vosk-model-small-en-us-0.15", "zh": "vosk-model-small-cn-0.22"}   # about 40 MB each
VOSK_URL = "https://alphacephei.com/vosk/models/{}.zip"
CJK = re.compile(r"[\u3400-\u9fff\uf900-\ufaff]")   # CJK ideographs; escaped so tracked files stay ASCII


# -- vocabulary and matching -------------------------------------------------------------------
def load_vocabulary(path=None):
    """{label: [phrases]} from a voice_targets.json file."""
    data = json.loads(Path(path or DEFAULT_VOCABULARY).read_text(encoding="utf-8"))
    targets = data["targets"] if "targets" in data else data
    if not targets or not all(isinstance(v, list) and v for v in targets.values()):
        raise ValueError("The vocabulary needs at least one label with a non-empty list of phrases")
    return {label: [str(p) for p in phrases] for label, phrases in targets.items()}


def normalize(text):
    """NFKC, lower case, punctuation to spaces. Chinese characters are kept."""
    return re.sub(r"[^\w\s]", " ", unicodedata.normalize("NFKC", text).lower())


@dataclass
class Match:
    label: str | None
    reason: str                    # matched, fuzzy, ambiguous, no_target_word, empty
    phrases: list = field(default_factory=list)   # [label, phrase] pairs found, in spoken order


def match_target(text, vocabulary):
    """The one label whose words occur in the text.

    Chinese phrases match as substrings; English phrases as whole words, with a plural s/es.
    A match inside a longer match is dropped. Words of two different labels are ambiguous.
    With no exact word, an English word within edit ratio 0.8 of a single-word phrase counts.
    """
    norm = normalize(text or "")
    if not norm.strip():
        return Match(None, "empty")
    found = []
    for label, phrases in vocabulary.items():
        for phrase in phrases:
            key = normalize(phrase).strip()
            if not key:
                continue
            if CJK.search(key):
                pattern = re.escape(key)
            else:
                pattern = r"(?<![a-z])"+r"\s+".join(map(re.escape, key.split()))+r"(?:e?s)?(?![a-z])"
            found += [(m.start(), m.end(), label, phrase) for m in re.finditer(pattern, norm)]
    if not found:
        words = {p: label for label, phrases in vocabulary.items() for p in phrases
                 if " " not in p and not CJK.search(p)}
        for token in re.findall(r"[a-z]{3,}", norm):
            close = difflib.get_close_matches(token, list(words), n=1, cutoff=.8)
            if close:
                return Match(words[close[0]], "fuzzy", [[words[close[0]], close[0]]])
        return Match(None, "no_target_word")
    kept = [f for f in found if not any(g[0] <= f[0] and f[1] <= g[1] and g[1]-g[0] > f[1]-f[0] for g in found)]
    phrases = [[f[2], f[3]] for f in sorted(kept)]
    labels = {f[2] for f in kept}
    if len(labels) == 1:
        return Match(labels.pop(), "matched", phrases)
    return Match(None, "ambiguous", phrases)


def examples(vocabulary):
    """'cup/<Chinese word>, bottle/...' for prompts: the first English and Chinese phrase per label."""
    parts = []
    for label, phrases in vocabulary.items():
        english = next((p for p in phrases if not CJK.search(p)), label)
        chinese = next((p for p in phrases if CJK.search(p)), None)
        parts.append(english+("/"+chinese if chinese else ""))
    return ", ".join(parts)


# -- audio -----------------------------------------------------------------------------------------
def resample(audio, rate, target=SAMPLE_RATE):
    """Linear-interpolation resampling. Downsampling first low-passes below the new Nyquist
    frequency, or the raw 48 kHz stream's 8-24 kHz noise folds into the speech band."""
    x = np.asarray(audio, dtype=np.float64)
    if rate == target or len(x) == 0:
        return x.astype(np.float32)
    if rate > target:
        cutoff, n = .45*target/rate, np.arange(-64, 65)                # 7.2 kHz for 16 kHz output
        taps = 2*cutoff*np.sinc(2*cutoff*n)*np.hamming(len(n))
        if len(x) > len(taps):
            x = np.convolve(x, taps/taps.sum(), mode="same")
    n = int(round(len(x)*target/rate))
    return np.interp(np.linspace(0, len(x)-1, n), np.arange(len(x)), x).astype(np.float32)


def save_wav(path, audio, rate=SAMPLE_RATE):
    data = (np.clip(audio, -1, 1)*32767).astype("<i2")
    with wave.open(str(path), "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(rate)
        f.writeframes(data.tobytes())


def load_wav(path):
    """16-bit PCM WAV (any rate, any channels) as mono float32 at 16 kHz."""
    with wave.open(str(path), "rb") as f:
        if f.getsampwidth() != 2:
            raise ValueError("Only 16-bit PCM WAV files are supported")
        rate, channels = f.getframerate(), f.getnchannels()
        data = np.frombuffer(f.readframes(f.getnframes()), dtype="<i2").astype(np.float32)/32768
    if channels > 1:
        data = data.reshape(-1, channels).mean(axis=1)
    return resample(data, rate)


def voicing(block, rate):
    """Normalized autocorrelation peak for a 80-400 Hz pitch: about 0.6-0.9 for voiced speech,
    below 0.4 for keyboard clicks, taps and breath (November 15 clips)."""
    step = max(1, int(rate//16000))
    b = np.asarray(block, dtype=np.float64)
    b = b[:len(b)//step*step].reshape(-1, step).mean(axis=1)     # average, not skip: no 8-24 kHz aliasing
    b = b-b.mean()
    energy = float(b@b)
    if energy <= 0.:
        return 0.
    r = rate/step
    lo, hi = int(r/400), min(int(r/80), len(b)-1)
    if hi <= lo:
        return 0.
    ac = np.correlate(b, b, "full")[len(b)-1:]
    return float(ac[lo:hi].max()/energy)


class Endpointer:
    """Push-to-talk endpointing: decides when to stop recording, never what to keep.

    The raw microphone also hears the Enter key, and the first version stopped on that click
    before the operator spoke (November 15, flight approach-voice-05). Speech is therefore
    recognized as started only after `start_voiced` voiced blocks among the last five loud ones;
    clicks and taps have no pitch (none of 25 Enter-click blocks reached 0.5; 44% of the
    operator's voice blocks did). Recording ends `silence_s` after the last loud block once speech
    started, or at the caller's time limit.

    audio() is the whole recording. The November 15 version returned only the part after a
    voiced start, so when the start was missed (November 16, approach-voice-06, on audio the
    capture had chopped up) the spoken word was thrown away before recognition saw it.

    Feed fixed-size blocks; feed() returns calibrating, waiting, speech or done. gated is True
    when most calibration samples were exactly zero: an operating-system noise gate is active.
    """

    def __init__(self, rate, calibrate_s=.3, silence_s=1., floor=.002, voiced=.5, start_voiced=2):
        self.rate, self.calibrate_s, self.silence_s, self.floor = rate, calibrate_s, silence_s, floor
        self.voiced_threshold, self.start_voiced = voiced, start_voiced
        self.blocks, self.levels, self.zero_share, self.recent = [], [], [], []
        self.noise = self.start = None
        self.voiced_s = self.silent_s = self.elapsed = 0.
        self.ignored = 0                      # loud unvoiced blocks before speech (clicks, taps)

    @property
    def gated(self):
        return bool(self.zero_share) and float(np.mean(self.zero_share)) > .5

    def feed(self, block):
        block = np.asarray(block, dtype=np.float32).reshape(-1)
        level = float(np.sqrt(np.mean(block**2))) if len(block) else 0.
        seconds = len(block)/self.rate
        self.blocks.append(block)
        self.elapsed += seconds
        if self.elapsed <= self.calibrate_s:
            self.levels.append(level)
            self.zero_share.append(float(np.mean(block == 0)) if len(block) else 1.)
            return "calibrating"
        if self.noise is None:
            # Low quantile: a word spoken during calibration must not raise the threshold.
            self.noise = min(.01, float(np.quantile(self.levels, .25))) if self.levels else 0.
        loud = level > max(self.floor, 3*self.noise)
        is_voiced = loud and voicing(block, self.rate) >= self.voiced_threshold
        if self.start is None:
            if loud:
                self.recent = (self.recent+[(len(self.blocks)-1, is_voiced)])[-5:]
                if sum(v for _, v in self.recent) >= self.start_voiced:
                    self.start = self.recent[0][0]
                    self.voiced_s = sum(v for _, v in self.recent)*seconds
                elif not is_voiced:
                    self.ignored += 1
            else:
                self.recent = []
            return "waiting" if self.start is None else "speech"
        if loud:
            self.silent_s = 0.
            self.voiced_s += seconds if is_voiced else 0.
        else:
            self.silent_s += seconds
            if self.silent_s >= self.silence_s:
                return "done"
        return "speech"

    @property
    def heard(self):
        """True once voiced speech started."""
        return self.start is not None

    def audio(self):
        """Everything fed so far, from the moment the stream opened."""
        return np.concatenate(self.blocks) if self.blocks else np.zeros(0, np.float32)


def capture_options(device=None):
    """Microphone streams to try, best first: [(device, rate, extra_settings, description)].

    Without an explicit device: WASAPI exclusive mode on the default microphone (raw, no
    Windows noise suppression), then the default device in shared mode.
    """
    import sounddevice as sd
    if device is not None:
        d = sd.query_devices(device)
        return [(device, int(d["default_samplerate"]), None, f"device {device}, shared")]
    options = []
    default = sd.query_devices(kind="input")
    apis = sd.query_hostapis()
    wasapi = next((i for i, a in enumerate(apis) if "WASAPI" in a["name"]), None)
    if wasapi is not None and hasattr(sd, "WasapiSettings"):
        for i, d in enumerate(sd.query_devices()):
            if d["hostapi"] == wasapi and d["max_input_channels"] > 0 and d["name"][:16] == default["name"][:16]:
                options.append((i, int(d["default_samplerate"]), sd.WasapiSettings(exclusive=True),
                                "WASAPI exclusive (raw microphone)"))
    options.append((None, int(default["default_samplerate"]), None,
                    "shared (Windows noise suppression may cut the start of words)"))
    return options


def record_utterance(device=None, max_s=6., echo=print):
    """Record one utterance: (the whole recording as mono float32 at 16 kHz, capture info).

    The stream delivers its blocks through a callback. Blocking reads in WASAPI exclusive mode
    silently lost about a third of the samples (November 16 room capture: 1.32 s of audio in 2 s,
    no overflow flag, a jump at every read), which chopped a voice into pieces without pitch.
    info records captured_s against wall_s so a loss like that shows in the flight log.
    """
    import queue
    import sounddevice as sd
    for dev, rate, extra, description in capture_options(device):
        block, blocks, status = int(rate*.03), queue.Queue(), []

        def callback(indata, frames, timing, flags):
            if flags:
                status.append(str(flags))
            blocks.put(indata[:, 0].copy())
        try:
            stream = sd.InputStream(samplerate=rate, channels=1, dtype="float32", blocksize=block, device=dev,
                                    extra_settings=extra, callback=callback)
        except sd.PortAudioError:
            continue
        end, told = Endpointer(rate), False
        with stream:
            start = time.monotonic()
            while end.elapsed < max_s and time.monotonic()-start < max_s+1.:
                try:
                    state = end.feed(blocks.get(timeout=.5))
                except queue.Empty:
                    continue
                if state != "calibrating" and not told:
                    if end.gated:
                        echo("  warning: the microphone input is noise-gated by Windows; see voice devices")
                    echo("  listening ... speak now")
                    told = True
                if state == "done":
                    break
            wall = time.monotonic()-start
        info = dict(capture=description, gated=end.gated, noise=round(end.noise or 0., 5),
                    ignored_clicks=end.ignored, voiced_s=round(end.voiced_s, 2), heard=end.heard,
                    captured_s=round(end.elapsed, 2), wall_s=round(wall, 2), stream_status=sorted(set(status)))
        if end.elapsed < .9*wall-.2:
            echo(f"  warning: the microphone stream lost audio ({end.elapsed:.1f} s recorded in {wall:.1f} s)")
        return resample(end.audio(), rate), info
    raise RuntimeError("No microphone stream could be opened")


# -- recognition -----------------------------------------------------------------------------------
class KeywordRecognizer:
    """Vosk grammar-mode recognition with an English and a Chinese model.

    Each model's grammar is the vocabulary phrases it knows plus [unk]. Words at confidence
    min_conf or more from both models are joined into the text that match_target reads.
    """

    def __init__(self, vocabulary, model_dir=None, min_conf=.5):
        from vosk import Model, SetLogLevel
        SetLogLevel(-1)
        self.min_conf, self.models = min_conf, {}
        for language, name in VOSK_MODELS.items():
            path = Path(model_dir or DEFAULT_MODEL_DIR)/name
            if not path.is_dir():
                raise RuntimeError(f"Speech model {path} is missing. While online, run: python -m objectpilot.voice download")
            model = Model(str(path))
            known = sorted({p for phrases in vocabulary.values() for p in phrases
                            if bool(CJK.search(p)) == (language == "zh")
                            and all(model.vosk_model_find_word(w) >= 0 for w in p.lower().split())})
            self.models[language] = (model, json.dumps(known+["[unk]"], ensure_ascii=False))

    def __call__(self, audio):
        """(text of confident target words, {language: [[word, confidence], ...]})."""
        from vosk import KaldiRecognizer
        peak = float(np.abs(audio).max()) if len(audio) else 0.
        gain = min(30., .5/peak) if peak > 0 else 1.
        padded = np.concatenate([np.zeros(SAMPLE_RATE//4), audio*gain, np.zeros(SAMPLE_RATE//2)])
        pcm = (np.clip(padded, -1, 1)*32767).astype("<i2").tobytes()
        words, detail = [], {}
        for language, (model, grammar) in self.models.items():
            recognizer = KaldiRecognizer(model, SAMPLE_RATE, grammar)
            recognizer.SetWords(True)
            recognizer.AcceptWaveform(pcm)
            result = json.loads(recognizer.FinalResult()).get("result", [])
            detail[language] = [[w["word"], round(w["conf"], 2)] for w in result]
            words += [w["word"] for w in result if w["word"] != "[unk]" and w["conf"] >= self.min_conf]
        return " ".join(words), detail


def download(model_dir=None, echo=print):
    """Fetch the English and Chinese Vosk models (about 40 MB each) unless present."""
    root = Path(model_dir or DEFAULT_MODEL_DIR)
    root.mkdir(parents=True, exist_ok=True)
    for name in VOSK_MODELS.values():
        if (root/name).is_dir():
            echo(f"{name}: present")
            continue
        archive = root/f"{name}.zip"
        echo(f"{name}: downloading {VOSK_URL.format(name)}")
        urllib.request.urlretrieve(VOSK_URL.format(name), archive)
        with zipfile.ZipFile(archive) as z:
            z.extractall(root)
        archive.unlink()
        echo(f"{name}: {root/name}")
    return root


def resolve_by_confidence(detail, vocabulary, margin=.25):
    """When the two language models name different labels, the label whose words they heard
    with clearly more confidence (summed, at least `margin` ahead), else None."""
    totals = {}
    for words in detail.values():
        for word, confidence in words:
            label = match_target(word, vocabulary).label if word != "[unk]" else None
            if label:
                totals[label] = totals.get(label, 0.)+confidence
    ranked = sorted(totals.items(), key=lambda kv: -kv[1])
    if ranked and (len(ranked) == 1 or ranked[0][1]-ranked[1][1] >= margin):
        return ranked[0][0]
    return None


# -- dialogue ---------------------------------------------------------------------------------------
def ask_target(vocabulary, listen, recognize, input_fn=input, echo=print, attempts=3):
    """Ask until the operator names one target: Enter then speak, or type it.

    listen() returns (16 kHz audio or None, capture info); recognize(audio) returns (text, detail).
    The recording is recognized even when the endpointer heard no voiced speech (info heard
    False): the grammar recognizer, not the pitch test, decides whether a target word was said.
    Returns (label or None, attempts as dicts; voice attempts carry their audio).
    """
    records = []
    for attempt in range(1, attempts+1):
        typed = input_fn(f"Press Enter and say the target ({examples(vocabulary)}), or type it: ").strip()
        record = dict(attempt=attempt, source="typed" if typed else "voice", text=typed or None)
        if not typed:
            audio, info = listen()
            record.update(info)
            if audio is None or len(audio) == 0:
                echo("  no audio recorded")
                records.append(dict(record, label=None, reason="no_audio"))
                continue
            if info.get("heard") is False:
                echo("  no clear voice detected; recognizing the whole recording anyway")
            record["audio"] = audio
            record["text"], record["detail"] = recognize(audio)
        match = match_target(record["text"], vocabulary)
        if match.reason == "ambiguous" and record.get("detail"):
            label = resolve_by_confidence(record["detail"], vocabulary)
            if label:
                match = Match(label, "matched_by_confidence", match.phrases)
        records.append(dict(record, label=match.label, reason=match.reason, phrases=match.phrases))
        heard = record["text"] if typed else " | ".join(
            f"{language}: " + (" ".join(f"{w} {c:.2f}" for w, c in words) or "-")
            for language, words in record["detail"].items())
        echo(f"  heard: {heard} -> {match.label or match.reason}")
        if match.label:
            return match.label, records
    return None, records


def voice_chooser(vocabulary, model_dir=None, device=None, echo=print, input_fn=input):
    """A target chooser for the approach flight. Loads the models now, before any network use."""
    recognize = KeywordRecognizer(vocabulary, model_dir)
    return lambda: ask_target(vocabulary, lambda: record_utterance(device, echo=echo), recognize,
                              input_fn=input_fn, echo=echo)


# -- command line -----------------------------------------------------------------------------------
def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")      # Chinese Windows consoles use GBK
    parser = argparse.ArgumentParser(description="Spoken target selection (offline keyword recognition)")
    parser.add_argument("--vocabulary", default=str(DEFAULT_VOCABULARY))
    parser.add_argument("--models", default=str(DEFAULT_MODEL_DIR), help="folder with the Vosk models")
    sub = parser.add_subparsers(dest="mode", required=True)
    sub.add_parser("download", help="fetch the English and Chinese models while online")
    sub.add_parser("devices", help="list microphones and the capture path the recorder uses")
    sub.add_parser("words", help="print the vocabulary")
    parse = sub.add_parser("parse", help="match a typed sentence")
    parse.add_argument("text")
    listen = sub.add_parser("listen", help="microphone test without the aircraft")
    listen.add_argument("--mic", type=int, default=None, help="input device index (default: best path to the default mic)")
    listen.add_argument("--save", help="write the last utterance to this WAV file")
    clip = sub.add_parser("file", help="recognize a 16-bit WAV file")
    clip.add_argument("wav")
    a = parser.parse_args(argv)
    if a.mode == "download":
        download(a.models)
        return 0
    if a.mode == "devices":
        import sounddevice as sd
        apis = sd.query_hostapis()
        for i, d in enumerate(sd.query_devices()):
            if d["max_input_channels"] > 0:
                print(f"{i:>3} {apis[d['hostapi']]['name']:<20} {d['name']} ({int(d['default_samplerate'])} Hz)")
        print("recorder tries, in order:", "; ".join(f"{o[3]} [{o[0]}]" for o in capture_options()))
        return 0
    vocabulary = load_vocabulary(a.vocabulary)
    if a.mode == "words":
        for label, phrases in vocabulary.items():
            print(f"{label}: {', '.join(phrases)}")
        return 0
    if a.mode == "parse":
        match = match_target(a.text, vocabulary)
        print(f"{match.label or match.reason} {match.phrases}")
        return 0 if match.label else 1
    recognize = KeywordRecognizer(vocabulary, a.models)
    if a.mode == "file":
        start = time.perf_counter()
        text, detail = recognize(load_wav(a.wav))
        match = match_target(text, vocabulary)
        print(f"{detail} [{time.perf_counter()-start:.2f} s] -> {match.label or match.reason}")
        return 0 if match.label else 1
    label, records = ask_target(vocabulary, lambda: record_utterance(a.mic), recognize)
    voiced = [r for r in records if "audio" in r]
    if voiced:
        last = voiced[-1]
        print(f"capture: {last['capture']}, noise floor {last['noise']}, gated {last['gated']}, "
              f"{last.get('captured_s')} s recorded in {last.get('wall_s')} s, voice detected {last.get('heard')}")
        if a.save:
            save_wav(a.save, voiced[-1]["audio"])
    print(f"target: {label}")
    return 0 if label else 1


if __name__ == "__main__":
    raise SystemExit(main())
