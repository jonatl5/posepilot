"""Latest-only video input with monotonically increasing frame IDs."""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Optional


@dataclass(frozen=True)
class Frame:
    image: Any
    captured_at: float  # PC decode/capture time, not drone exposure timestamp
    frame_id: int


class LatestVideo:
    def __init__(self, source="synthetic", detector="red", marker_id=0):
        self.source, self.detector, self.marker_id = source, detector, marker_id
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest: Optional[Frame] = None
        self.error: Optional[str] = None
        self.finished = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()
        return self

    def latest(self):
        with self._lock:
            return self._latest

    def _publish(self, image):
        with self._lock:
            frame_id = self._latest.frame_id + 1 if self._latest else 1
            self._latest = Frame(image, time.monotonic(), frame_id)

    def _run(self):
        try:
            if self.source == "synthetic":
                self._synthetic()
            elif self.source == "tello":
                self._tello()
            else:
                self._opencv()
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            self.finished = True

    def _synthetic(self):
        import cv2
        import numpy as np
        marker = None
        if self.detector == "aruco":
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            marker = cv2.aruco.generateImageMarker(dictionary, self.marker_id, 200)
        start = time.monotonic()
        while not self._stop.is_set():
            t = time.monotonic() - start
            image = np.full((480, 640, 3), 235, dtype=np.uint8)
            size = int(45 + 30 * (1 + math.sin(t * .5)))
            x = int(320 + 125 * math.sin(t * .7))
            y = 240
            if marker is None:
                cv2.rectangle(image, (x-size//2, y-size//2), (x+size//2, y+size//2), (0, 0, 230), -1)
            else:
                patch = cv2.resize(marker, (size, size), interpolation=cv2.INTER_NEAREST)
                image[y-size//2:y-size//2+size, x-size//2:x-size//2+size] = cv2.cvtColor(patch, cv2.COLOR_GRAY2BGR)
            self._publish(image)
            self._stop.wait(1/30)

    def _opencv(self):
        import cv2
        source = int(self.source) if str(self.source).isdigit() else self.source
        capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Cannot open camera/video {source!r}")
        is_file = not isinstance(source, int)
        fps = capture.get(cv2.CAP_PROP_FPS)
        period = 1 / fps if is_file and math.isfinite(fps) and fps > 0 else 0
        try:
            while not self._stop.is_set():
                start = time.monotonic()
                ok, image = capture.read()
                if not ok:
                    if not is_file:
                        raise RuntimeError("Camera stopped producing frames")
                    break
                self._publish(image)
                self._stop.wait(max(0, period - (time.monotonic()-start)))
        finally:
            capture.release()

    def _tello(self):
        import av
        # Tello video has no trustworthy PC-synchronized camera exposure time.
        # Keep decoding continuously; consumers never queue images for inference.
        url = "udp://0.0.0.0:11111?fifo_size=65536&overrun_nonfatal=1&timeout=3000000"
        with av.open(url, format="h264", mode="r", timeout=(5.0, 3.0),
                     options={"fflags": "nobuffer", "flags": "low_delay"}) as container:
            for frame in container.decode(video=0):
                if self._stop.is_set():
                    break
                self._publish(frame.to_ndarray(format="bgr24"))

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
