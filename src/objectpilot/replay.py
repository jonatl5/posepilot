"""Camera/file/Tello video -> shared 64-value pipeline -> dry RC and JSONL.

Tello mode enables video and optional range queries only. There is no takeoff,
landing, RC transmission or --fly option in this command.
"""
import argparse
import json
import math
import time
from pathlib import Path
from dataclasses import replace
import cv2
from .core import Config, RangeSample
from .pipeline import ApproachPipeline
from .vision import YOLODetector, MarkerDetector, TargetLock
from .evaluate import load_policy
from .video import LatestVideo


def read_ranges(path):
    rows = [RangeSample(**json.loads(line)) for line in Path(path).read_text().splitlines() if line.strip()]
    if any(not math.isfinite(r.timestamp) or r.timestamp < 0 for r in rows):
        raise ValueError("Range timestamps must be finite nonnegative video seconds")
    if any(a.timestamp > b.timestamp for a, b in zip(rows, rows[1:])):
        raise ValueError("Range timestamps must be sorted")
    return rows


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", required=True, help="Video path, camera index, synthetic, or tello")
    p.add_argument("--detector", choices=("yolo", "aruco", "red"), default="yolo")
    p.add_argument("--weights", default="yolo11n.pt")
    p.add_argument("--class-name", default="bottle")
    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--model", help="Optional PPO policy.zip; defaults to PID")
    p.add_argument("--config", help="Complete JSON Config; must match PPO metadata")
    p.add_argument("--visual-scale", type=float, help="Per-print calibrated scale; not a universal distance model")
    p.add_argument("--range-jsonl", help="File-only range sidecar using video seconds")
    p.add_argument("--range-mode", choices=("none", "stock", "custom"), default="none")
    p.add_argument("--host", default="192.168.10.1")
    p.add_argument("--out", default="runs/replay.jsonl")
    p.add_argument("--show", action="store_true")
    p.add_argument("--max-frames", type=int, default=1000)
    p.add_argument("--seconds", type=float, default=0)
    a = p.parse_args(argv)
    live = a.source.isdigit() or a.source in ("synthetic", "tello")
    if a.max_frames < 1 or not math.isfinite(a.seconds) or a.seconds < 0:
        p.error("max-frames must be positive and seconds finite and nonnegative")
    if a.range_jsonl and live:
        p.error("Range sidecars use video time and require a file source")
    if a.range_mode != "none" and a.source != "tello":
        p.error("Range queries require --source tello")
    c = Config.from_json(a.config) if a.config else Config()
    if a.visual_scale is not None:
        c = replace(c, visual_scale_m=a.visual_scale)
    ranges = read_ranges(a.range_jsonl) if a.range_jsonl else []
    output = Path(a.out)
    if output.exists():
        p.error("Output exists; choose a new log path")
    detector = (YOLODetector(a.weights, a.class_name, c.confidence_min) if a.detector == "yolo"
                else MarkerDetector(a.detector, a.marker_id))
    pipeline = ApproachPipeline(c, load_policy(a.model, c) if a.model else None)
    lock = TargetLock()
    link = video = cap = None
    count = 0
    try:
        if a.source == "tello":
            from .ground_link import GroundLink
            link = GroundLink(a.host, config=c)
            link.connect(a.range_mode)
        if live:
            video = LatestVideo(a.source, a.detector, a.marker_id).start()
        else:
            cap = cv2.VideoCapture(a.source)
            if not cap.isOpened():
                raise RuntimeError("Cannot open video")
        fps = cap.get(cv2.CAP_PROP_FPS) if cap else None
        if cap and (not math.isfinite(fps) or fps <= 0):
            raise ValueError("Video FPS must be known for timestamped replay")
        seen = -1
        video_index = 0
        range_index = -1
        last_processed = -math.inf
        started = time.monotonic()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x", encoding="utf-8") as log:
            while count < a.max_frames:
                if a.seconds and time.monotonic()-started >= a.seconds:
                    break
                if video:
                    if video.error:
                        raise RuntimeError(video.error)
                    sample = video.latest()
                    if sample is None or sample.frame_id == seen:
                        if video.finished:
                            break
                        time.sleep(.005)
                        continue
                    frame_id, timestamp, frame = sample.frame_id, sample.captured_at, sample.image.copy()
                    if timestamp-last_processed < c.dt:
                        time.sleep(.005)
                        continue
                else:
                    ok, frame = cap.read()
                    if not ok:
                        break
                    frame_id, timestamp = video_index, video_index/fps
                    video_index += 1
                    if timestamp-last_processed < c.dt-1e-6:
                        continue
                seen, last_processed = frame_id, timestamp
                start = time.perf_counter()
                candidates = detector.detect(frame, timestamp, frame_id)
                detection = lock.select(candidates, frame_id)
                inference_s = time.perf_counter()-start
                while range_index+1 < len(ranges) and ranges[range_index+1].timestamp <= timestamp:
                    range_index += 1
                ranging = link.range_sample if link else ranges[range_index] if range_index >= 0 else None
                now = time.monotonic() if live else timestamp
                row = pipeline.step(detection, ranging, now)
                row.update(frame_id=frame_id, frame_timestamp=timestamp, source=a.source,
                           timestamp_kind="host_decode_receipt" if live else "video_timeline",
                           inference_s=inference_s, detector=a.detector)
                log.write(json.dumps(row, allow_nan=False)+"\n")
                log.flush()
                count += 1
                if a.show:
                    h, w = frame.shape[:2]
                    for d in candidates:
                        cv2.rectangle(frame, (int((d.cx-d.width/2)*w), int((d.cy-d.height/2)*h)),
                                      (int((d.cx+d.width/2)*w), int((d.cy+d.height/2)*h)), (0, 180, 0), 2)
                    cv2.putText(frame, f"DRY RUN {row['rc']} {row['supervisor']}", (15, 30), 0, .55, (0, 0, 255), 2)
                    cv2.imshow("objectPilot", frame)
                    if cv2.waitKey(1) & 255 in (27, ord('q')):
                        break
    finally:
        if video:
            video.close()
        if cap:
            cap.release()
        if link:
            link.close()
        if a.show:
            cv2.destroyAllWindows()
    print(f"{count} samples; {output}; no flight commands sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
