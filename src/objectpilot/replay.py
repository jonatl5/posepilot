"""Camera/file -> real detector -> shared state -> controller -> dry RC + JSONL.

No sockets, takeoff or RC transmission are available in this entry point.
"""
import argparse
import json
import time
import threading
from dataclasses import asdict
from pathlib import Path
import cv2
import numpy as np
from .core import Config, FeatureBuilder, PID, Supervisor, RangeSample, rc_channels, SCHEMA
from .vision import YOLODetector, TargetLock
from .evaluate import load_policy

class LatestCamera:
    def __init__(self, index):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened(): raise RuntimeError("Cannot open camera")
        self.sample = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._read,daemon=True)
        self.thread.start()

    def _read(self):
        index = 0
        while not self.stop.is_set():
            ok,frame = self.cap.read()
            if not ok: break
            with self.lock: self.sample = (index,time.monotonic(),frame)
            index += 1

    def read(self):
        with self.lock: return self.sample

    def close(self):
        self.stop.set(); self.cap.release(); self.thread.join(timeout=2)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source",required=True,help="Video path or camera index, e.g. 0")
    p.add_argument("--weights",default="yolo11n.pt")
    p.add_argument("--class-name",default="bottle")
    p.add_argument("--model",help="Optional PPO policy.zip (otherwise PID)")
    p.add_argument("--range-jsonl",help="Replay range sidecar: timestamp (video seconds), distance_m, status, source")
    p.add_argument("--visual-scale",type=float,default=.22)
    p.add_argument("--out",default="runs/replay.jsonl")
    p.add_argument("--show",action="store_true")
    p.add_argument("--max-frames",type=int,default=1000)
    a = p.parse_args()
    c = Config(visual_scale_m=a.visual_scale)
    detector, lock = YOLODetector(a.weights,a.class_name), TargetLock()
    builder, supervisor, pid = FeatureBuilder(c), Supervisor(c), PID(c)
    policy = load_policy(a.model,c) if a.model else None
    camera = LatestCamera(int(a.source)) if a.source.isdigit() else None
    cap = None if camera else cv2.VideoCapture(a.source)
    if cap is not None and not cap.isOpened(): raise RuntimeError("Cannot open video")
    if camera and a.range_jsonl: raise ValueError("Range sidecar uses video time and cannot be attached to live camera")
    ranges = []
    if a.range_jsonl:
        ranges = [RangeSample(**json.loads(line)) for line in Path(a.range_jsonl).read_text().splitlines() if line.strip()]
        if any(ranges[i].timestamp > ranges[i+1].timestamp for i in range(len(ranges)-1)):
            raise ValueError("Range timestamps must be sorted")
    output = Path(a.out); output.parent.mkdir(parents=True,exist_ok=True)
    previous = np.zeros(2); seen = -1; count = 0; range_index = -1
    video_index = 0; last_processed_time = -100.
    fps = (cap.get(cv2.CAP_PROP_FPS) or 30) if cap else 30
    try:
        with output.open("x",encoding="utf-8") as log:
            while count < a.max_frames:
                if camera:
                    sample = camera.read()
                    if sample is None or sample[0]==seen:
                        if not camera.thread.is_alive(): break
                        time.sleep(.005); continue
                    frame_id, timestamp, frame = sample
                    if timestamp-last_processed_time < c.dt:
                        time.sleep(.005); continue
                    now = time.monotonic()
                else:
                    ok,frame = cap.read()
                    if not ok: break
                    frame_id = video_index; timestamp = video_index/fps; now = timestamp
                    video_index += 1
                    if timestamp-last_processed_time < c.dt-1e-6: continue
                last_processed_time = timestamp
                seen = frame_id
                start = time.perf_counter()
                candidates = detector.detect(frame,timestamp,frame_id)
                detection = lock.select(candidates,frame_id)
                duration = time.perf_counter()-start
                if camera: now = time.monotonic()
                while range_index+1 < len(ranges) and ranges[range_index+1].timestamp <= timestamp:
                    range_index += 1
                ranging = ranges[range_index] if range_index >= 0 else None
                obs = builder.build(detection,ranging,now,previous)
                proposed = pid.predict(obs) if policy is None else policy.predict(obs,deterministic=True)[0]
                previous,reason = supervisor.apply(proposed,obs)
                row = {"schema":SCHEMA,"mode":"dry_run","frame_id":frame_id,"timestamp":timestamp,
                       "timestamp_kind":"host_decode_receipt" if camera else "video_timeline",
                       "source":a.source,"detection":asdict(detection) if detection else None,
                       "forward_range":asdict(ranging) if ranging else None,"observation":obs.tolist(),
                       "proposed_action":proposed.tolist(),"safe_action":previous.tolist(),
                       "rc":rc_channels(previous,c),"supervisor":reason,"inference_s":duration}
                log.write(json.dumps(row)+"\n"); log.flush()
                if a.show:
                    for d in candidates:
                        h,w = frame.shape[:2]
                        cv2.rectangle(frame,(int((d.cx-d.width/2)*w),int((d.cy-d.height/2)*h)),
                                      (int((d.cx+d.width/2)*w),int((d.cy+d.height/2)*h)),(0,180,0),2)
                    cv2.putText(frame,f"DRY RUN {row['rc']} {reason}",(15,30),0,.6,(0,0,255),2)
                    cv2.imshow("ObjectPilot",frame)
                    if cv2.waitKey(1)&255 in (27,ord('q')): break
                count += 1
    finally:
        camera.close() if camera else cap.release()
        cv2.destroyAllWindows()
    print(f"{count} frames; {output}; no flight commands sent")

if __name__ == "__main__": main()
