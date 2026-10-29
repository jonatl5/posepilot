"""YOLO adapter and conservative same-class association; no person/pose path."""
import math
from .core import Detection

def iou(a,b):
    def corners(d): return (d.cx-d.width/2,d.cy-d.height/2,d.cx+d.width/2,d.cy+d.height/2)
    ax,ay,ar,ab = corners(a); bx,by,br,bb = corners(b)
    intersection = max(0,min(ar,br)-max(ax,bx))*max(0,min(ab,bb)-max(ay,by))
    return intersection/max(1e-9,a.width*a.height+b.width*b.height-intersection)

class TargetLock:
    def __init__(self, confirm_frames=3):
        self.last = None
        self.count = 0
        self.confirm_frames = confirm_frames
        self.last_frame = None

    def select(self, candidates, frame_id):
        if frame_id == self.last_frame: return None
        self.last_frame = frame_id
        if not candidates:
            self.last = None; self.count = 0
            return None
        if self.last is None:
            chosen = max(candidates,key=lambda d:(d.confidence,-abs(d.cx-.5)))
            self.count = 1
        else:
            chosen = max(candidates,key=lambda d:iou(d,self.last))
            close = (iou(chosen,self.last)>.2 and chosen.label==self.last.label
                     and math.hypot(chosen.cx-self.last.cx,chosen.cy-self.last.cy)<.2)
            self.count = self.count+1 if close else 1
        self.last = chosen
        return chosen if self.count >= self.confirm_frames else None

class YOLODetector:
    def __init__(self, weights="yolo11n.pt", label="bottle", confidence=.45):
        from ultralytics import YOLO
        self.model = YOLO(weights)
        self.label, self.confidence = label, confidence
        if label not in self.model.names.values():
            raise ValueError(f"Unknown class {label}; choose from {self.model.names}")

    def detect(self, frame, timestamp, frame_id):
        result = self.model.predict(frame,imgsz=640,conf=self.confidence,device="cpu",verbose=False)[0]
        detections = []
        for box in result.boxes:
            label = result.names[int(box.cls.item())]
            if label == self.label:
                cx,cy,w,h = box.xywhn[0].cpu().tolist()
                detections.append(Detection(cx,cy,w,h,float(box.conf.item()),timestamp,frame_id,label))
        return detections
