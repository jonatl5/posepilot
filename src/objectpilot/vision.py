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


class MarkerDetector:
    """Training-free bench targets using the same normalized Detection contract.

    Markers exercise the pipeline; the demonstration target remains a wall print
    detected by YOLO. They do not introduce a second controller or policy schema.
    """
    def __init__(self, kind="aruco", marker_id=0):
        import cv2
        if kind not in ("aruco", "red") or not 0 <= marker_id < 50:
            raise ValueError("Use red/aruco and a DICT_4X4_50 marker ID")
        self.kind, self.marker_id = kind, marker_id
        if kind == "aruco":
            self.aruco = cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

    def detect(self, frame, timestamp, frame_id):
        import cv2
        import numpy as np
        h, w = frame.shape[:2]
        boxes = []
        if self.kind == "aruco":
            corners, ids, _ = self.aruco.detectMarkers(frame)
            if ids is not None:
                for corner, ident in zip(corners, ids.ravel()):
                    if int(ident) == self.marker_id:
                        points = corner.reshape(-1, 2)
                        lo, hi = points.min(axis=0), points.max(axis=0)
                        boxes.append((*lo, *(hi-lo), f"aruco:{self.marker_id}"))
        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 100, 70), (10, 255, 255))
            mask |= cv2.inRange(hsv, (170, 100, 70), (179, 255, 255))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) >= .001*w*h:
                    boxes.append((*cv2.boundingRect(contour), "red_marker"))
        return [Detection(float((x+bw/2)/w), float((y+bh/2)/h), float(bw/w), float(bh/h),
                          1., timestamp, frame_id, label)
                for x, y, bw, bh, label in boxes if bw > 0 and bh > 0]
