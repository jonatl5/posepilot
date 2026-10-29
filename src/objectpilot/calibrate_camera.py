"""Calibrate a fixed-resolution camera from varied checkerboard images."""
from __future__ import annotations

import argparse
import glob
import json
import math
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--images", required=True, help="Quoted glob, e.g. 'data/calibration/*.jpg'")
    p.add_argument("--cols", type=int, default=9, help="Internal checkerboard corners across")
    p.add_argument("--rows", type=int, default=6, help="Internal checkerboard corners down")
    p.add_argument("--square-m", type=float, required=True, help="Measured edge of one printed square, metres")
    p.add_argument("--output", default="configs/camera.json")
    a = p.parse_args()
    if a.cols < 2 or a.rows < 2 or not math.isfinite(a.square_m) or a.square_m <= 0:
        p.error("Invalid board dimensions")
    import cv2
    import numpy as np
    points = np.zeros((a.rows * a.cols, 3), np.float32)
    points[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square_m
    obj, image_points, used = [], [], []
    image_size = None
    for name in sorted(glob.glob(a.images)):
        img = cv2.imread(name)
        if img is None:
            print(f"Skip unreadable: {name}")
            continue
        size = img.shape[1], img.shape[0]
        if image_size is not None and size != image_size:
            raise ValueError("Mixed image resolutions; use a separate calibration per resolution/camera")
        image_size = size
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCornersSB(gray, (a.cols, a.rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            obj.append(points.copy())
            image_points.append(corners)
            used.append(name)
    if len(obj) < 10:
        raise ValueError(f"Only {len(obj)} valid views. Collect >=10 (prefer 20-30) varied positions/angles, including edges")
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(obj, image_points, image_size, None, None)
    per_view = []
    for object_pts, img_pts, rv, tv in zip(obj, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(object_pts, rv, tv, matrix, distortion)
        per_view.append(float(np.sqrt(np.mean(np.sum((img_pts - projected) ** 2, axis=2)))))
    if not np.isfinite(rms) or not np.isfinite(matrix).all():
        raise ValueError("Calibration failed; inspect images")
    result = {"camera_matrix": matrix.tolist(), "distortion": distortion.flatten().tolist(),
              "image_size": list(image_size), "rms_px": float(rms), "square_m": a.square_m,
              "inner_corners": [a.cols, a.rows], "images": used, "per_view_rms_px": per_view}
    path = Path(a.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2) + "\n")
    print(f"Saved {path.resolve()}; RMS={rms:.3f}px, {len(used)} views.")
    print("Verify known marker distance at several measured distances before flight. RMS alone does not validate metric accuracy.")
    if rms > 1.0:
        print("WARNING: RMS exceeds 1 px; review worst views and capture more varied sharp views.")


if __name__ == "__main__":
    main()
