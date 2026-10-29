"""Generate printable marker/checkerboard images; print at measured physical size."""
from __future__ import annotations

import argparse
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--kind", choices=["aruco", "checkerboard"], default="aruco")
    p.add_argument("--output", default="assets/aruco_0.png")
    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--side-pixels", type=int, default=800, help="Black marker square edge in pixels")
    p.add_argument("--margin", type=int, default=100, help="White quiet margin, pixels")
    p.add_argument("--cols", type=int, default=9, help="Checkerboard inner corners across")
    p.add_argument("--rows", type=int, default=6, help="Checkerboard inner corners down")
    p.add_argument("--square-pixels", type=int, default=100)
    a = p.parse_args()
    import cv2
    import numpy as np
    if a.margin < 1 or a.side_pixels < 32 or not 0 <= a.marker_id < 50:
        p.error("Need margin>=1, side-pixels>=32, marker-id in 0..49")
    if a.cols < 2 or a.rows < 2 or a.square_pixels < 4:
        p.error("Invalid checkerboard dimensions")
    if a.kind == "aruco":
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        img = cv2.aruco.generateImageMarker(dictionary, a.marker_id, a.side_pixels)
    else:
        squares = (np.indices((a.rows + 1, a.cols + 1)).sum(axis=0) % 2 * 255).astype(np.uint8)
        img = np.repeat(np.repeat(squares, a.square_pixels, axis=0), a.square_pixels, axis=1)
    img = cv2.copyMakeBorder(img, a.margin, a.margin, a.margin, a.margin, cv2.BORDER_CONSTANT, value=255)
    path = Path(a.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), img):
        raise RuntimeError(f"Could not write {path}")
    print(f"Wrote {path.resolve()}. Print without stretching, mount flat, and measure the black marker edge / checker square in metres.")


if __name__ == "__main__":
    main()
