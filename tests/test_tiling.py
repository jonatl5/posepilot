import numpy as np
import pytest

from objectpilot.core import Detection
from objectpilot.vision import detect_tiled, tile_boxes


def test_tiles_overlap_and_cover_the_frame():
    tiles = tile_boxes(960, 720)
    assert tiles == [(0, 0, 640, 480), (320, 0, 960, 480), (0, 240, 640, 720), (320, 240, 960, 720)]
    # Any box smaller than a third of the frame lies whole inside one tile.
    for cx, cy in ((.1, .1), (.5, .5), (.9, .2), (.33, .66)):
        x0, x1, y0, y1 = (cx-.16)*960, (cx+.16)*960, (cy-.16)*720, (cy+.16)*720
        assert any(a <= max(x0, 0) and min(x1, 960) <= c and b <= max(y0, 0) and min(y1, 720) <= d
                   for a, b, c, d in tiles)


class CenterDetector:
    """Reports one box at the center of whatever image it is given."""

    def detect(self, frame, timestamp, frame_id):
        return [Detection(.5, .5, .1, .1, .5, timestamp, frame_id, "cup")]


def test_tile_detections_map_back_to_full_frame_coordinates():
    found = detect_tiled(CenterDetector(), np.zeros((720, 960, 3), np.uint8), 0., 1)
    centers = sorted((round(d.cx, 4), round(d.cy, 4)) for d in found)
    assert centers == sorted((round(x, 4), round(y, 4)) for x, y in [(.5, .5), (1/3, 1/3), (2/3, 1/3), (1/3, 2/3), (2/3, 2/3)])
    assert all(d.width == pytest.approx(.1*640/960) and d.height == pytest.approx(.1*480/720) for d in found[1:])
