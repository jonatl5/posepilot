"""Collect unlabelled bench frames from a webcam or grounded Tello camera."""
import argparse
import csv
import math
from pathlib import Path
import time
import cv2
from .data import split_for
from .video import LatestVideo


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', default='0', help='Camera index or tello; no flight commands')
    p.add_argument('--seconds', type=float, default=20.)
    p.add_argument('--fps', type=float, default=2.)
    p.add_argument('--out', required=True)
    p.add_argument('--group-id', required=True)
    p.add_argument('--print-id', required=True)
    p.add_argument('--split', choices=('train', 'val', 'test', 'calibration'))
    a = p.parse_args(argv)
    if not (a.source.isdigit() or a.source == 'tello'):
        p.error('Use a camera index or tello; extract existing videos with objectpilot.data')
    if not all(math.isfinite(v) and v > 0 for v in (a.seconds, a.fps)):
        p.error('seconds and fps must be finite and positive')
    if not a.group_id.strip() or not a.print_id.strip():
        p.error('group-id and print-id are required')
    out = Path(a.out)
    if out.exists():
        p.error('Choose a new output directory to preserve earlier sessions')
    link = video = None
    records = []
    try:
        if a.source == 'tello':
            from .ground_link import GroundLink
            link = GroundLink()
            link.connect()
        video = LatestVideo(a.source).start()
        out.mkdir(parents=True, exist_ok=False)
        start = time.monotonic()
        last = -math.inf
        while time.monotonic()-start < a.seconds:
            if video.error:
                raise RuntimeError(video.error)
            frame = video.latest()
            if frame is not None and frame.captured_at-last >= 1/a.fps:
                path = out/f'frame_{frame.frame_id:06d}.jpg'
                if not cv2.imwrite(str(path), frame.image):
                    raise OSError(f'Could not write {path}')
                records.append(dict(image=str(path.resolve()), label_path='', label_reviewed='no',
                                    group_id=a.group_id, print_id=a.print_id,
                                    split=a.split or split_for(a.group_id), frame_id=frame.frame_id,
                                    timestamp=frame.captured_at, timestamp_kind='host_decode_receipt'))
                last = frame.captured_at
            if video.finished:
                break
            time.sleep(.005)
    finally:
        if video:
            video.close()
        if link:
            link.close()
        if records:
            with (out/'frames.csv').open('x', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=records[0])
                writer.writeheader()
                writer.writerows(records)
    if not records:
        raise RuntimeError('No frames collected')
    print(f'{len(records)} frames; annotate and review labels before dataset preparation')


if __name__ == '__main__':
    main()
