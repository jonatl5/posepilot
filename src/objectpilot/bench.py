"""Props-off forward-ToF SDK bring-up. Never sends takeoff, motoron or rc."""
import argparse
import json
import time
from dataclasses import asdict
from pathlib import Path
from .core import parse_ext_tof, CustomRangeParser

class TelloBench:
    def __init__(self, custom=False):
        from djitellopy import Tello
        self.tello = Tello()
        self.custom = custom
        self.parser = CustomRangeParser()

    def connect(self):
        self.tello.connect()
        return self.tello.send_read_command("EXT version?")

    def front_range(self):
        start = time.monotonic()
        response = self.tello.send_read_command("EXT oprange?" if self.custom else "EXT tof?")
        # Conservative host request timestamp; firmware sample age remains unknown.
        end = time.monotonic()
        sample = self.parser.parse(response,start,end) if self.custom else parse_ext_tof(response,start)
        return sample,end-start

    def close(self): self.tello.end()

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seconds",type=float,default=15)
    p.add_argument("--out",default="runs/tof_bench.jsonl")
    p.add_argument("--custom",action="store_true",help="Only after custom ESP32 firmware/routing is verified")
    a = p.parse_args()
    out = Path(a.out); out.parent.mkdir(parents=True,exist_ok=True)
    bench = TelloBench(a.custom)
    try:
        version = bench.connect(); print(version)
        deadline = time.monotonic()+a.seconds
        with out.open("x",encoding="utf-8") as f:
            while time.monotonic()<deadline:
                sample,rtt = bench.front_range()
                f.write(json.dumps({**asdict(sample),"query_rtt_s":rtt,"firmware":version,
                                    "capture_age_known":a.custom})+"\n"); f.flush()
                time.sleep(.2)
    finally: bench.close()

if __name__ == "__main__": main()
