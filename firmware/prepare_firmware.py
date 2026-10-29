"""Generate a reviewable firmware sketch from the inspected official source.

Usage: python firmware/prepare_firmware.py --upstream artifacts/references/RMTT_Libs --out artifacts/firmware/objectpilot_default
Never compiles, flashes, installs board packages or modifies the upstream checkout.
"""
import argparse
import shutil
from pathlib import Path

def build(upstream, out):
    source=(Path(upstream)/"examples/default/default.ino").read_text(encoding="utf-8")
    edits = [
        ('#define SDK_VERSION "esp32v1.0.0.11"', '#include "range_snapshot.h"\n#define SDK_VERSION "objectpilot-bench-v0.1"'),
        ('    shell_cmd_init();', '    shell_cmd_init();\n    cmd_register((char*)"oprange?", op_range_callback);'),
        ('        range_ori = tt_tof.ReadRangeContinuousMillimeters();', '        range_ori = tt_tof.ReadRangeContinuousMillimeters();\n        op_record_range((uint16_t)range_ori);'),
    ]
    for old,new in edits:
        if source.count(old)!=1: raise ValueError("Upstream changed; review integration anchor: "+old)
        source=source.replace(old,new)
    out=Path(out); out.mkdir(parents=True,exist_ok=False)
    (out/(out.name+".ino")).write_text(source,encoding="utf-8")
    shutil.copyfile(Path(__file__).with_name("range_snapshot.h"),out/"range_snapshot.h")
    print(f"Generated {out}; Arduino/TT compilation and hardware checks still required.")

if __name__=="__main__":
    p=argparse.ArgumentParser(); p.add_argument("--upstream",required=True); p.add_argument("--out",required=True)
    a=p.parse_args(); build(a.upstream,a.out)
