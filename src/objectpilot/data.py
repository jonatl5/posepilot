"""Local video extraction, leakage-safe dataset assembly, and detector training."""
import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path
import cv2

def split_for(group):
    bucket = int(hashlib.sha256(group.encode()).hexdigest()[:8],16)%100
    return "train" if bucket<70 else "val" if bucket<85 else "test"

def extract(args):
    manifest = list(csv.DictReader(Path(args.manifest).open(encoding="utf-8-sig")))
    root = Path(args.out); root.mkdir(parents=True,exist_ok=False)
    records = []
    groups = {}
    for row in manifest:
        # All clips from the same room/session/print family must share group_id.
        group = row["group_id"]
        split = row.get("split") or split_for(group)
        if split not in ("train","val","test"): raise ValueError("Unknown split")
        if group in groups and groups[group]!=split: raise ValueError(f"Group leakage: {group}")
        groups[group]=split
        clip_id = row["clip_id"]
        if not clip_id or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for ch in clip_id):
            raise ValueError("Use simple unique clip IDs")
        cap = cv2.VideoCapture(row["path"])
        if not cap.isOpened(): raise ValueError(f"Cannot open {row['path']}")
        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps<=0: raise ValueError("Video FPS unknown")
        stride = max(1,round(fps/args.fps)); index = 0
        try:
            while True:
                ok,frame = cap.read()
                if not ok: break
                if index%stride == 0:
                    name = f"{clip_id}_{index:06d}.jpg"
                    target = root/name
                    if target.exists(): raise ValueError(f"Duplicate clip/frame {name}")
                    if not cv2.imwrite(str(target),frame): raise IOError(str(target))
                    records.append({**row,"image":str(target.resolve()),"frame_index":index,
                                    "video_time_s":index/fps,"split":split,"label_reviewed":"no",
                                    "label_path":""})
                index += 1
        finally: cap.release()
    if not records: raise ValueError("No frames extracted")
    with (root/"frames.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=records[0].keys()); w.writeheader(); w.writerows(records)
    print(f"Extracted {len(records)} frames. Annotate YOLO boxes, review every label, update frames.csv.")

def prepare(args):
    rows = list(csv.DictReader(Path(args.frames).open(encoding="utf-8-sig")))
    root = Path(args.out)
    groups, images, counts = {},set(),{s:0 for s in ("train","val","test")}
    labels = [n.strip() for n in args.classes.split(",")]
    for row in rows:
        if row["label_reviewed"].lower() != "yes": raise ValueError("Unreviewed label: "+row["image"])
        split = row["split"]; group = row["group_id"]
        if split not in counts: raise ValueError("Invalid split")
        if group in groups and groups[group]!=split: raise ValueError("Group leakage")
        groups[group]=split
        image, label = Path(row["image"]),Path(row["label_path"])
        if not image.is_file() or not label.is_file(): raise ValueError("Image or explicit label file missing")
        digest=hashlib.sha256(image.read_bytes()).hexdigest()
        if digest in images: raise ValueError("Duplicate image content; deduplicate before splitting")
        images.add(digest)
        for line in label.read_text().splitlines():
            values = list(map(float,line.split()))
            if len(values)!=5: raise ValueError("Expected YOLO class cx cy width height")
            k,x,y,w,h=values
            if not(k.is_integer() and 0<=k<len(labels) and 0<=x<=1 and 0<=y<=1 and 0<w<=1 and 0<h<=1):
                raise ValueError(f"Invalid YOLO label {label}")
            if x-w/2 < -1e-5 or x+w/2>1.00001 or y-h/2 < -1e-5 or y+h/2>1.00001:
                raise ValueError(f"Box extends outside image: {label}")
        counts[split]+=1
    if not all(counts.values()): raise ValueError(f"Need nonempty train/val/test groups: {counts}")
    root.mkdir(parents=True,exist_ok=False)
    for split in counts:
        (root/"images"/split).mkdir(parents=True); (root/"labels"/split).mkdir(parents=True)
    for i,row in enumerate(rows):
        image=Path(row["image"]); split=row["split"]
        name=f"{i:06d}_{image.stem}"
        shutil.copyfile(image,root/"images"/split/(name+image.suffix))
        shutil.copyfile(row["label_path"],root/"labels"/split/(name+".txt"))
    # JSON is valid YAML, avoiding ambiguous escaping of Windows paths.
    config={"path":root.resolve().as_posix(),"train":"images/train","val":"images/val","test":"images/test","names":labels}
    (root/"dataset.yaml").write_text(json.dumps(config,indent=2),encoding="utf-8")
    shutil.copyfile(args.frames,root/"source_frames.csv")
    print(counts)

def main():
    p=argparse.ArgumentParser(); sub=p.add_subparsers(dest="command",required=True)
    e=sub.add_parser("extract"); e.add_argument("--manifest",required=True); e.add_argument("--out",required=True); e.add_argument("--fps",type=float,default=2)
    d=sub.add_parser("prepare"); d.add_argument("--frames",required=True); d.add_argument("--out",required=True); d.add_argument("--classes",required=True)
    t=sub.add_parser("train"); t.add_argument("--data",required=True); t.add_argument("--weights",default="yolo11n.pt"); t.add_argument("--epochs",type=int,default=50); t.add_argument("--device",default="cpu")
    c=sub.add_parser("calibrate"); c.add_argument("--csv",required=True,help="distance_m,box_height_fraction,split columns; calibration rows only")
    a=p.parse_args()
    if a.command=="extract":
        if a.fps<=0: p.error("fps must be positive")
        extract(a)
    elif a.command=="prepare": prepare(a)
    elif a.command=="calibrate":
        import numpy as np
        rows=list(csv.DictReader(Path(a.csv).open()))
        if not rows or any(r["split"]!="calibration" for r in rows): raise ValueError("Use separate calibration data, never test data")
        values=[float(r["distance_m"])*float(r["box_height_fraction"]) for r in rows]
        if not all(np.isfinite(v) and v>0 for v in values): raise ValueError("Invalid calibration")
        print(json.dumps({"visual_scale_m":float(np.median(values)),"samples":len(values),"std_m":float(np.std(values))}))
    else:
        from ultralytics import YOLO
        model=YOLO(a.weights)
        model.train(data=a.data,epochs=a.epochs,imgsz=640,seed=0,device=a.device,
                    project="runs/detector",name="printed_targets",exist_ok=False,workers=0)
        print("Select using validation only. Run final test once after freezing model and confidence threshold.")

if __name__ == "__main__": main()
