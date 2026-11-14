"""Geometric test world for the stop-and-go planner. NOT a validated flight model.

The aircraft is assumed to hover between commands, so a command changes the pose
and the pose fixes the target box through a pinhole camera. The camera is fitted
to the November 5-7 TT cup stand clips (height x distance = 0.328 + 0.018 margin,
width x distance about 0.19, 10.5 degree downward tilt, 32 x 54 degree video FOV)
and the ToF model to the whiteboard bench. Command accuracy, detection versus
viewing angle and the bottle/bag print sizes are assumptions until the step calibration
flight (objectpilot.step_flight, research/STEP_FLIGHT.md) measures them; pass its
noise.json with --noise.

World frame: wall plane x = 0, aircraft at x < 0, y to the right when facing the
wall, z up. Heading 0 faces the wall; positive heading is clockwise (SDK cw).
"""
from dataclasses import dataclass, replace
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from .stepwise import StepConfig, StepPlanner, check_command, summarize

RESERVED_SEEDS = range(10000, 60000)   # PID/PPO diagnostic, tuning, validation and final test


@dataclass
class Camera:
    hfov_deg: float = 54.0
    vfov_deg: float = 32.0
    tilt_deg: float = 10.5
    box_margin: float = .018

    def project(self, pose, point):
        """(u, v, depth) of a world point, u to the right and v down; None behind the camera."""
        x, y, z, heading = pose
        rx, ry, rz = point[0]-x, point[1]-y, point[2]-z
        forward = rx*math.cos(heading)+ry*math.sin(heading)
        right = -rx*math.sin(heading)+ry*math.cos(heading)
        tilt = math.radians(self.tilt_deg)
        depth = forward*math.cos(tilt)-rz*math.sin(tilt)
        down = -forward*math.sin(tilt)-rz*math.cos(tilt)
        if depth < .05:
            return None
        return (.5+right/depth/(2*math.tan(math.radians(self.hfov_deg)/2)),
                .5+down/depth/(2*math.tan(math.radians(self.vfov_deg)/2)), depth)

    def box(self, pose, target):
        """Clipped YOLO-style box of a wall print: dict(cx, cy, w, h, visible, depth) or None."""
        corners = [self.project(pose, (0., target.y+sy*target.width/2, target.z+sz*target.height/2))
                   for sy in (-1, 1) for sz in (-1, 1)]
        if any(c is None for c in corners):
            return None
        m = self.box_margin/2
        u0, u1 = min(c[0] for c in corners)-m/2, max(c[0] for c in corners)+m/2
        v0, v1 = min(c[1] for c in corners)-m, max(c[1] for c in corners)+m
        cu0, cu1, cv0, cv1 = max(u0, 0.), min(u1, 1.), max(v0, 0.), min(v1, 1.)
        if cu1 <= cu0 or cv1 <= cv0:
            return None
        return dict(cx=(cu0+cu1)/2, cy=(cv0+cv1)/2, w=cu1-cu0, h=cv1-cv0,
                    visible=(cu1-cu0)*(cv1-cv0)/((u1-u0)*(v1-v0)),
                    depth=sum(c[2] for c in corners)/4)


@dataclass
class Print:
    label: str
    y: float
    z: float
    width: float
    height: float


def default_prints():
    # Cup from the stand fit (0.328 * 2 tan 16 deg tall, 0.19 * 2 tan 27 deg wide).
    # Bottle and bag sizes are placeholders until their prints are measured.
    return [Print("bottle", -1.0, 1.45, .09, .20), Print("cup", 0., 1.45, .194, .188),
            Print("bag", 1.0, 1.45, .17, .19)]


@dataclass
class Noise:
    move_scale_sd: float = .08      # assumed: VPS position hold over a textured floor
    move_cross_sd: float = .03
    yaw_scale_sd: float = .03
    yaw_sd_deg: float = 1.
    move_heading_sd_deg: float = 1.
    fail_prob: float = .02          # command answered with "error", no motion
    hover_sd_m: float = .005
    pixel_sd: float = .004
    size_sd: float = .02
    detect_scale: float = 1.        # multiplies per-frame detection probability
    tof_sd_m: float = .002          # plus 0.002 per metre (bench: 1.7 mm at 0.65 m, 3.1 mm at 1 m)
    tof_bias_m: float = .02         # bench: head-on readings about 2 cm long
    tof_oblique_excess: float = .35  # bench: 20 and 40 degree readings exceed D/cos
    tof_dropout: float = .01
    imu_drift_deg: float = .3       # assumed random walk of the state-stream yaw per command
    move_scale_bias: float = 0.     # mean move scale - 1 (November 13 flight: moves fall about 8% short)
    yaw_bias_deg: float = 0.        # fixed shortfall of every turn (November 13 flight: about 2 degrees)
    tof_obs_sd_m: float = 0.        # ToF error shared by one hover's replies (flight sweep: about 1 cm)


class StepWorld:
    def __init__(self, rng, pose, prints=None, camera=None, noise=None, frames=6, speed_cm_s=30):
        self.rng = rng
        self.x, self.y, self.z, self.heading = pose
        self.prints = prints or default_prints()
        self.camera = camera or Camera()
        self.n = noise or Noise()
        self.frames, self.speed = frames, speed_cm_s
        self.time = 0.
        self.min_clearance = -self.x
        self.imu_bias = 0.

    @property
    def pose(self):
        return (self.x, self.y, self.z, self.heading)

    def imu_yaw(self):
        """State-stream yaw: integer degrees in [-180, 180), with a slowly drifting bias."""
        return float(round((math.degrees(self.heading)+self.imu_bias+180) % 360-180))

    def detect_probability(self, target, box):
        view = math.degrees(math.atan2(abs(self.y-target.y), -self.x))
        p_range = float(np.interp(box["depth"], [2.8, 3.6], [.97, 0.]))     # flights: lost beyond ~2.8 m
        p_view = float(np.interp(view, [35., 65.], [1., 0.]))               # assumed, unmeasured
        p_cut = 1. if box["visible"] >= .85 else .7 if box["visible"] >= .6 else 0.
        return p_range*p_view*p_cut*self.n.detect_scale

    def observe(self):
        """One hover: per-frame detections for every print, and ToF replies."""
        r, n = self.rng, self.n
        frames = []
        for _ in range(self.frames):
            detections = []
            for target in self.prints:
                box = self.camera.box(self.pose, target)
                if box is None or r.random() >= self.detect_probability(target, box):
                    continue
                detections.append((target.label, box["cx"]+r.normal(0, n.pixel_sd), box["cy"]+r.normal(0, n.pixel_sd),
                                   box["w"]*(1+r.normal(0, n.size_sd)), box["h"]*(1+r.normal(0, n.size_sd)), .9))
            frames.append(detections)
        replies = []
        shared = r.normal(0, n.tof_obs_sd_m) if n.tof_obs_sd_m else 0.
        for _ in range(self.frames):
            cos = math.cos(self.heading)
            reading = None
            if cos > .05:
                ray = -self.x/cos
                reading = (ray*(1+n.tof_oblique_excess*(1/cos-1))+n.tof_bias_m+shared
                           + r.normal(0, n.tof_sd_m+.002*ray))
                if reading > 1.22 or r.random() < n.tof_dropout:
                    reading = None
            replies.append(reading)
        self.time += .6
        self._drift()
        return frames, replies

    def _drift(self):
        self.x += self.rng.normal(0, self.n.hover_sd_m)
        self.y += self.rng.normal(0, self.n.hover_sd_m)
        self.min_clearance = min(self.min_clearance, -self.x)

    def execute(self, command):
        verb, value = check_command(command)
        r, n = self.rng, self.n
        self.imu_bias += r.normal(0, n.imu_drift_deg)
        if r.random() < n.fail_prob:
            self.time += .5
            return "error"
        if verb in ("cw", "ccw"):
            turn = (math.radians(max(0., value*(1+r.normal(0, n.yaw_scale_sd))-n.yaw_bias_deg))
                    + math.radians(r.normal(0, n.yaw_sd_deg)))
            self.heading += turn if verb == "cw" else -turn
            self.time += value/75+.8
            self._drift()
            return "ok"
        metres = value/100*(1+n.move_scale_bias+r.normal(0, n.move_scale_sd))
        cross = r.normal(0, .01+n.move_cross_sd*value/100)
        if verb in ("up", "down"):
            self.z += metres if verb == "up" else -metres
        else:
            fx, fy = math.cos(self.heading), math.sin(self.heading)
            rx, ry = -fy, fx
            along, side = {"forward": (metres, cross), "back": (-metres, cross),
                           "right": (cross, metres), "left": (cross, -metres)}[verb]
            self.x += along*fx+side*rx
            self.y += along*fy+side*ry
            self.heading += math.radians(r.normal(0, n.move_heading_sd_deg))
        self.time += value/self.speed+1.
        self.min_clearance = min(self.min_clearance, -self.x)
        return "ok"


def scenario(seed):
    """Start pose and requested label for one seed: 1.5-3.2 m from the print, up to 50 degrees off its normal."""
    r = np.random.default_rng(seed)
    prints = default_prints()
    target = prints[int(r.integers(len(prints)))]
    distance = float(r.uniform(1.5, 3.2))
    view = math.radians(float(r.uniform(-50, 50)))
    x, y = -distance*math.cos(view), target.y-distance*math.sin(view)
    bearing = math.atan2(target.y-y, -x)
    offset = math.radians(float(r.uniform(-180, 180) if r.random() < .4 else r.uniform(-25, 25)))
    return dict(seed=seed, label=target.label, target=target, distance_m=distance,
                view_deg=math.degrees(view), heading_offset_deg=math.degrees(offset),
                pose=(x, y, float(r.normal(1.60, .03)), bearing+offset))


def run_episode(seed, config=None, camera=None, noise=None, imu=True, keep_log=False):
    s = scenario(seed)
    world = StepWorld(np.random.default_rng(seed+1_000_000), s["pose"], camera=camera, noise=noise)
    planner = StepPlanner(replace(config or StepConfig(), label=s["label"]))
    log = []
    while True:
        frames, replies = world.observe()
        obs = summarize(frames, replies, s["label"], yaw_deg=world.imu_yaw() if imu else None)
        decision = planner.decide(obs)
        reply = None
        if decision.command and not decision.outcome:
            reply = world.execute(decision.command)
            planner.applied(decision.command, reply)
        if keep_log:
            log.append(dict(t=round(world.time, 1), phase=decision.phase, reason=decision.reason,
                            command=decision.command, reply=reply,
                            box=None if obs.box is None else [round(v, 3) for v in obs.box],
                            tof=None if obs.tof is None else round(obs.tof, 3),
                            x=round(world.x, 3), y=round(world.y, 3),
                            heading_deg=round(math.degrees(world.heading), 1)))
        if decision.outcome:
            break
    result = dict(seed=seed, label=s["label"], start_distance_m=round(s["distance_m"], 3),
                  start_view_deg=round(s["view_deg"], 1), start_heading_offset_deg=round(s["heading_offset_deg"], 1),
                  outcome=planner.outcome, commands=planner.commands, observations=planner.observations,
                  time_s=round(world.time, 1), **final_metrics(world, s["target"], planner.outcome),
                  squared=planner.squared)
    return (result, log) if keep_log else result


def final_metrics(world, target, outcome, stand_off_m=.65, minimum_m=.40):
    """True final pose against the print, and the success rule used for every result table."""
    heading_error = math.degrees(math.atan2(math.sin(world.heading), math.cos(world.heading)))
    metrics = dict(perpendicular_m=round(-world.x, 3), lateral_m=round(world.y-target.y, 3),
                   heading_error_deg=round(heading_error, 1), min_clearance_m=round(world.min_clearance, 3))
    metrics["success"] = bool(outcome == "arrived" and abs(metrics["perpendicular_m"]-stand_off_m) <= .08
                              and abs(metrics["lateral_m"]) <= .08 and abs(heading_error) <= 8
                              and world.min_clearance >= minimum_m)
    return metrics


VARIANTS = {
    "square": dict(),
    "square_no_imu": dict(imu=False),
    "no_square": dict(config=dict(square=False)),
    "fov_wrong_10pct": dict(camera=dict(hfov_deg=59.4, vfov_deg=35.2)),
    "steps_2x_error": dict(noise=dict(move_scale_sd=.16, move_cross_sd=.06, yaw_scale_sd=.06, yaw_sd_deg=2.)),
    "detections_70pct": dict(noise=dict(detect_scale=.7)),
    "tof_3x_noise": dict(noise=dict(tof_sd_m=.006, tof_oblique_excess=.7, tof_dropout=.05)),
}


def evaluate(variant, seeds, base_noise=None):
    """base_noise: Noise to start from, e.g. measured by objectpilot.step_flight; variants override it."""
    spec = VARIANTS[variant]
    config = replace(StepConfig(), **spec.get("config", {}))
    camera = replace(Camera(), **spec.get("camera", {}))
    noise = replace(base_noise or Noise(), **spec.get("noise", {}))
    return [run_episode(seed, config, camera, noise, imu=spec.get("imu", True)) for seed in seeds]


def summarize_results(rows, stand_off_m=.65):
    def median(key, subset):
        values = sorted(abs(r[key]) for r in subset)
        return round(values[len(values)//2], 3) if values else None
    arrived = [r for r in rows if r["outcome"] == "arrived"]
    by_view = {}
    for low, high in ((0, 15), (15, 30), (30, 50)):
        part = [r for r in rows if low <= abs(r["start_view_deg"]) < high]
        by_view[f"{low}-{high}deg"] = round(sum(r["success"] for r in part)/max(1, len(part)), 3)
    outcomes = {}
    for r in rows:
        outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0)+1
    return dict(episodes=len(rows), success=round(sum(r["success"] for r in rows)/len(rows), 3),
                outcomes=outcomes, success_by_start_view=by_view,
                arrived_median_abs_heading_error_deg=median("heading_error_deg", arrived),
                arrived_median_abs_lateral_m=median("lateral_m", arrived),
                arrived_median_abs_distance_error_m=(round(sorted(abs(r["perpendicular_m"]-stand_off_m) for r in arrived)[len(arrived)//2], 3)
                                                     if arrived else None),
                median_commands=sorted(r["commands"] for r in rows)[len(rows)//2],
                median_time_s=sorted(r["time_s"] for r in rows)[len(rows)//2],
                min_clearance_violations=sum(r["min_clearance_m"] < .40 for r in rows))


def stand_check():
    """Model box versus the measured cup stand clips (lens 1.60 m, cup center 1.40 m, heading 0)."""
    measured = {0.8: (.2341, .4285, .110), 1.0: (.1829, .3433, .027), 1.2: (.1490, .2832, -.097),
                2.0: (.0976, .1869, -.147), 2.5: (.0760, .1497, -.220), 3.0: (.0658, .1277, -.237)}
    camera, cup = Camera(), Print("cup", 0., 1.40, .194, .188)
    rows = []
    for d, (w, h, ey) in measured.items():
        b = camera.box((-d, 0., 1.60, 0.), cup)
        rows.append(dict(distance_m=d, w_model=round(b["w"], 4), w_measured=w, h_model=round(b["h"], 4),
                         h_measured=h, ey_model=round(b["cy"]-.5, 3), ey_measured=ey))
    return rows


def plot(path, example_seeds, square_rows, baseline_rows):
    """Top-view example paths (one per panel) and final pose errors versus start angle."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ink, muted, surface, grid = "#0b0b0b", "#52514e", "#fcfcfb", "#e4e3df"
    blue, orange = "#2a78d6", "#eb6834"
    plt.rcParams.update({"font.size": 9, "text.color": ink, "axes.labelcolor": muted,
                         "xtick.color": muted, "ytick.color": muted, "axes.edgecolor": grid})
    fig = plt.figure(figsize=(13, 7.2), facecolor=surface)
    gs = fig.add_gridspec(2, 4, height_ratios=[.9, 1], hspace=.32, wspace=.3)
    for i, seed in enumerate(example_seeds):
        result, log = run_episode(seed, keep_log=True)
        s = scenario(seed)
        ax = fig.add_subplot(gs[0, i], facecolor=surface)
        ax.axhline(0, color=ink, lw=2)
        ax.axhline(-.65, color=muted, lw=1, ls=(0, (3, 3)))
        for p in default_prints():
            ax.plot([p.y-p.width/2, p.y+p.width/2], [0, 0], color=ink, lw=6, solid_capstyle="butt")
            ax.text(p.y, .06, p.label, ha="center", va="bottom", color=ink if p.label == s["label"] else muted,
                    fontweight="bold" if p.label == s["label"] else "normal")
        xs = [s["pose"][1]]+[e["y"] for e in log]
        ys = [s["pose"][0]]+[e["x"] for e in log]
        ax.plot(xs, ys, color=blue, lw=2, marker="o", ms=4, mfc=surface, mew=1.2)
        for x, y, heading in ((xs[0], ys[0], s["pose"][3]), (xs[-1], ys[-1], math.radians(log[-1]["heading_deg"]))):
            ax.annotate("", xy=(x+.25*math.sin(heading), y+.25*math.cos(heading)), xytext=(x, y),
                        arrowprops=dict(arrowstyle="-|>", color=ink, lw=1.2))
        ax.text(xs[0], ys[0]-.12, "start", ha="center", va="top", color=muted)
        ax.set_title(f"{s['label']}: {abs(s['view_deg']):.0f}° off the normal\n"
                     f"{result['commands']} commands, {result['time_s']:.0f} s", color=ink, fontsize=9)
        ax.set_xlim(-2.6, 2.6)
        ax.set_ylim(min(ys)-.35, .35)
        ax.set_aspect("equal")
        ax.grid(color=grid, lw=.6)
        ax.set_xlabel("along the wall (m)")
        if i == 0:
            ax.set_ylabel("from the wall (m)")
            ax.text(-2.5, -.7, "0.65 m stand-off", ha="left", va="top", color=muted, fontsize=8)
    for j, (key, limit, label) in enumerate((("lateral_m", .08, "final offset from the print (m)"),
                                            ("heading_error_deg", 8., "final angle to the wall normal (°)"))):
        ax = fig.add_subplot(gs[1, 2*j:2*j+2], facecolor=surface)
        for rows, color, name in ((baseline_rows, orange, "yaw only"), (square_rows, blue, "ToF square-up")):
            done = [r for r in rows if r["outcome"] == "arrived"]
            ax.scatter([abs(r["start_view_deg"]) for r in done], [abs(r[key]) for r in done], s=26,
                       color=color, edgecolors=surface, linewidths=.8, label=name, zorder=3)
        ax.axhline(limit, color=muted, lw=1, ls=(0, (3, 3)))
        ax.text(50, limit, "  tolerance", va="center", ha="left", color=muted, fontsize=8)
        ax.set_xlim(0, 58)
        ax.set_ylim(0, None)
        ax.grid(color=grid, lw=.6)
        ax.set_xlabel("start angle between the line of sight and the wall normal (°)")
        ax.set_ylabel(label)
        ax.legend(frameon=False, loc="upper left")
    fig.suptitle("Stop-and-go planner in the geometric step world (not a flight model)", color=ink, fontsize=11)
    fig.savefig(path, dpi=130, facecolor=surface, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Evaluate the stop-and-go planner in the geometric step world")
    parser.add_argument("--episodes", type=int, default=300)
    parser.add_argument("--seed", type=int, default=60000)
    parser.add_argument("--variants", nargs="+", default=list(VARIANTS), choices=list(VARIANTS))
    parser.add_argument("--example", type=int, default=None, help="seed whose step-by-step log is written")
    parser.add_argument("--plot", type=int, nargs=4, metavar="SEED",
                        help="write stepwise.png with these four example seeds (needs matplotlib)")
    parser.add_argument("--noise", help="JSON of Noise fields, e.g. noise.json from objectpilot.step_flight")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    seeds = range(args.seed, args.seed+args.episodes)
    if seeds.start < RESERVED_SEEDS.stop and seeds.stop > RESERVED_SEEDS.start:
        parser.error("seeds 10000-59999 belong to the PID/PPO simulator splits; use 60000 or higher")
    base = None
    if args.noise:
        fields = json.loads(Path(args.noise).read_text(encoding="utf-8"))
        unknown = set(fields)-set(Noise.__dataclass_fields__)
        if unknown:
            parser.error(f"unknown Noise fields: {sorted(unknown)}")
        base = replace(Noise(), **fields)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    summary = dict(model="objectpilot.stepsim geometric step world (not a flight model)",
                   seeds=[seeds.start, seeds.stop-1], stand_check=stand_check(), variants={},
                   noise=vars(base or Noise()))
    results = {}
    for variant in args.variants:
        rows = results[variant] = evaluate(variant, seeds, base)
        summary["variants"][variant] = summarize_results(rows)
        with open(out/f"{variant}-episodes.csv", "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(variant, json.dumps(summary["variants"][variant]))
    if args.example is not None:
        result, log = run_episode(args.example, noise=base, keep_log=True)
        (out/f"example-{args.example}.json").write_text(json.dumps(dict(result=result, steps=log), indent=1),
                                                        encoding="utf-8")
    (out/"summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    if args.plot:
        square = results.get("square") or evaluate("square", seeds, base)
        baseline = results.get("no_square") or evaluate("no_square", seeds, base)
        plot(out/"stepwise.png", args.plot, square, baseline)


if __name__ == "__main__":
    main()
