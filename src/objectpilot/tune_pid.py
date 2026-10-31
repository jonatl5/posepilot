"""Tune the PID baseline on simulation tuning seeds and select on validation seeds.

Both seed ranges are development data for the planar simulator. The reserved final
test scenarios (50000-50099) are refused here so they stay untouched until feature
freeze. Selected gains are a simulation candidate: RC response, braking, camera
delay and range timing still need hardware calibration before any gain is trusted
in flight. Failed episodes are never dropped from the ranking metrics.
"""
import argparse
import csv
import itertools
import json
import os
import platform
import subprocess
import time
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from datetime import date
from importlib.metadata import version
from pathlib import Path

import numpy as np

from .core import Config, PIDGains, policy_contract
from .evaluate import benchmark, summarize, seed_role, check_seed_request

GAIN_NAMES = ("kp_forward", "ki_forward", "kd_forward", "kp_yaw", "ki_yaw", "kd_yaw")
DEFAULT_GRID = {"kp_forward": (1.0, 1.5, 2.0, 2.5, 3.0, 4.0), "ki_forward": (0.0, 0.1, 0.2),
                "kd_forward": (0.0, 0.08, 0.2), "kp_yaw": (2.0, 3.5, 5.0), "ki_yaw": (0.0,), "kd_yaw": (0.05, 0.15)}
SELECTION_RULE = ("highest success rate, then fewest minimum-distance violations, shortest mean episode "
                  "time with failures counted at the time limit, smoothest commands; exact ties keep the starting gains")


def describe(rows, limit_s):
    """Aggregate episode rows; failures keep the full time limit in the time mean."""
    summary = summarize(rows, limit_s)
    summary.update(
        successes=sum(r["success"] for r in rows),
        outcomes=dict(sorted(Counter(r["outcome"] for r in rows).items())),
        mean_center_error=float(np.mean([r["mean_center_error"] for r in rows])),
        mean_action_delta_sq=float(np.mean([r["mean_action_delta_sq"] for r in rows])),
        minimum_clearance_m=float(min(r["minimum_clearance_m"] for r in rows)))
    return summary


def ranking_key(name, summary):
    return (-summary["success_rate"], summary["minimum_distance_violations"], summary["mean_time_censored_s"],
            summary["mean_action_delta_sq"], name != "starting", name)


def candidate_grid(grid):
    """Starting gains first, then every grid combination that differs from them."""
    candidates = {"starting": PIDGains()}
    for combo in itertools.product(*(grid[name] for name in GAIN_NAMES)):
        gains = PIDGains(**dict(zip(GAIN_NAMES, combo)))
        if gains != candidates["starting"]:
            candidates[f"c{len(candidates):03d}"] = gains
    return candidates


def _run(task):
    name, gains, config, episodes, seed = task
    return name, benchmark(episodes, seed, None, True, Config(**config), PIDGains(**gains))


def run_all(candidates, config, episodes, seed, workers):
    tasks = [(name, asdict(gains), asdict(config), episodes, seed) for name, gains in candidates.items()]
    if workers <= 1:
        return dict(map(_run, tasks))
    with ProcessPoolExecutor(max_workers=min(workers, len(tasks))) as pool:
        return dict(pool.map(_run, tasks))


def values(text):
    try:
        return tuple(float(v) for v in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError("Use comma-separated numbers")


def write_table(path, names, candidates, summaries):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = None
        for rank, name in enumerate(names, 1):
            s = summaries[name]
            row = {"rank": rank, "candidate": name, **asdict(candidates[name]),
                   "episodes": s["episodes"], "successes": s["successes"], "success_rate": s["success_rate"],
                   "wilson_low": s["success_95pct_wilson"][0], "wilson_high": s["success_95pct_wilson"][1],
                   "minimum_distance_violations": s["minimum_distance_violations"],
                   "mean_time_censored_s": s["mean_time_censored_s"], "mean_success_time_s": s["mean_success_time_s"],
                   "mean_final_distance_error_m": s["mean_final_distance_error_m"],
                   "mean_center_error": s["mean_center_error"], "mean_action_delta_sq": s["mean_action_delta_sq"],
                   "minimum_clearance_m": s["minimum_clearance_m"],
                   "outcomes": " ".join(f"{k}:{v}" for k, v in s["outcomes"].items())}
            if writer is None:
                writer = csv.DictWriter(f, fieldnames=list(row))
                writer.writeheader()
            writer.writerow(row)


def write_episodes(path, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", help="JSON Config shared with evaluation, demo and replay")
    p.add_argument("--tuning-seed", type=int, default=40000)
    p.add_argument("--tuning-episodes", type=int, default=50)
    p.add_argument("--validation-seed", type=int, default=30000)
    p.add_argument("--validation-episodes", type=int, default=100)
    p.add_argument("--top", type=int, default=5, help="Tuning candidates re-run on the validation seeds")
    p.add_argument("--workers", type=int, default=max(1, min(16, os.cpu_count() or 1)))
    for name in GAIN_NAMES:
        p.add_argument("--"+name.replace("_", "-"), type=values, default=DEFAULT_GRID[name], metavar="V1,V2,...")
    p.add_argument("--out", default="runs/pid-tuning")
    a = p.parse_args(argv)
    if min(a.tuning_episodes, a.validation_episodes, a.top, a.workers) < 1:
        p.error("episodes, top and workers must be positive")
    try:
        check_seed_request(a.tuning_seed, a.tuning_episodes)
        check_seed_request(a.validation_seed, a.validation_episodes)
    except ValueError as error:
        p.error(str(error))
    tuning = range(a.tuning_seed, a.tuning_seed+a.tuning_episodes)
    validation = range(a.validation_seed, a.validation_seed+a.validation_episodes)
    if tuning.start < validation.stop and validation.start < tuning.stop:
        p.error("Tuning and validation seeds must not overlap")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    c = Config.from_json(a.config) if a.config else Config()
    limit_s = c.max_steps*c.dt
    candidates = candidate_grid({name: getattr(a, name) for name in GAIN_NAMES})
    started = time.perf_counter()
    tuning_rows = run_all(candidates, c, a.tuning_episodes, a.tuning_seed, a.workers)
    tuning_summaries = {name: describe(rows, limit_s) for name, rows in tuning_rows.items()}
    order = sorted(candidates, key=lambda name: ranking_key(name, tuning_summaries[name]))
    write_table(out/"candidates.csv", order, candidates, tuning_summaries)
    shortlist = order[:a.top]
    if "starting" not in shortlist:
        shortlist.append("starting")
    validation_rows = run_all({name: candidates[name] for name in shortlist}, c,
                              a.validation_episodes, a.validation_seed, a.workers)
    validation_summaries = {name: describe(rows, limit_s) for name, rows in validation_rows.items()}
    final_order = sorted(shortlist, key=lambda name: ranking_key(name, validation_summaries[name]))
    write_table(out/"validation.csv", final_order, candidates, validation_summaries)
    for name in final_order:
        write_episodes(out/f"validation-{name}-episodes.csv", validation_rows[name])
    selected = final_order[0]
    revision = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True)
    record = {
        "gains": asdict(candidates[selected]), "candidate": selected, "label": candidates[selected].label(),
        "status": "simulation candidate selected on validation seeds; not hardware calibrated",
        "selection_rule": SELECTION_RULE,
        "tuning": {"seed_start": a.tuning_seed, "episodes": a.tuning_episodes,
                   "seed_role": seed_role(a.tuning_seed, a.tuning_episodes), "candidates": len(candidates),
                   "grid": {name: list(getattr(a, name)) for name in GAIN_NAMES},
                   "selected_rank": order.index(selected)+1, "selected_summary": tuning_summaries[selected],
                   "starting_summary": tuning_summaries["starting"]},
        "validation": {"seed_start": a.validation_seed, "episodes": a.validation_episodes,
                       "seed_role": seed_role(a.validation_seed, a.validation_episodes),
                       "ranking": final_order, "summaries": {name: validation_summaries[name] for name in final_order}},
        **policy_contract(c),
        "date": date.today().isoformat(), "git_base": revision.stdout.strip(),
        "python": platform.python_version(), "platform": platform.platform(),
        "versions": {name: version(name) for name in ("numpy", "gymnasium")},
        "workers": a.workers, "elapsed_s": round(time.perf_counter()-started, 1),
        "evidence": "synthetic sensor simulation; not flight results",
    }
    (out/"selected_gains.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    print(json.dumps({"selected": selected, "gains": record["gains"],
                      "validation_successes": {name: [validation_summaries[name]["successes"], a.validation_episodes]
                                               for name in final_order},
                      "out": str(out)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
