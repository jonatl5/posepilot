import argparse
import csv
import json
import math
from pathlib import Path
from dataclasses import asdict
import numpy as np
from .core import Config, PID, SCHEMA
from .env import ApproachEnv

def load_policy(path, config=None):
    from stable_baselines3 import PPO
    path = Path(path)
    meta = json.loads((path.parent/"metadata.json").read_text(encoding="utf-8"))
    if meta["schema"] != SCHEMA or meta["config"] != asdict(config or Config()):
        raise ValueError("Policy schema/config mismatch; regenerate with the deployment configuration")
    return PPO.load(path, device="cpu")

def benchmark(episodes, seed, model=None, randomized=True):
    rows = []
    for i in range(episodes):
        env = ApproachEnv(randomized=randomized)
        obs, info = env.reset(seed=seed+i)
        baseline = PID()
        changes, errors, minimum = [], [], 100.
        while True:
            action = baseline.predict(obs) if model is None else model.predict(obs,deterministic=True)[0]
            obs, reward, term, trunc, info = env.step(action)
            minimum = min(minimum,info["wall_clearance_m"])
            changes.append(info["smoothness"])
            errors.append(abs(info["center_error"]))
            if term or trunc: break
        rows.append({"seed":seed+i, "success":info["success"], "outcome":info["outcome"],
                     "time_s":info["time_s"], "final_distance_error_m":abs(info["wall_clearance_m"]-.65),
                     "mean_center_error":float(np.mean(errors)), "mean_action_delta_sq":float(np.mean(changes)),
                     "minimum_clearance_m":minimum})
        env.close()
    return rows

def summarize(rows):
    n = len(rows)
    phat = sum(r["success"] for r in rows)/n
    z = 1.96
    center = (phat+z*z/(2*n))/(1+z*z/n)
    half = z*math.sqrt(phat*(1-phat)/n+z*z/(4*n*n))/(1+z*z/n)
    successes = [r["time_s"] for r in rows if r["success"]]
    return {"episodes":n,"success_rate":phat,"success_95pct_wilson":[max(0,center-half),min(1,center+half)],
            "mean_success_time_s":float(np.mean(successes)) if successes else None,
            "minimum_distance_violations":sum(r["outcome"]=="minimum_distance" for r in rows),
            "mean_final_distance_error_m":float(np.mean([r["final_distance_error_m"] for r in rows])),
            "evidence":"synthetic sensor simulation; not flight results"}

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model")
    p.add_argument("--episodes",type=int,default=100)
    p.add_argument("--seed",type=int,default=10000)
    p.add_argument("--nominal",action="store_true")
    p.add_argument("--out",default="runs/evaluation")
    a = p.parse_args()
    if a.episodes < 1: p.error("episodes must be positive")
    out = Path(a.out)
    out.mkdir(parents=True,exist_ok=False)
    model = load_policy(a.model) if a.model else None
    rows = benchmark(a.episodes,a.seed,model,not a.nominal)
    with (out/"episodes.csv").open("w",newline="",encoding="utf-8") as f:
        w = csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
    summary = summarize(rows)
    summary.update(controller="PPO" if model else "untuned PID starting gains",seed_start=a.seed,
                   randomized=not a.nominal,model=a.model)
    (out/"summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
    print(json.dumps(summary,indent=2))

if __name__ == "__main__": main()
