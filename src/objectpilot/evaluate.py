import argparse
import csv
import json
import math
from dataclasses import asdict
from pathlib import Path
import numpy as np
from .core import Config, PID, PIDGains, policy_contract
from .env import ApproachEnv

# Scenario seeds are the simulator's only randomness, so seed ranges partition the
# scenarios. Development seeds were inspected while debugging. Tuning seeds choose
# PID gains, validation seeds choose between controllers and training seeds, and
# the final test range stays untouched until feature freeze.
SEED_RANGES = {
    "development": (range(10000, 10030), range(20000, 20030)),
    "validation": (range(30000, 30100),),
    "tuning": (range(40000, 40100),),
    "final_test": (range(50000, 50100),),
}
RESERVED_FINAL_TEST_SEEDS = SEED_RANGES["final_test"][0]


def seed_role(seed, episodes):
    """Name the documented range holding every requested seed; otherwise mixed or unassigned."""
    requested = range(seed, seed+episodes)
    for role, ranges in SEED_RANGES.items():
        if any(requested.start >= r.start and requested.stop <= r.stop for r in ranges):
            return role
    overlapping = any(requested.start < r.stop and r.start < requested.stop
                      for ranges in SEED_RANGES.values() for r in ranges)
    return "mixed" if overlapping else "unassigned"


def touches_final_test(seed, episodes):
    r = RESERVED_FINAL_TEST_SEEDS
    return seed < r.stop and r.start < seed+episodes


def check_seed_request(seed, episodes, final_test=False):
    """Refuse the reserved final test scenarios unless the caller declares the freeze."""
    if episodes < 1:
        raise ValueError("episodes must be positive")
    if touches_final_test(seed, episodes) and not final_test:
        raise ValueError(f"Seeds {RESERVED_FINAL_TEST_SEEDS.start}-{RESERVED_FINAL_TEST_SEEDS.stop-1} are reserved "
                         "for the frozen final comparison; pass --final-test only at feature freeze")


def controller_label(model, gains):
    if model is not None:
        return "PPO"
    g = gains if gains is not None else PIDGains()
    return "untuned PID starting gains" if g == PIDGains() else "PID tuned gains"

def load_policy(path, config=None):
    path = Path(path)
    meta = json.loads((path.parent/"metadata.json").read_text(encoding="utf-8"))
    c = config or Config()
    for key, value in policy_contract(c).items():
        if meta.get(key) != value:
            raise ValueError(f"Policy {key} mismatch; retrain with the current deployment contract/config")
    from stable_baselines3 import PPO
    from gymnasium.spaces import Box
    model = PPO.load(path, device="cpu")
    for space, shape in ((model.observation_space, (16*c.history,)), (model.action_space, (2,))):
        if (not isinstance(space, Box) or space.shape != shape or space.dtype != np.float32
                or not np.all(space.low == -1) or not np.all(space.high == 1)):
            raise ValueError("Policy tensor spaces do not match the current contract")
    return model

def benchmark(episodes, seed, model=None, randomized=True, config=None, gains=None):
    c = config or Config()
    rows = []
    for i in range(episodes):
        env = ApproachEnv(c, randomized=randomized)
        obs, info = env.reset(seed=seed+i)
        baseline = PID(c, gains)
        changes, errors, minimum = [], [], 100.
        while True:
            action = baseline.predict(obs) if model is None else model.predict(obs,deterministic=True)[0]
            obs, reward, term, trunc, info = env.step(action)
            minimum = min(minimum,info["wall_clearance_m"])
            changes.append(info["smoothness"])
            errors.append(abs(info["center_error"]))
            if term or trunc: break
        rows.append({"seed":seed+i, "success":info["success"], "outcome":info["outcome"],
                     "time_s":info["time_s"], "final_clearance_m":info["wall_clearance_m"],
                     "final_distance_error_m":abs(info["wall_clearance_m"]-c.stand_off_m),
                     "measured_arrival":info["measured_arrival"], "truth_verified":info["truth_verified"],
                     "mean_center_error":float(np.mean(errors)), "mean_action_delta_sq":float(np.mean(changes)),
                     "minimum_clearance_m":minimum})
        env.close()
    return rows

def summarize(rows, limit_s=None):
    n = len(rows)
    phat = sum(r["success"] for r in rows)/n
    z = 1.96
    center = (phat+z*z/(2*n))/(1+z*z/n)
    half = z*math.sqrt(phat*(1-phat)/n+z*z/(4*n*n))/(1+z*z/n)
    successes = [r["time_s"] for r in rows if r["success"]]
    summary = {"episodes":n,"success_rate":phat,"success_95pct_wilson":[max(0,center-half),min(1,center+half)],
               "mean_success_time_s":float(np.mean(successes)) if successes else None,
               "minimum_distance_violations":sum(r["outcome"]=="minimum_distance" for r in rows),
               "mean_final_distance_error_m":float(np.mean([r["final_distance_error_m"] for r in rows])),
               "evidence":"synthetic sensor simulation; not flight results"}
    if limit_s is not None:
        # Failures count at the full time limit so a fast crash never looks quick.
        summary["mean_time_censored_s"] = float(np.mean([r["time_s"] if r["success"] else limit_s for r in rows]))
    return summary

def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--model")
    p.add_argument("--episodes",type=int,default=100)
    p.add_argument("--seed",type=int,default=10000)
    p.add_argument("--nominal",action="store_true")
    p.add_argument("--config", help="JSON Config; must match training and replay")
    p.add_argument("--gains", help="PID gains JSON for the baseline, e.g. configs/pid_gains_tuned.json; not used with --model")
    p.add_argument("--final-test", action="store_true",
                   help="Allow the reserved final test seeds 50000-50099; use once, at feature freeze")
    p.add_argument("--out",default="runs/evaluation")
    a = p.parse_args(argv)
    if a.model and a.gains:
        p.error("--gains configures the PID baseline; omit it when evaluating a PPO model")
    try:
        check_seed_request(a.seed, a.episodes, a.final_test)
        c = Config.from_json(a.config) if a.config else Config()
        gains = PIDGains.from_json(a.gains) if a.gains else None
    except (OSError, ValueError) as error:
        p.error(str(error))
    model = load_policy(a.model, c) if a.model else None
    # Inputs are validated and loaded first, so a failed call leaves no output directory behind.
    out = Path(a.out)
    out.mkdir(parents=True,exist_ok=False)
    rows = benchmark(a.episodes,a.seed,model,not a.nominal,c,gains)
    with (out/"episodes.csv").open("w",newline="",encoding="utf-8") as f:
        w = csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
    summary = summarize(rows, c.max_steps*c.dt)
    summary.update(**policy_contract(c), controller=controller_label(model, gains), seed_start=a.seed,
                   seed_end=a.seed+a.episodes-1, seed_role=seed_role(a.seed, a.episodes),
                   randomized=not a.nominal, model=a.model,
                   pid_gains=None if model else asdict(gains if gains is not None else PIDGains()), gains_file=a.gains)
    (out/"summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
    print(json.dumps(summary,indent=2))

if __name__ == "__main__": main()
