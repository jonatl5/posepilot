import argparse
import json
import subprocess
from dataclasses import asdict
from pathlib import Path
from importlib.metadata import version
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv
from .env import ApproachEnv
from .core import Config, SCHEMA, FEATURE_NAMES

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=int, default=100000)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--envs", type=int, default=4)
    p.add_argument("--nominal", action="store_true")
    p.add_argument("--out", default="runs/ppo")
    args = p.parse_args()
    if args.steps <= 0 or args.envs <= 0: p.error("steps and envs must be positive")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(1)
    c = Config()
    check_env(ApproachEnv(c), warn=True)
    env = DummyVecEnv([lambda: ApproachEnv(c, randomized=not args.nominal) for _ in range(args.envs)])
    env.seed(args.seed)
    model = PPO("MlpPolicy", env, seed=args.seed, device="cpu", n_steps=256,
                batch_size=64, learning_rate=3e-4, gamma=.99, gae_lambda=.95,
                ent_coef=.01, policy_kwargs={"net_arch":[64,64]}, verbose=1)
    model.learn(total_timesteps=args.steps)
    model.save(out/"policy")
    revision = subprocess.run(["git","rev-parse","HEAD"],capture_output=True,text=True)
    metadata = {"schema":SCHEMA, "features":FEATURE_NAMES, "config":asdict(c),
                "seed":args.seed, "requested_steps":args.steps, "actual_steps":model.num_timesteps,
                "randomized":not args.nominal, "normalization":"fixed scaling in core.py; no VecNormalize",
                "git_base":revision.stdout.strip(), "status":"simulation candidate only",
                "versions":{n:version(n) for n in ("numpy","torch","gymnasium","stable-baselines3")}}
    (out/"metadata.json").write_text(json.dumps(metadata,indent=2),encoding="utf-8")
    env.close()
    print(f"Saved {out / 'policy.zip'}; no hardware validation")

if __name__ == "__main__": main()
