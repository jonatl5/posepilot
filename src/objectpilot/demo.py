"""Offline, recorded simulator demo: no camera, downloaded weights or drone required."""
import argparse
import json
from dataclasses import asdict
from importlib.resources import files
from pathlib import Path

from .core import Config, PID, SCHEMA, rc_channels
from .env import ApproachEnv
from .evaluate import load_policy


def run_demo(out, seed=10000, model_path=None, randomized=True):
    out = Path(out)
    if out.exists():
        raise FileExistsError(f"Output already exists: {out}; choose a new --out directory")
    config = Config()
    policy = load_policy(model_path, config) if model_path else None
    controller = PID(config)
    env = ApproachEnv(config, randomized=randomized)
    samples = []
    try:
        obs, info = env.reset(seed=seed)
        samples.append(dict(t=0., x=env.x, y=env.y, heading=env.heading,
                            observation=obs.tolist(), proposed_action=[0., 0.],
                            safe_action=[0., 0.], rc=[0, 0, 0, 0],
                            supervisor="initial", outcome="running"))
        while True:
            proposed = controller.predict(obs) if policy is None else policy.predict(obs, deterministic=True)[0]
            obs, reward, terminated, truncated, info = env.step(proposed)
            samples.append(dict(t=info["time_s"], x=env.x, y=env.y, heading=env.heading,
                                observation=obs.tolist(), proposed_action=proposed.tolist(),
                                safe_action=info["safe_action"], rc=rc_channels(info["safe_action"], config),
                                supervisor=info["supervisor"], outcome=info["outcome"]))
            if terminated or truncated:
                break
    finally:
        env.close()
    summary = dict(schema=SCHEMA, seed=seed, randomized=randomized,
                   controller="PPO" if policy else "PID starting gains", model=str(model_path) if model_path else None,
                   config=asdict(config), outcome=info["outcome"], success=info["success"],
                   time_s=info["time_s"], final_clearance_m=info["wall_clearance_m"],
                   minimum_clearance_m=min(-s["x"] for s in samples),
                   evidence="synthetic sensor simulation; no flight or detector inference")
    # Embedding the trace makes the HTML work from a file:// URL, without a server.
    template = files("objectpilot").joinpath("templates/demo.html").read_text(encoding="utf-8")
    payload = json.dumps(dict(summary=summary, samples=samples)).replace("<", "\\u003c")
    html = template.replace("__OBJECTPILOT_TRACE__", payload)
    out.mkdir(parents=True, exist_ok=False)
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (out / "trajectory.jsonl").write_text("".join(json.dumps(s) + "\n" for s in samples), encoding="utf-8")
    (out / "demo.html").write_text(html, encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="runs/demo_pid")
    parser.add_argument("--seed", type=int, default=10000)
    parser.add_argument("--model", help="Optional PPO policy.zip; metadata.json must be beside it")
    parser.add_argument("--nominal", action="store_true", help="Reduce randomization, retaining the same observation interface")
    args = parser.parse_args()
    summary = run_demo(args.out, args.seed, args.model, not args.nominal)
    print(json.dumps(summary, indent=2))
    print(f"Open {(Path(args.out) / 'demo.html').resolve()}")


if __name__ == "__main__":
    main()
