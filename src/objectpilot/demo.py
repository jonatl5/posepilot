"""Offline, recorded simulator demo: no camera, downloaded weights or drone required."""
import argparse
import json
from dataclasses import asdict
from importlib.resources import files
from pathlib import Path

from .core import Config, PID, PIDGains, policy_contract, rc_channels
from .env import ApproachEnv
from .evaluate import check_seed_request, load_policy


def run_demo(out, seed=10000, model_path=None, randomized=True, config=None, gains=None, gains_file=None):
    out = Path(out)
    if out.exists():
        raise FileExistsError(f"Output already exists: {out}; choose a new --out directory")
    config = config or Config()
    policy = load_policy(model_path, config) if model_path else None
    controller = PID(config, gains)
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
    summary = dict(**policy_contract(config), seed=seed, randomized=randomized,
                   controller="PPO" if policy else "PID "+controller.gains.label(), model=str(model_path) if model_path else None,
                   pid_gains=None if policy else asdict(controller.gains), gains_file=gains_file,
                   outcome=info["outcome"], success=info["success"],
                   measured_arrival=info["measured_arrival"], truth_verified=info["truth_verified"],
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
    parser.add_argument("--config", help="JSON Config shared with training and camera replay")
    parser.add_argument("--gains", help="PID gains JSON for the baseline, e.g. configs/pid_gains_tuned.json; not used with --model")
    parser.add_argument("--nominal", action="store_true", help="Reduce randomization, retaining the same observation interface")
    parser.add_argument("--final-test", action="store_true",
                        help="Allow the reserved final test seeds 50000-50099; use once, at feature freeze")
    args = parser.parse_args()
    if args.model and args.gains:
        parser.error("--gains configures the PID baseline; omit it with --model")
    try:
        check_seed_request(args.seed, 1, args.final_test)
        config = Config.from_json(args.config) if args.config else Config()
        gains = PIDGains.from_json(args.gains) if args.gains else None
    except (OSError, ValueError) as error:
        parser.error(str(error))
    summary = run_demo(args.out, args.seed, args.model, not args.nominal, config, gains, args.gains)
    print(json.dumps(summary, indent=2))
    print(f"Open {(Path(args.out) / 'demo.html').resolve()}")


if __name__ == "__main__":
    main()
