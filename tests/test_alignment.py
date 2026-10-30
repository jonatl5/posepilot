"""Cross-path tests for the TT control contract, configuration and checkpoints."""
from dataclasses import asdict, replace
import json
import subprocess
import sys

import numpy as np
import pytest

from objectpilot.core import Config, CONTROL_CONTRACT, CustomRangeParser, Detection, RangeSample, policy_contract
from objectpilot.env import ApproachEnv
from objectpilot.evaluate import benchmark, load_policy
from objectpilot.pipeline import ApproachPipeline


def test_sim_and_camera_use_identical_measurements_actions_and_arrival():
    env = ApproachEnv(randomized=False)
    obs, _ = env.reset(seed=42)
    camera = ApproachPipeline(env.c)
    reasons = set()
    for _ in range(env.c.max_steps):
        row = camera.step(env.detection, env.range_sample, env.time)
        np.testing.assert_array_equal(row['observation'], obs)
        obs, _, term, trunc, info = env.step(row['proposed_action'])
        np.testing.assert_array_equal(row['safe_action'], info['safe_action'])
        assert tuple(info['rc']) == row['rc']
        assert info['supervisor'] == row['supervisor']
        assert info['measured_arrival'] == row['arrived']
        reasons.add(info['supervisor'])
        if term or trunc:
            break
    assert info['success']
    assert {'stale_or_missing_detection', 'arrival_hold', 'arrived'} <= reasons


def test_rc_caps_and_integer_quantization_affect_simulated_motion():
    def command(cap):
        env = ApproachEnv(Config(rc_forward_cap=cap), randomized=False)
        env.reset(seed=42)
        env.x, env.y, env.heading = -2., 0., 0.
        env.command_lag = 0
        env.commands.clear()
        for _ in range(5):
            env.step([0., 0.])
        _, _, _, _, info = env.step([.23, 0.])
        return env.v, info
    fast, a = command(20)
    slow, b = command(10)
    assert a['rc'][1] == 5 and b['rc'][1] == 2
    assert slow/fast == pytest.approx(2/5)


def test_measured_arrival_cannot_claim_success_when_physical_clearance_is_wrong():
    env = ApproachEnv(randomized=False)
    env.reset(seed=42)
    env.x, env.y, env.heading = -2., 0., 0.

    def biased_sensor():
        env.detection = Detection(.5, .5, .3, .22/.65, .9, env.time, env.steps)
        env.range_sample = RangeSample(.65, env.time, 'sim_forward')
        return env.builder.build(env.detection, env.range_sample, env.time, env.previous)

    env._observe = biased_sensor
    env.obs = env._observe()
    for _ in range(30):
        _, _, term, trunc, info = env.step([1., 0.])
        assert info['safe_action'] == [0., 0.]
        if term or trunc:
            break
    assert term and not trunc and info['measured_arrival']
    assert not info['success'] and not info['truth_verified']
    assert info['outcome'] == 'arrival_unverified'


@pytest.mark.parametrize('key,value', [
    ('control_contract', None), ('features', ['wrong']*16),
    ('action_names', ['yaw', 'forward']), ('schema', 'old-11'),
    ('config', asdict(Config(stand_off_m=.75))),
])
def test_incompatible_metadata_rejected_before_loading_weights(tmp_path, key, value):
    metadata = policy_contract(Config())
    metadata[key] = value
    (tmp_path/'metadata.json').write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match=key):
        load_policy(tmp_path/'policy.zip')


def test_nondefault_config_reaches_evaluation_metrics():
    config = Config(stand_off_m=.75, visual_scale_m=.27, max_steps=1)
    row = benchmark(1, 42, randomized=False, config=config)[0]
    assert row['final_distance_error_m'] == pytest.approx(abs(row['final_clearance_m']-.75))


def test_range_parser_uses_the_shared_freshness_configuration():
    parser = CustomRangeParser(Config(range_stale_s=.2))
    assert parser.parse('oprange 1 1000 650 1 250', 1., 1.01).status == 'stale'
    parser = CustomRangeParser(Config(range_stale_s=.6))
    assert parser.parse('oprange 1 1000 650 1 450', 1., 1.01).status == 'ok'


def test_all_ppo_entrypoints_share_nondefault_config(tmp_path):
    pytest.importorskip('stable_baselines3')
    c = replace(Config(), stand_off_m=.75, visual_scale_m=.27, rc_forward_cap=12, max_steps=12)
    config = tmp_path/'config.json'
    config.write_text(json.dumps(asdict(c)))

    def run(module, *args):
        subprocess.run([sys.executable, '-m', 'objectpilot.'+module, '--config', str(config), *map(str,args)],
                       check=True, capture_output=True, text=True, timeout=120)

    run('train', '--steps', 256, '--envs', 1, '--out', tmp_path/'train')
    model = tmp_path/'train/policy.zip'
    loaded = load_policy(model, c)
    assert loaded.observation_space.shape == (64,)
    run('evaluate', '--model', model, '--episodes', 1, '--out', tmp_path/'eval')
    run('demo', '--model', model, '--out', tmp_path/'demo')
    # A generated file exercises policy loading through the real replay CLI.
    import cv2
    clip = tmp_path/'clip.avi'
    writer = cv2.VideoWriter(str(clip), cv2.VideoWriter_fourcc(*'MJPG'), 10., (320,240))
    assert writer.isOpened()
    for _ in range(6):
        frame = np.full((240,320,3), 255, np.uint8)
        frame[90:150,130:190] = (0,0,255)
        writer.write(frame)
    writer.release()
    run('replay', '--source', clip, '--detector', 'red', '--model', model, '--out', tmp_path/'replay.jsonl')
    for path in ('train/metadata.json', 'eval/summary.json', 'demo/summary.json'):
        record = json.loads((tmp_path/path).read_text())
        assert record['config'] == asdict(c) and record['control_contract'] == CONTROL_CONTRACT
    rows = [json.loads(line) for line in (tmp_path/'replay.jsonl').read_text().splitlines()]
    assert rows and all(row['config'] == asdict(c) for row in rows)


def test_checkpoint_tensor_space_checked_even_with_valid_metadata(tmp_path):
    sb3 = pytest.importorskip('stable_baselines3')
    from gymnasium import Env, spaces

    class WrongLayout(Env):
        observation_space = spaces.Box(-1., 1., (11,), np.float32)
        action_space = spaces.Box(-1., 1., (2,), np.float32)

    model = sb3.PPO('MlpPolicy', WrongLayout(), n_steps=8, batch_size=8, device='cpu')
    model.save(tmp_path/'policy')
    (tmp_path/'metadata.json').write_text(json.dumps(policy_contract(Config())))
    with pytest.raises(ValueError, match='tensor spaces'):
        load_policy(tmp_path/'policy.zip')
