"""PID gains configuration, reserved final test seeds and the tuning workflow."""
import csv
import json
import subprocess
import sys
from dataclasses import asdict

import cv2
import numpy as np
import pytest

from objectpilot import tune_pid
from objectpilot.core import Detection, FeatureBuilder, PID, PIDGains, RangeSample
from objectpilot.demo import run_demo
from objectpilot.evaluate import benchmark, check_seed_request, seed_role, touches_final_test
from objectpilot.pipeline import ApproachPipeline
from objectpilot.replay import main as replay


def observation(cx=.6, distance=1.):
    detection = Detection(cx, .5, .3, .22/distance, .9, 0., 0)
    return FeatureBuilder().build(detection, RangeSample(distance, 0.), 0., [0, 0])


def test_gains_validate_load_and_label(tmp_path):
    assert PIDGains() == PIDGains(1.5, 3.5, 0., 0., .08, .05)
    for bad in (dict(kp_forward=0), dict(kd_yaw=-.1), dict(ki_forward=float('nan')),
                dict(kp_yaw=float('inf')), dict(kp_forward=True)):
        with pytest.raises(ValueError):
            PIDGains(**bad)
    flat = tmp_path/'flat.json'
    flat.write_text(json.dumps(dict(kp_forward=2, kp_yaw=4, ki_forward=.1, ki_yaw=0, kd_forward=.2, kd_yaw=.1)))
    record = tmp_path/'record.json'
    record.write_text(json.dumps({'gains': asdict(PIDGains(kp_forward=2.)), 'status': 'candidate'}))
    assert PIDGains.from_json(flat) == PIDGains(2., 4., .1, 0., .2, .1)
    assert PIDGains.from_json(record) == PIDGains(kp_forward=2.)
    assert PIDGains().label() == 'starting gains' and PIDGains(kp_forward=2.).label() == 'tuned gains'


def test_gains_scale_the_proposal_not_the_observation():
    obs = observation()
    starting = PID().predict(obs)
    tuned = PID(gains=PIDGains(kp_forward=3., kp_yaw=7.)).predict(obs)
    assert starting[0] == pytest.approx(1.5*.35) and starting[1] == pytest.approx(3.5*.1)
    assert tuned[0] == pytest.approx(1.) and tuned[1] == pytest.approx(.7)


def test_pipeline_rows_and_benchmark_report_the_gains_in_use():
    tuned = PIDGains(kp_forward=2.)
    row = ApproachPipeline(gains=tuned).step(Detection(.5, .5, .3, .22, .9, 0., 0), RangeSample(1., 0.), 0.)
    assert row['controller'] == 'PID tuned gains' and row['pid_gains'] == asdict(tuned)
    row = ApproachPipeline().step(None, None, 0.)
    assert row['controller'] == 'PID starting gains' and row['pid_gains'] == asdict(PIDGains())
    a = benchmark(1, 42, randomized=False)[0]
    b = benchmark(1, 42, randomized=False, gains=PIDGains(kp_forward=2.5, kd_forward=.2))[0]
    assert a['success'] and b['success'] and a['mean_action_delta_sq'] != b['mean_action_delta_sq']


def test_ranking_prefers_starting_gains_on_exact_ties():
    summary = dict(success_rate=1., minimum_distance_violations=0, mean_time_censored_s=10., mean_action_delta_sq=.01)
    ranked = sorted(['c001', 'starting', 'c002'], key=lambda name: tune_pid.ranking_key(name, summary))
    assert ranked == ['starting', 'c001', 'c002']


def test_reserved_final_test_seeds_need_an_explicit_freeze_flag(tmp_path):
    assert seed_role(10000, 30) == seed_role(20000, 30) == 'development'
    assert seed_role(30000, 100) == 'validation' and seed_role(40000, 50) == 'tuning'
    assert seed_role(50000, 100) == 'final_test'
    assert seed_role(29990, 20) == 'mixed' and seed_role(70000, 10) == 'unassigned'
    assert touches_final_test(49999, 2) and not touches_final_test(49990, 10) and not touches_final_test(50100, 5)
    with pytest.raises(ValueError, match='reserved'):
        check_seed_request(50050, 1)
    check_seed_request(50050, 1, final_test=True)
    with pytest.raises(ValueError):
        check_seed_request(30000, 0)
    out = tmp_path/'final'
    result = subprocess.run([sys.executable, '-m', 'objectpilot.evaluate', '--episodes', '1', '--seed', '50000',
                             '--out', str(out)], capture_output=True, text=True)
    assert result.returncode != 0 and 'reserved' in result.stderr and not out.exists()
    result = subprocess.run([sys.executable, '-m', 'objectpilot.evaluate', '--model', 'policy.zip', '--gains',
                             'gains.json', '--out', str(out)], capture_output=True, text=True)
    assert result.returncode != 0 and '--gains' in result.stderr and not out.exists()


def test_demo_and_replay_accept_a_gains_file(tmp_path):
    gains = PIDGains(kp_forward=2., kd_forward=.1)
    gains_file = tmp_path/'gains.json'
    gains_file.write_text(json.dumps({'gains': asdict(gains)}))
    summary = run_demo(tmp_path/'demo', seed=42, randomized=False, gains=gains, gains_file=str(gains_file))
    assert summary['controller'] == 'PID tuned gains' and summary['pid_gains'] == asdict(gains)
    assert summary['gains_file'] == str(gains_file) and summary['success']
    clip = tmp_path/'clip.avi'
    writer = cv2.VideoWriter(str(clip), cv2.VideoWriter_fourcc(*'MJPG'), 10., (320, 240))
    assert writer.isOpened()
    for _ in range(6):
        frame = np.full((240, 320, 3), 255, np.uint8)
        frame[90:150, 130:190] = (0, 0, 255)
        writer.write(frame)
    writer.release()
    out = tmp_path/'replay.jsonl'
    replay(['--source', str(clip), '--detector', 'red', '--gains', str(gains_file), '--out', str(out)])
    rows = [json.loads(line) for line in out.read_text().splitlines()]
    assert rows and all(row['pid_gains'] == asdict(gains) and row['controller'] == 'PID tuned gains' for row in rows)
    with pytest.raises(SystemExit):
        replay(['--source', str(clip), '--gains', str(gains_file), '--model', 'policy.zip',
                '--out', str(tmp_path/'rejected.jsonl')])


def test_tuning_ranks_on_tuning_seeds_then_selects_on_validation(tmp_path):
    out = tmp_path/'tuning'
    args = ['--tuning-seed', '40000', '--tuning-episodes', '2', '--validation-seed', '30000',
            '--validation-episodes', '2', '--kp-forward', '1.5,2.0', '--ki-forward', '0', '--kd-forward', '0.08',
            '--kp-yaw', '3.5', '--ki-yaw', '0', '--kd-yaw', '0.05', '--top', '2', '--workers', '1', '--out', str(out)]
    assert tune_pid.main(args) == 0
    record = json.loads((out/'selected_gains.json').read_text())
    assert PIDGains.from_json(out/'selected_gains.json') in (PIDGains(), PIDGains(kp_forward=2.))
    assert record['tuning']['seed_role'] == 'tuning' and record['validation']['seed_role'] == 'validation'
    assert record['tuning']['candidates'] == 2 and record['control_contract'] == 'objectpilot-control-v2'
    candidates = list(csv.DictReader((out/'candidates.csv').open()))
    validation = list(csv.DictReader((out/'validation.csv').open()))
    assert {c['candidate'] for c in candidates} == {'starting', 'c001'} == {v['candidate'] for v in validation}
    assert validation[0]['candidate'] == record['candidate'] and validation[0]['rank'] == '1'
    assert len(list(csv.DictReader((out/'validation-starting-episodes.csv').open()))) == 2
    again = tmp_path/'again'
    tune_pid.main(args[:-1]+[str(again)])
    assert (again/'candidates.csv').read_text() == (out/'candidates.csv').read_text()
    with pytest.raises(SystemExit):
        tune_pid.main(['--validation-seed', '50000', '--workers', '1', '--out', str(tmp_path/'reserved')])
    with pytest.raises(SystemExit):
        tune_pid.main(['--tuning-seed', '30010', '--tuning-episodes', '5', '--workers', '1',
                       '--out', str(tmp_path/'overlap')])
    assert not (tmp_path/'reserved').exists() and not (tmp_path/'overlap').exists()
