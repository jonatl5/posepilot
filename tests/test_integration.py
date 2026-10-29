"""Regression checks for integration boundaries absent from the original package."""
from dataclasses import asdict
import json
import numpy as np
import pytest
import cv2
from objectpilot.core import Config, Detection, RangeSample, CustomRangeParser, Supervisor, parse_ext_tof
from objectpilot.pipeline import ApproachPipeline
from objectpilot.dispatch import CommandDispatcher, Telemetry
from objectpilot.ground_link import GroundLink
from objectpilot.vision import MarkerDetector
from objectpilot.replay import main, read_ranges


def det(t, frame=0, distance=2., cx=.5):
    return Detection(cx, .5, .3, .22/distance, .9, t, frame)


@pytest.mark.parametrize('values', [dict(dt=0), dict(history=3), dict(stand_off_m=.3),
                                  dict(rc_forward_cap=101), dict(visual_scale_m=float('nan'))])
def test_config_rejects_incompatible_or_invalid_values(values):
    with pytest.raises(ValueError):
        Config(**values)


def test_stock_reply_does_not_authorize_approach_or_arrival():
    p = ApproachPipeline()
    stock = parse_ext_tof('tof 650', 0.)
    assert stock.distance_m == .65 and stock.status == 'sample_age_unknown'
    row = p.step(det(0, distance=.65), stock, 0.)
    assert row['rc'] == (0, 0, 0, 0) and not row['arrived']
    assert row['supervisor'] == 'near_range_unavailable'
    row = ApproachPipeline().step(det(0, distance=2., cx=.8), parse_ext_tof('tof 250', 0.), 0.)
    assert row['supervisor'] == 'front_clearance_stop' and row['rc'] == (0, 0, 0, 0)


def test_measured_arrival_requires_continuous_two_second_hold_and_latches():
    p = ApproachPipeline()
    for i in range(21):
        t = i*.1
        row = p.step(det(t, i, .65), RangeSample(.65, t, 'forward_custom', sequence=i), t)
        assert row['rc'] == (0, 0, 0, 0)
        if i < 20:
            assert not row['arrived']
    assert row['arrived']
    assert p.step(det(3, 30, 2.), None, 3)['rc'] == (0, 0, 0, 0)


def test_arrival_does_not_count_missing_frames_or_scheduler_gap():
    p = ApproachPipeline()
    for i in range(10):
        p.step(det(i*.1, i, .65), RangeSample(.65, i*.1, 'forward_custom'), i*.1)
    p.step(None, None, 1.)
    row = p.step(det(2., 20, .65), RangeSample(.65, 2., 'forward_custom'), 2.)
    assert not row['arrived']
    row = p.step(det(5., 50, .65), RangeSample(.65, 5., 'forward_custom'), 5.)
    assert not row['arrived']


def test_custom_range_rollover_and_invalid_clock():
    p = CustomRangeParser()
    assert p.parse('oprange 4294967295 0 650 1 0', 0, .01).status == 'ok'
    assert p.parse('oprange 0 100 650 1 0', .1, .11).status == 'ok'
    assert p.parse('oprange 1 200 650 1 0', float('nan'), .2).status == 'invalid_clock'
    assert p.parse('oprange 4294967296 0 650 1 0', 1, 1.01).status == 'malformed'


def test_wrong_schema_fails_closed():
    for obs in ([0]*11, [[0]*64], ['bad']*64):
        command, reason = Supervisor().apply([1, 1], obs)
        assert np.array_equal(command, [0, 0]) and reason == 'invalid_action_or_state'


def test_markers_produce_the_primary_detection_contract():
    frame = np.full((480, 640, 3), 255, np.uint8)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 120)
    frame[180:300, 380:500] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    candidates = MarkerDetector('aruco').detect(frame, 4., 9)
    assert len(candidates) == 1
    d = candidates[0]
    assert d.frame_id == 9 and d.timestamp == 4. and d.cx-.5 == pytest.approx(.187, abs=.003)
    row = ApproachPipeline().step(d, RangeSample(.9, 4., 'forward_custom'), 4.)
    json.dumps(row, allow_nan=False)
    assert len(row['observation']) == 64 and row['rc'][3] > 0 and row['rc'][0:3] == (0, 0, 0)
    frame[:] = (255, 0, 0)
    assert MarkerDetector('red').detect(frame, 0, 0) == []
    frame[100:200, 100:200] = (0, 0, 255)
    assert len(MarkerDetector('red').detect(frame, 0, 0)) == 1


class Sink:
    def __init__(self):
        self.sent, self.lands = [], 0
    def send_rc(self, command):
        self.sent.append(command)
    def request_land(self):
        self.lands += 1


def armed_dispatcher():
    clock = [0.]
    sink = Sink()
    dispatch = CommandDispatcher(sink, clock=lambda: clock[0])
    dispatch.update_telemetry(Telemetry(0., 90, 80))
    dispatch.arm_after_takeoff()
    dispatch.resume()
    dispatch.submit(ApproachPipeline().step(det(0.), None, 0.))
    return dispatch, sink, clock


def test_dispatcher_rejects_bad_metadata_and_stale_producer_independently():
    dispatch, sink, clock = armed_dispatcher()
    assert dispatch.tick()[1] > 0
    clock[0] = .4
    dispatch.heartbeat()
    dispatch.update_telemetry(Telemetry(.4, 90, 80))
    assert dispatch.tick() == (0, 0, 0, 0) and dispatch.reason == 'command_stale'
    clock[0] = 3.5
    dispatch.heartbeat()
    dispatch.update_telemetry(Telemetry(3.5, 90, 80))
    dispatch.tick()
    assert sink.lands == 1
    dispatch.tick()
    assert sink.lands == 1
    with pytest.raises(ValueError):
        dispatch.submit({'schema': 'old-11-feature'})


def test_dispatcher_ui_heartbeat_low_battery_and_manual_hold():
    dispatch, sink, clock = armed_dispatcher()
    dispatch.hold()
    assert sink.sent[-1] == (0, 0, 0, 0)
    clock[0] = .6
    dispatch.update_telemetry(Telemetry(.6, 90, 80))
    dispatch.tick()
    assert dispatch.reason == 'operator_heartbeat_lost'
    dispatch.heartbeat()
    dispatch.update_telemetry(Telemetry(.6, 19, 80))
    dispatch.tick()
    assert sink.lands == 1 and sink.sent[-1] == (0, 0, 0, 0)


def test_ground_link_rejects_every_flight_command_before_network_use():
    link = GroundLink.__new__(GroundLink)
    for command in ('takeoff', 'land', 'rc 0 20 0 0', 'motoron', 'command\ntakeoff'):
        with pytest.raises(ValueError):
            link.request(command)


def test_dispatcher_supervisor_fault_timeout_and_failed_zero_still_request_land():
    dispatch, sink, clock = armed_dispatcher()
    dispatch.submit(ApproachPipeline().step(None, None, 0.))
    assert dispatch.tick() == (0, 0, 0, 0)
    assert dispatch.reason == 'supervisor_hold'
    clock[0] = 3.1
    dispatch.heartbeat()
    dispatch.update_telemetry(Telemetry(3.1, 90, 80))
    dispatch.submit(ApproachPipeline().step(None, None, 3.1))
    dispatch.tick()
    assert sink.lands == 1

    dispatch, sink, clock = armed_dispatcher()
    def fail_send(command):
        raise OSError('Simulated transport failure')
    sink.send_rc = fail_send
    dispatch.update_telemetry(Telemetry(0., 19, 80))
    with pytest.raises(OSError):
        dispatch.tick()
    assert sink.lands == 1


def test_file_sidecar_and_dry_replay(tmp_path):
    video = tmp_path/'red.avi'
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'MJPG'), 30., (320, 240))
    assert writer.isOpened()
    for _ in range(30):
        frame = np.full((240, 320, 3), 240, np.uint8)
        cv2.rectangle(frame, (145, 105), (175, 135), (0, 0, 255), -1)
        writer.write(frame)
    writer.release()
    out = tmp_path/'output.jsonl'
    main(['--source', str(video), '--detector', 'red', '--out', str(out)])
    rows = [json.loads(x) for x in out.read_text().splitlines()]
    assert len(rows) == 10
    assert all(len(r['observation']) == 64 and not r['sent_rc'] for r in rows)
    assert rows[-1]['detection'] is not None
    assert all(r['rc'][0] == 0 and r['rc'][2] == 0 for r in rows)
    bad = tmp_path/'bad.jsonl'
    bad.write_text('{"distance_m":0.65,"timestamp":-1}\n')
    with pytest.raises(ValueError):
        read_ranges(bad)
