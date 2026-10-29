import numpy as np
import pytest
from objectpilot.core import Config,Detection,RangeSample,FeatureBuilder,Supervisor,PID,parse_ext_tof,rc_channels
from objectpilot.env import ApproachEnv
from objectpilot.vision import TargetLock

def det(t=0,frame=0,cx=.5,height=.25): return Detection(cx,.5,.3,height,.9,t,frame)

def test_ext_range_units_sentinel_and_telemetry_rejection():
    assert parse_ext_tof("tof 650\r\n",0).distance_m==.65
    for reply in ("tof 8192","tof 65535","tof 0","tof:65;h:70;","650mm","error"):
        assert parse_ext_tof(reply,0).distance_m is None

def test_invalid_or_stale_data_stops_and_near_latches():
    b,s=FeatureBuilder(),Supervisor()
    obs=b.build(det(),RangeSample(.8,0),0,[0,0])
    assert s.apply([1,0],obs)[0][0]>0
    obs=b.build(det(t=1,frame=1,height=.08),None,1,[0,0])
    assert s.apply([1,0],obs)[1]=="near_range_unavailable"
    obs=b.build(det(t=1),RangeSample(.8,1),2,[0,0])
    assert np.all(s.apply([1,1],obs)[0]==0)

def test_close_range_stops_even_if_target_off_axis():
    b,s=FeatureBuilder(),Supervisor()
    obs=b.build(det(cx=.8),RangeSample(.30,0),0,[0,0])
    assert obs[-10]==0  # range_associated is false
    assert s.apply([1,1],obs)[1]=="front_clearance_stop"

def test_downward_range_cannot_be_used_as_forward():
    obs=FeatureBuilder().build(det(),RangeSample(.7,0,"downward_tof"),0,[0,0])
    assert obs[-11]==0  # forward_valid
    assert Supervisor().apply([1,0],obs)[1]=="near_range_unavailable"

def test_nonfinite_action_and_future_sample_fail_closed():
    obs=FeatureBuilder().build(det(),RangeSample(.8,0),0,[0,0])
    assert np.all(Supervisor().apply([np.nan,1],obs)[0]==0)
    assert rc_channels([np.inf,1])==(0,0,0,0)
    obs=FeatureBuilder().build(det(t=2),RangeSample(.8,2),0,[0,0])
    assert np.all(Supervisor().apply([1,1],obs)[0]==0)

def test_history_repeated_frame_and_bounded_commands():
    b=FeatureBuilder(); b.build(det(),None,0,[0,0])
    obs=b.build(det(),None,.1,[0,0])
    assert obs.shape==(64,) and np.isfinite(obs).all()
    assert obs[-5]==0
    assert rc_channels([10,-10])==(0,20,0,-25)
    assert rc_channels([-1,0])==(0,0,0,0)

def test_target_confirmation_requires_distinct_frames():
    lock=TargetLock(3)
    assert lock.select([det()],0) is None
    assert lock.select([det()],0) is None
    assert lock.select([det()],1) is None
    assert lock.select([det()],2) is not None
    assert lock.select([],3) is None
    assert lock.select([det()],4) is None

def test_env_contract_and_reproducibility():
    a,b=ApproachEnv(),ApproachEnv()
    oa,_=a.reset(seed=123); ob,_=b.reset(seed=123)
    assert np.array_equal(oa,ob)
    for _ in range(50):
        oa,ra,ta,ua,ia=a.step([.3,.1]); ob,rb,tb,ub,ib=b.step([.3,.1])
        assert np.array_equal(oa,ob) and (ra,ta,ua,ia)==(rb,tb,ub,ib)
        assert a.observation_space.contains(oa)
        if ta or ua: break

def test_optional_sb3_environment_contract():
    checker = pytest.importorskip("stable_baselines3.common.env_checker")
    checker.check_env(ApproachEnv(), warn=True)

def test_time_limit_is_truncation():
    env=ApproachEnv(Config(max_steps=1),randomized=False)
    env.reset(seed=5)
    _,_,term,trunc,_=env.step([0,0])
    assert trunc and not term

def test_nominal_pid_can_finish():
    env=ApproachEnv(randomized=False); obs,_=env.reset(seed=42); pid=PID()
    for _ in range(400):
        obs,_,term,trunc,info=env.step(pid.predict(obs))
        if term or trunc: break
    assert info["success"],info

def test_custom_range_freshness_and_sequence():
    from objectpilot.core import CustomRangeParser
    p=CustomRangeParser()
    r=p.parse("oprange 10 2000 650 1 30",100.,100.02)
    assert r.distance_m==.65 and r.timestamp==pytest.approx(99.97)
    assert p.parse("oprange 10 2000 650 1 30",101.,101.02).distance_m is None
    assert p.parse("oprange 11 2100 650 1 500",102.,102.02).status=="stale"
    assert p.parse("oprange 12 2200 65535 0 0",103.,103.02).status=="sensor_invalid"
    assert p.parse("oprange 1 20 650 1 0",104.,104.02).status=="repeated_or_reset"
    assert p.parse("oprange 2 120 650 1 0",105.,105.02).distance_m is None
