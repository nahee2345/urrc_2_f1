import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from team_01_race.ramp import RampDetector, quaternion_pitch_deg
from team_01_race.navigation import DecisionEngine, Obstacle


def quaternion(pitch, roll=0., yaw=0.):
    r,p,y = (math.radians(v)/2 for v in (roll,pitch,yaw))
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def sample(detector, pitch, index, **kwargs):
    return detector.update(quaternion(pitch),index*.01,index*.01,**kwargs)


@pytest.mark.parametrize('pitch', [0.,4.9,5.,7.,-5.,-8.,45.])
def test_pitch_from_orientation_with_roll_and_yaw(pitch):
    assert quaternion_pitch_deg(*quaternion(pitch,13.,121.)) == pytest.approx(pitch)


@pytest.mark.parametrize('pitch', [0.,4.9,-4.9])
def test_flat_and_below_exact_five_degree_threshold(pitch):
    d = RampDetector()
    for i in range(20):sample(d,pitch,i)
    assert d.state(.19) == 'NORMAL'
    assert d.speed_limit(.19) == 10.


@pytest.mark.parametrize('values', [[5.,5.,5.],[5.1,5.3,5.4],[-5.,-5.,-5.],[-5.2,-5.5,-5.8]])
def test_uphill_downhill_three_distinct_samples(values):
    d = RampDetector()
    for i,p in enumerate(values):
        sample(d,p,i)
        assert d.active == (i == 2)
    assert d.state(.02) == 'RAMP'
    assert d.speed_limit(.02) == 5.


def test_filtered_threshold_and_hysteresis():
    d = RampDetector()
    for i in range(3):sample(d,0.,i)
    sample(d,8.,3)
    assert d.filtered == pytest.approx(2.4)
    assert not d.active
    for i in range(4,25):sample(d,8.,i)
    assert d.active
    for i in range(25,40):sample(d,4.,i)
    assert d.active  # Between enter/exit thresholds.
    for i in range(40,65):sample(d,2.5,i)
    assert not d.active
    assert d.speed_limit(.64) == 10.


def test_exact_exit_threshold_requires_three_samples():
    d = RampDetector(alpha=1.)
    for i in range(3):sample(d,5.,i)
    for i,p in enumerate([2.8,3.1,2.8,2.5,2.2],3):
        sample(d,p,i)
        assert d.active == (i < 7)
    for i in range(8,11):sample(d,-5.,i)
    assert d.active
    for i in range(11,14):sample(d,3.,i)
    assert not d.active


@pytest.mark.parametrize('quaternion_value', [(0.,0.,0.,0.),(0.,0.,0.,2.),(math.nan,0.,0.,1.),
                                               (0.,math.inf,0.,1.),(0.,0.,0.,math.inf)])
def test_invalid_orientation_never_updates_pitch_or_confirms(quaternion_value):
    d = RampDetector()
    sample(d,6.,0);sample(d,6.,1)
    assert not d.update(quaternion_value,.02,.02)
    assert d.enter_count == 0 and not d.active
    assert d.state(.02) == 'IMU_STALE'
    assert d.speed_limit(.02) == 5.
    assert d.raw == pytest.approx(6.)
    for i in range(3,5):sample(d,6.,i)
    assert not d.active
    sample(d,6.,5)
    assert d.active


def test_replay_old_sample_timeout_and_recovery():
    d = RampDetector()
    sample(d,6.,0)
    for _ in range(10):sample(d,6.,0)
    assert d.enter_count == 1
    assert d.last_valid_time == 0.
    assert d.state(.26) == 'IMU_STALE'
    assert d.speed_limit(.26) == 5.
    sample(d,6.,30)
    assert d.enter_count == 1
    sample(d,6.,31);sample(d,6.,32)
    assert d.active
    assert not d.update(quaternion(0.),.33,1.,age=.67)
    assert d.state(1.) == 'IMU_STALE' and d.active
    for i in range(101,104):sample(d,0.,i)
    assert not d.active


def test_orientation_unavailable_does_not_renew_freshness():
    d = RampDetector()
    sample(d,0.,0)
    assert not sample(d,0.,1,orientation_available=False)
    assert d.last_valid_time == 0.
    assert d.state(.01) == 'IMU_STALE'


def test_speed_cap_preserves_pass_path_and_emergency():
    d = RampDetector()
    for i in range(3):sample(d,6.,i)
    path = np.array([[.1,0.],[1.,0.],[5.,0.]])
    base = dict(path=path,mode='CENTER',speed=10.,enabled=True,reason='camera_center_path')
    result = d.constrain(base,.02)
    assert result['speed'] == 5. and result['mode'] == 'RAMP'
    assert result['path'] is path and base['speed'] == 10.
    base.update(mode='LEFT_PASS',speed=3.)
    result = d.constrain(base,.02)
    assert result['speed'] == 3. and result['mode'] == 'LEFT_PASS'
    base.update(mode='EMERGENCY_STOP',speed=0.,enabled=False)
    result = d.constrain(base,.02)
    assert result['speed'] == 0. and result['mode'] == 'EMERGENCY_STOP'
    assert d.constrain(dict(base,enabled=True,mode='CENTER',speed=10.),1.)['speed'] == 5.


def test_real_decision_obstacle_and_emergency_remain_below_ramp_cap():
    e = DecisionEngine(vehicle_width=.242,wheelbase=.34,require_official=False)
    e.controller_ready=True;e.set_light('LIGHTS_OUT')
    x=np.arange(.1,5.01,.1);p=np.column_stack((x,np.zeros_like(x)))
    l=p+[0.,1.2];r=p-[0.,1.2]
    for i in range(3):e.camera(p,l,r,1.,(True,True),i,i*.01);e.lidar([],i*.01)
    d=RampDetector()
    for i in range(3):sample(d,6.,i)
    assert d.constrain(e.step(.02),.02)['speed'] == 5.
    e.external_emergency=True
    assert d.constrain(e.step(.02),.02)['speed'] == 0.
    e.external_emergency=False
    e.lidar([Obstacle('car',.4,0.,.3,.3,1.,5.)],.02)
    assert d.constrain(e.step(.02),.02)['speed'] == 0.


def test_sensor_mount_and_world_system_support_absolute_orientation():
    root=Path(__file__).resolve().parents[1]
    model=ET.parse(root/'models/team_01_car/model.sdf').getroot().find('model')
    imu=model.find("link[@name='imu_link']")
    assert imu is not None and imu.find('visual') is None and imu.find('collision') is None
    assert imu.findtext('pose') == model.findtext("link[@name='base_link']/pose")
    sensor=imu.find('sensor')
    assert sensor.get('type') == 'imu' and float(sensor.findtext('update_rate')) == 100.
    assert sensor.findtext('imu/orientation_reference_frame/localization') == 'ENU'
    assert sensor.findtext('gz_frame_id') == 'team_01/imu_link'
    assert sensor.findtext('topic') == '/team_01/imu'
