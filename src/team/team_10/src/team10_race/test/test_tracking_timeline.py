"""Acquisition-order and ego-frame regressions, with no world-pose inputs."""
import math
from types import SimpleNamespace

import pytest

from team10_race.geometry import Point2
from team10_race.lidar_processing import ScanPoint
from team10_race.opponent import (
    OpponentCandidate, OpponentTracker, TrackingMotionTimeline, fractional_motion,
)
from team10_race.relaxed_navigation_node import NavigationNode


def candidate(x,y,stamp,source='rear'):
    return OpponentCandidate(x,y,math.hypot(x,y),.25,math.atan2(y,x),
                             OpponentTracker._position(x,y),(20,21,22),
                             timestamp=stamp,sources=(source,),extent=.2,point_count=3)


def timeline():
    return TrackingMotionTimeline(4.,.488692,.340)


def scan(stamp):
    sec=math.floor(stamp)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=sec,nanosec=round((stamp-sec)*1e9))))


def test_irregular_acquisition_intervals_use_exact_dt_and_continuous_id():
    motion=timeline();tracker=OpponentTracker();last=0.;ids=set()
    for i,stamp in enumerate((0.,.027,.143,.192,.310,.477,.551,.681)):
        delta=motion.update([],stamp,1.,0.)
        tracks=tracker.update([candidate(-6+stamp,.1,stamp)],
                              .1 if i==0 else stamp-last,timestamp=stamp,ego_motion=delta)
        if i:
            assert motion.diagnostics['dt']==pytest.approx(stamp-last)
            assert delta[0]==pytest.approx(-(stamp-last))
        ids.update(t.track_id for t in tracks)
        last=stamp
    assert ids=={1} and len(tracker._tracks)==1


@pytest.mark.parametrize('stamp',[1.,.99,float('nan')])
def test_out_of_order_and_duplicate_ego_epoch_do_not_mutate_pair_or_tracker(stamp):
    motion=timeline();tracker=OpponentTracker()
    motion.update([Point2(1,1)],1.,1.,0.)
    tracker.update([candidate(-3,.1,1.)],.1,timestamp=1.)
    assert motion.update([Point2(9,9)],stamp,1.,0.) is None
    tracker.update([candidate(-8,1.,stamp)],.1,timestamp=stamp)
    assert motion.timestamp==tracker.timestamp==1.
    assert motion.cloud==[Point2(1,1)]
    assert list(tracker._tracks)==[1] and tracker._tracks[1].age==1


def test_wrapped_ego_yaw_has_short_rotation_not_full_turn(monkeypatch):
    import team10_race.opponent as module
    old_yaw=math.pi-.01;new_yaw=-math.pi+.01
    monkeypatch.setattr(module,'estimate_tracking_motion',lambda *args:(-.1,0.,old_yaw-new_yaw))
    motion=timeline();motion.update([],1.,1.,0.)
    delta=motion.update([],1.1,1.,.2)
    assert delta[2]==pytest.approx(-.02)
    assert motion.diagnostics['rejected_reason'] is None
    assert fractional_motion(delta,.5)[2]==pytest.approx(-.01)


def test_one_frame_recorded_reverse_spike_falls_back_without_rear_id_birth(monkeypatch):
    import team10_race.opponent as module
    motion=timeline();tracker=OpponentTracker()
    def register(previous,current,dt,*args):
        return (.345835,.004354,-.108476) if current==['spike'] else (-dt,0.,0.)
    monkeypatch.setattr(module,'estimate_tracking_motion',register)
    for i in range(1,15):
        stamp=i*.099
        delta=motion.update(['spike'] if i==9 else [],stamp,1.,0.)
        tracks=tracker.update([candidate(-6+i*.099,.1,stamp)],.099,
                              timestamp=stamp,ego_motion=delta,motion_timeline=motion)
        if i==9:
            assert motion.diagnostics['rejected_reason']=='reverse_translation'
            assert delta==pytest.approx((-.099,0.,0.))
            assert motion.diagnostics['previous_valid']['timestamp']==pytest.approx(8*.099)
        if i>=4:
            assert [t.track_id for t in tracks]==[1]
        assert len(tracker._tracks)==1


def test_spike_pairing_advances_cloud_and_short_fallback_expires(monkeypatch):
    import team10_race.opponent as module
    pairs=[]
    def register(previous,current,dt,*args):
        pairs.append((previous,current,dt))
        return (.4,0.,0.) if current!=[1] else (-dt,0.,0.)
    monkeypatch.setattr(module,'estimate_tracking_motion',register)
    motion=timeline();motion.update([0],0.,1.,0.)
    motion.update([1],.1,1.,0.)
    motion.update([2],.2,1.,0.)
    motion.update([3],.5,2.,0.)
    assert pairs[-1][:2]==([2],[3])
    assert motion.diagnostics['fallback']=='command_seed'
    assert motion.diagnostics['velocity'][0]==pytest.approx(-2.)


@pytest.mark.parametrize('raw,reason',[
    ((-.6,0.,0.),'translation_speed_limit'),
    ((-.1,0.,1.),'yaw_rate_limit'),
    ((-.1,.30,.01),'lateral_kinematic_limit'),
])
def test_delta_limits_derive_from_car_kinematics(monkeypatch,raw,reason):
    import team10_race.opponent as module
    monkeypatch.setattr(module,'estimate_tracking_motion',lambda *args:raw)
    motion=timeline();motion.update([],1.,1.,0.)
    assert motion.update([],1.1,1.,0.)==pytest.approx((-.1,0.,0.))
    assert motion.diagnostics['rejected_reason']==reason


def test_multisensor_arrival_order_uses_acquisition_epoch_not_future_cache():
    from collections import deque
    results=[]
    for future_first in (False,True):
        cache=deque(maxlen=8)
        for stamp in (.967,1.):NavigationNode._cache_tracking_scan(cache,scan(stamp))
        if future_first:NavigationNode._cache_tracking_scan(cache,scan(1.033))
        selected=NavigationNode._tracking_scan_at(cache,1.,.15)
        results.append(selected.header.stamp)
        assert not NavigationNode._cache_tracking_scan(cache,scan(.933))
        assert not NavigationNode._cache_tracking_scan(cache,scan(1.))
    assert results[0].sec==results[1].sec==1
    assert results[0].nanosec==results[1].nanosec==0


def test_small_registration_lateral_noise_does_not_override_valid_turn(monkeypatch):
    import team10_race.opponent as module
    raw=(-.10189,-.01846,-.01490)
    monkeypatch.setattr(module,'estimate_tracking_motion',lambda *args:raw)
    motion=timeline();motion.update([],1.,1.5,.15)
    assert motion.update([],1.099,1.5,.15)==pytest.approx(raw)
    assert motion.diagnostics['rejected_reason'] is None


def test_cached_raw_recovery_is_aligned_before_lateral_gate():
    tracker=OpponentTracker()
    for i in range(1,7):
        tracker.update([candidate(-6+i*.2,.1,i*.1)],.1,timestamp=i*.1,
                       ego_motion=(-.1,0.,0.))
    # The new yaw makes old-frame y differ by >0.30 m. Former raw admission
    # discarded the true compact facet before compensating its acquisition.
    points=[ScanPoint(20+i,math.hypot(-4.6,.1+i*.02),Point2(-4.6,.1+i*.02))
            for i in range(3)]
    tracks=tracker.update([],.1,timestamp=.7,ego_motion=(-.1,0.,-.1),
                          raw_observations=[('rear',.61,points)])
    assert [t.track_id for t in tracks]==[1] and tracks[0].missed==0


def test_same_timestamp_sensor_duplicate_reuses_missed_id_instead_of_birth():
    tracker=OpponentTracker()
    for i in range(1,5):tracker.update([candidate(-3,.1,i*.1)],.1,timestamp=i*.1)
    tracker.update([],.05,timestamp=.45)
    tracker.update([candidate(-3,.1,.4,'secondary')],.05,timestamp=.5)
    assert list(tracker._tracks)==[1]
    assert tracker.diagnostics['candidates'][0]['matched_id']==1
    assert tracker.diagnostics['candidates'][0]['reason']=='existing_footprint_duplicate'


def test_two_nearby_opponents_stay_separate_through_spike_and_sensor_handoff(monkeypatch):
    import team10_race.opponent as module
    monkeypatch.setattr(module,'estimate_tracking_motion',
                        lambda prev,cur,dt,*args:(.346,0.,-.108) if cur==[5] else (-dt,0.,0.))
    motion=timeline();tracker=OpponentTracker()
    for i in range(1,13):
        stamp=i*.1;delta=motion.update([i],stamp,1.,0.)
        observations=[candidate(-5+i*.1,.0,stamp,'rear' if i<6 else 'primary'),
                      candidate(-5+i*.1,.48,stamp,'rear')]
        tracks=tracker.update(observations,.1,timestamp=stamp,ego_motion=delta)
        if i>=4:
            assert {t.track_id for t in tracks}=={1,2}
            assert len(tracker._tracks)==2


def test_saved_c_wall_visibility_spike_replays_deterministically_and_is_rejected():
    import json
    from pathlib import Path
    from team10_race.opponent import estimate_tracking_motion
    data=json.loads((Path(__file__).parent/'data/ego_wall_visibility_spike.json').read_text())
    previous=[Point2(*p) for p in data['clouds']['previous']]
    current=[Point2(*p) for p in data['clouds']['current']]
    yaw_rate=data['speed']*math.tan(data['steering'])/.340
    raw=estimate_tracking_motion(previous,current,data['dt'],data['speed'],yaw_rate)
    assert raw==pytest.approx(data['expected_raw_motion'],abs=1e-10)
    assert raw[0]>.10  # stationary walls apparently move forwards
    motion=timeline();motion.update(previous,1.,data['speed'],yaw_rate)
    corrected=motion.update(current,1.+data['dt'],data['speed'],yaw_rate)
    assert corrected[0]<0
    assert motion.diagnostics['rejected_reason']=='reverse_translation'
