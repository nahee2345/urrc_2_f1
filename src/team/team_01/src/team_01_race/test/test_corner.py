"""Camera-only severity, preview braking, hysteresis and limiter composition."""
import math
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.corner import CornerConfig,CornerDetector,SEVERITIES,preview_geometry
from team_01_race.navigation import pure_pursuit
from team_01_race.ramp import RampDetector


def path(k=0.,start=0.,offset=0.,slope=0.):
    x=np.arange(.1,5.01,.1)
    return np.column_stack((x,offset+slope*x+.5*k*np.maximum(0.,x-start)**2))


def base(speed=7.,enabled=True,mode='CENTER'):
    return dict(speed=speed,enabled=enabled,mode=mode,path=path(),active_limiter='EXISTING',reason='test')


def confirmed(k):
    d=CornerDetector()
    for i in range(3):d.update(path(k),i,i*.05,speed=2.5)
    return d


@pytest.mark.parametrize('sign',[-1,1])
@pytest.mark.parametrize('k,level,speed,steer',[(0.,0,7.,8.),(.1,1,5.,14.),(.2,2,3.5,20.),(.35,3,2.5,25.)])
def test_signed_severity_policy(sign,k,level,speed,steer):
    d=confirmed(sign*k);r=d.constrain(base(),.1,7.)
    assert r['corner_severity']==SEVERITIES[level]
    assert r['corner_speed_limit']==speed and r['corner_steering_limit_deg']==steer
    assert r['speed']==speed


@pytest.mark.parametrize('offset,slope',[(0.,0.),(.6,0.),(-.6,0.),(.3,.2),(-.3,-.2)])
def test_translation_and_constant_heading_are_straight(offset,slope):
    d=CornerDetector()
    for i in range(4):d.update(path(offset=offset,slope=slope),i,i*.05,speed=7.,steering_demand=.5)
    assert d.severity=='STRAIGHT'
    assert math.isinf(d.policy(.15,7.)['distance_to_corner'])


@pytest.mark.parametrize('sign',[-1,1])
def test_future_sharp_brakes_before_pp_turn_and_reports_short_horizon(sign):
    d=CornerDetector();p=path(sign*.7,start=4.)
    assert abs(pure_pursuit(p,7.,.34)[0])<1e-12
    d.update(p,1,0.,speed=7.)
    assert d.severity=='STRAIGHT'
    d.update(p,2,.05,speed=7.)
    r=d.constrain(base(),.05,7.)
    assert r['corner_severity']=='SHARP_CORNER' and r['corner_state']=='CORNER_APPROACH'
    assert r['speed']==2.5 and r['corner_braking']
    assert 3.5<r['distance_to_corner']<4.5
    assert r['required_braking_distance']==pytest.approx((49-6.25)/8)
    assert r['braking_margin']==pytest.approx(1.45)
    assert not r['braking_feasible'] and r['preview_braking_insufficient']


def test_confirmation_counts_source_frames_not_timer_or_replay():
    d=CornerDetector();p=path(.35);d.update(p,1,0.,speed=7.)
    for i in range(10):d.update(p,1,.01,speed=7.);d.policy(.01,7.)
    assert d.severity=='STRAIGHT' and d.enter_count==1
    d.reject(2);d.update(p,3,.1,speed=7.)
    assert d.severity=='STRAIGHT'
    d.update(p,4,.15,speed=7.);assert d.severity=='SHARP_CORNER'


@pytest.mark.parametrize('sign',[-1,1])
def test_fast_trigger_requires_large_curvature_and_heading(sign):
    d=CornerDetector();d.update(path(sign*.6),1,0.,speed=7.)
    assert d.severity=='SHARP_CORNER'
    c=CornerConfig(sharp_fast_heading_deg=85.)
    d=CornerDetector(c);d.update(path(sign*.6),1,0.,speed=7.)
    assert d.severity=='STRAIGHT'


def test_one_point_noise_cannot_create_sharp_and_curvature_alone_is_insufficient():
    p=path();p[25,1]=.04
    assert preview_geometry(p,CornerConfig())['candidate_level']==0
    # Strong curvature is visible only over a tiny far segment; heading does
    # not corroborate a sharp turn, so 25-degree authority is withheld.
    m=preview_geometry(path(1.,start=4.8),CornerConfig())
    assert m['candidate_level']<3


def test_gaps_low_confidence_invalid_geometry_cannot_confirm_or_clear():
    d=CornerDetector();d.update(path(.35),1,0.,speed=7.);d.update(path(.35),2,1.,speed=7.)
    assert d.severity=='STRAIGHT'
    d.update(path(.35),3,1.05,speed=7.);assert d.severity=='SHARP_CORNER'
    for i in range(4,20):d.update(path(),i,1.05+(i-3)*.05,speed=2.5,confidence=.68)
    d.update(None,20,2.,speed=2.5)
    assert d.severity=='SHARP_CORNER'


def test_exit_requires_three_stable_camera_frames_and_steps_down():
    d=confirmed(.35)
    d.update(path(),3,.15,speed=2.5)
    assert d.severity=='SHARP_CORNER'
    seen=[]
    for i in range(4,17):
        # Steering is deliberately large/unavailable: it cannot classify or
        # block a Camera geometry transition anymore.
        d.update(path(),i,i*.05,speed=2.5,steering_demand=.5,steering_fresh=False)
        seen.append(d.severity)
    assert 'MEDIUM_CORNER' in seen and 'MILD_CORNER' in seen and seen[-1]=='STRAIGHT'


def test_hysteresis_does_not_chatter_at_threshold():
    d=confirmed(.2)
    for i,k in enumerate((.17,.19,.17,.19,.17,.19),3):d.update(path(k),i,i*.05,speed=3.5)
    assert d.severity=='MEDIUM_CORNER'


def test_s_curve_next_bend_and_one_flat_frame_never_unlock_seven():
    d=confirmed(.35);x=np.arange(.1,5.01,.1)
    s=np.column_stack((x,.4*(1-np.cos(1.5*x))))
    for i in range(3,9):d.update(s,i,i*.05,speed=2.5)
    d.update(path(),9,.45,speed=2.5)
    assert d.constrain(base(),.45,2.5)['speed']<7.
    for i in range(10,14):d.update(s,i,i*.05,speed=2.5)
    assert d.level>0


def test_pass_path_never_classifies_the_track():
    d=confirmed(0.);result=base(2.,mode='LEFT_PASS');result['path']=path(.35)
    out=d.constrain(result,.1,2.)
    assert out['corner_severity']=='STRAIGHT' and out['mode']=='LEFT_PASS' and out['speed']==2.
    assert out['path'] is result['path']


def test_ramp_obstacle_ttc_and_emergency_min_policy():
    sharp=confirmed(.35);mild=confirmed(.1);ramp=RampDetector(normal_speed=7.)
    q=(0.,math.sin(math.radians(7)/2),0.,math.cos(math.radians(7)/2))
    for i in range(5):ramp.update(q,i*.05,i*.05)
    out=sharp.constrain(ramp.constrain(base(),.2),.2,7.)
    assert out['speed']==2.5 and out['ramp_speed_limit']==5.
    out=mild.constrain(ramp.constrain(base(2.,mode='RIGHT_PASS'),.2),.2,3.)
    assert out['speed']==2. and out['mode']=='RIGHT_PASS'
    out=sharp.constrain(base(0.,enabled=False,mode='EMERGENCY_STOP'),.2,7.)
    assert out['speed']==0. and not out['corner_braking']


def test_distance_age_and_actual_entry_overspeed_are_reported():
    d=confirmed(.35);r=d.policy(.2,7.)
    assert r['entry_overspeed'] and not r['braking_feasible']
    assert r['required_braking_distance']==pytest.approx(5.34375)


def test_disabled_preserves_existing_speed_and_default_physical_limit():
    d=CornerDetector(CornerConfig(enabled=False));d.update(path(.5),1,0.,speed=7.)
    assert d.constrain(base(),0.,7.)['speed']==7.
    assert d.c.physical_max_steering_deg==30.


def test_medium_releases_to_true_mild_without_requiring_a_straight_heading():
    d=confirmed(.2)
    mild=path(.1)
    assert preview_geometry(mild,d.c)['future_heading_change_deg']>d.c.medium_heading_exit_deg
    for i in range(3,8):d.update(mild,i,i*.05,speed=3.5)
    assert d.severity=='MILD_CORNER'
    assert d.policy(.35,3.5)['corner_speed_limit']==5.


def test_straight_release_requires_both_low_curvature_and_small_heading():
    d=confirmed(.1);p=path(.04)
    assert preview_geometry(p,d.c)['future_heading_change_deg']>d.c.mild_heading_exit_deg
    for i in range(3,14):d.update(p,i,i*.05,speed=5.)
    assert d.severity=='MILD_CORNER'


def test_single_rejected_jump_does_not_erase_preview_confirmation():
    d=CornerDetector();p=path(.35)
    d.update(p,1,0.,speed=7.)
    d.reject(2,transient=True)
    d.update(p,3,.06,speed=7.)
    assert d.severity=='SHARP_CORNER'
    # Rejecting frames does not fabricate geometry source timestamps.
    d.reject(4,transient=True)
    assert d.policy(.07,7.)['camera_source_stamp']==3


def test_confirmed_future_corner_commits_after_speed_is_ready():
    d=CornerDetector();p=path(.7,start=4.)
    d.update(p,1,0.,speed=7.);d.update(p,2,.03,speed=7.)
    assert d.state=='CORNER_APPROACH'
    d.update(p,3,.06,speed=2.5)
    assert d.state=='CORNER_COMMIT'


def test_short_view_cannot_clear_confirmed_bend_just_because_heading_is_hidden():
 d=confirmed(.35);x=np.arange(.1,1.21,.1);p=np.column_stack((x,.175*x*x))
 for i in range(3,14):d.update(p,i,i*.03,speed=2.5)
 assert d.severity=='SHARP_CORNER'


def test_partial_sharp_view_brakes_without_granting_unconfirmed_steering():
 d=CornerDetector();p=path(.35,start=3.5)
 d.update(p,1,0.,speed=7.)
 out=d.constrain(base(),0.,7.)
 assert out['speed']==2.5 and out['corner_braking']
 assert out['corner_steering_limit_deg']==8.  # Still awaiting severity confirmation.


def test_straight_road_heading_is_not_a_completed_corner_exit_until_aligned():
 d=confirmed(.35)
 for i in range(3,18):d.update(path(slope=.4),i,i*.03,speed=2.)
 assert d.severity=='SHARP_CORNER' and d.state!='STRAIGHT'
 for i in range(18,35):d.update(path(),i,i*.03,speed=2.)
 assert d.severity=='STRAIGHT'
