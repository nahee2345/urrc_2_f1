import math
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.steering_guard import SteeringGuard,SteeringGuardConfig,ImuRotation,angle


def quaternion(deg):
    return (0.,0.,math.sin(math.radians(deg)/2),math.cos(math.radians(deg)/2))


def path(offset=0.,curve=0.):
    x=np.arange(.1,5.01,.1)
    return np.column_stack((x,offset+.5*curve*x*x))


def imu(g,t,yaw=0.,rate=0.):
    return g.imu.update(quaternion(yaw),math.radians(rate),t,t)


def camera(g,p,t,confidence=1.):
    g.camera_geometry(p,p+[0.,1.2],p-[0.,1.2],confidence,t,t)


def ready(yaw=0.,p=None,corner='STRAIGHT'):
    g=SteeringGuard(SteeringGuardConfig(yaw_safety_enabled=True));p=path() if p is None else p
    for t in (1.,1.01,1.02):imu(g,t,yaw)
    camera(g,p,1.02);g.set_corner(corner,1.02)
    assert g.accept_path(p,1.02,1.02)
    return g


def test_normal_straight_allows_seven_and_eight_degree_ceiling():
    g=ready()
    steering,cap,stop,d=g.evaluate(math.radians(24),np.array([1.,.5]),1.,0.,1.03,.02)
    assert math.degrees(steering)==pytest.approx(8.) and cap==7. and not stop
    assert d['state']=='STRAIGHT' and not d['straight_overrotation']


@pytest.mark.parametrize('sign',[-1,1])
def test_confirmed_near_corner_has_twenty_degree_authority(sign):
    g=ready(p=path(curve=sign*.2),corner='CORNER')
    steering,cap,stop,d=g.evaluate(sign*math.radians(27),np.array([1.,sign*.2]),1.,0.,1.03,.02)
    assert math.degrees(steering)==pytest.approx(sign*20.) and not stop
    assert d['state']=='CORNER'


def test_future_corner_approach_does_not_grant_twenty_on_straight():
    g=ready(corner='CORNER_APPROACH')
    steering,_,_,d=g.evaluate(math.radians(25),np.array([1.,.5]),1.,0.,1.03,.02)
    assert math.degrees(steering)<=8. and d['state']=='STRAIGHT'


def test_one_curvature_frame_without_confirmed_corner_keeps_straight_limit():
    g=ready(p=path(curve=.2),corner='STRAIGHT')
    assert math.degrees(g.evaluate(.5,np.array([1.,.2]),1.,0.,1.03,.02)[0])<=8.


def test_stale_corner_state_does_not_keep_twenty_degree_authority():
    g=ready(p=path(curve=.2),corner='CORNER')
    imu(g,1.3);camera(g,path(curve=.2),1.3)
    assert math.degrees(g.evaluate(.5,np.array([1.,.2]),1.,0.,1.3,.02)[0])<=8.


def test_corner_exit_restores_straight_limit_without_false_reverse_latch():
    g=ready(yaw=90.,p=path(curve=.2),corner='CORNER')
    g.was_corner=True;g.straight_yaw=0.
    g.set_corner('CORNER_EXIT',1.03)
    steering,_,stop,d=g.evaluate(.5,np.array([1.,.2]),1.,.1,1.03,.02)
    assert math.degrees(steering)<=8. and not stop and not d['reverse_risk']


def test_yaw_wrap_uses_shortest_difference():
    g=ready(yaw=179.)
    imu(g,1.03,-179.,2.)
    assert abs(math.degrees(angle(g.imu.yaw-math.radians(179.))))<2.1
    assert abs(math.degrees(g.imu.unwrapped)-179.)<2.1


def test_single_250_degree_rate_spike_is_held_then_normal_resumes():
    g=ready()
    imu(g,1.03,0.,250.)
    assert g.imu.spike_suspect and abs(g.imu.rate)<1.
    _,cap,stop,d=g.evaluate(.1,np.array([1.,.1]),3.,.05,1.03,.02)
    assert cap==7. and not stop and not d['straight_overrotation']
    imu(g,1.04,0.,7.)
    assert abs(g.imu.rate)<10.


def test_sustained_extreme_rate_is_not_discarded():
    g=ready()
    imu(g,1.03,2.5,250.);imu(g,1.04,5.,250.)
    assert g.imu.rate>=70.


def test_single_orientation_spike_is_not_a_reverse_turn():
    g=ready()
    imu(g,1.03,90.,0.)
    assert abs(math.degrees(g.imu.yaw))<1
    imu(g,1.04,0.,0.)
    assert abs(math.degrees(g.imu.yaw))<1


@pytest.mark.parametrize('sign',[-1,1])
def test_growing_straight_rotation_relaxes_steering_and_caps_recovery(sign):
    g=ready();current=sign*math.radians(8)
    for i in range(1,12):
        t=1.02+i*.01;imu(g,t,sign*i*.6,sign*60.)
        steering,cap,stop,d=g.evaluate(sign*.4,np.array([1.,sign*.5]),2.,current,t,.01)
        current=steering
    assert cap<=2. and abs(steering)<math.radians(8.)
    assert d['state'] in ('RECOVERY','STOP')
    assert steering*sign>=0  # Never manufactures opposite full steering.


def test_imu_does_not_generate_steering_when_pp_is_zero():
    g=ready()
    for i in range(1,12):
        t=1.02+i*.01;imu(g,t,i*.6,60.)
        steering,_,_,_=g.evaluate(0.,np.array([1.,0.]),2.,.1,t,.01)
        assert steering==0.


@pytest.mark.parametrize('sign',[-1,1])
def test_reverse_stop_occurs_before_ninety_degrees(sign):
    g=ready();first=None
    for i in range(1,30):
        t=1.02+i*.01;imu(g,t,sign*i*3.,sign*300.)
        _,_,stop,d=g.evaluate(sign*.2,np.array([1.,sign*.2]),2.,sign*.1,t,.01)
        if stop:first=i*3.;break
    assert first is not None and first<90.,first
    assert g.stop_latched


def test_high_speed_prediction_stops_before_braking_consumes_heading_margin():
    g=ready()
    stopped=False
    for i in range(1,30):
        t=1.02+i*.01;imu(g,t,i*.6,60.)
        _,_,stop,d=g.evaluate(.2,np.array([1.,.2]),7.,.1,t,.01)
        if stop:stopped=True;break
    assert stopped and d['reason']=='PREDICTED_OVERROTATION'


def test_wrong_direction_signed_comparison_is_symmetric():
    for sign in (-1,1):
        g=ready()
        for i in range(1,25):
            t=1.02+i*.01;imu(g,t,sign*i,sign*100.)
        _,_,_,d=g.evaluate(sign*.2,np.array([1.,sign*.2]),2.,sign*.1,t,.01)
        assert d['wrong_direction_steering']
        _,_,_,d=g.evaluate(-sign*.2,np.array([1.,-sign*.2]),2.,0.,t,.01)
        assert not d['wrong_direction_steering']


def test_target_jump_is_held_until_two_distinct_camera_frames():
    g=ready();p=path(offset=.65)
    assert not g.accept_path(p,1.03,1.03)
    assert g.path_hold and g.target_jump
    assert not g.accept_path(p,1.03,1.05)  # timer/republication cannot confirm
    assert g.accept_path(p,1.06,1.06)
    assert not g.path_hold


def test_single_target_outlier_returns_to_previous_valid_path():
    g=ready()
    assert not g.accept_path(path(offset=.65),1.03,1.03)
    assert g.accept_path(path(offset=.02),1.06,1.06)
    assert not g.path_hold
    assert abs(g.last_features['lateral']-.02)<1e-8


def test_heading_jump_is_rejected_before_pp():
    g=ready();p=path();p[:,1]=1.3*p[:,0]
    assert not g.accept_path(p,1.03,1.03)
    assert g.heading_jump


def test_replayed_bad_path_does_not_become_valid_after_hold_expiry():
    g=ready();p=path(offset=.65)
    assert not g.accept_path(p,1.03,1.03)
    assert not g.accept_path(p,1.03,1.4)
    assert g.evaluate(.1,np.array([1.,0.]),2.,.1,1.4,.02)[2]


def test_relative_path_heading_is_transformed_to_world_at_its_stamp():
    g=ready(yaw=90.)
    assert math.degrees(g.reference)==pytest.approx(90.)
    assert abs(g.evaluate(0.,np.array([1.,0.]),7.,0.,1.03,.02)[3]['heading_error_deg'])<1e-8


def test_recovery_exit_counts_camera_frames_not_control_ticks():
    g=ready();g.recovery=True;g.exit_count=0
    for _ in range(10):g.evaluate(0.,np.array([1.,0.]),2.,0.,1.03,.001)
    assert g.recovery
    for i in range(1,6):
        t=1.02+i*.03;imu(g,t);camera(g,path(),t);g.accept_path(path(),t,t)
        assert g.recovery==(i<5)


def test_camera_or_command_dropouts_do_not_reset_reverse_reference():
    g=ready();g.stop_latched=True;g.recovery=True;old=g.reference
    g.evaluate(0.,None,0.,0.,1.4,.02)
    assert g.reference==old and g.stop_latched


def test_new_visual_race_cycle_can_explicitly_reset_reference():
    g=ready();g.stop_latched=True;g.reset_mission()
    assert not g.stop_latched and g.reference is None


@pytest.mark.parametrize('bad',[(0.,0.,0.,0.),(0.,0.,float('nan'),1.),(0.,0.,0.,2.)])
def test_invalid_quaternion_disables_guard_feedback_and_caps_speed(bad):
    g=ready();assert not g.imu.update(bad,0.,1.03,1.03)
    _,cap,stop,d=g.evaluate(.5,np.array([1.,.2]),7.,0.,1.03,.02)
    assert cap<=2. and d['state']=='IMU_STALE'


def test_stale_imu_stops_after_bounded_recovery_period():
    g=ready()
    assert g.evaluate(.1,np.array([1.,.1]),7.,0.,1.4,.02)[1]<=2.
    assert g.evaluate(.1,np.array([1.,.1]),7.,0.,2.2,.02)[2]


def test_replayed_imu_does_not_renew_freshness():
    g=ready();assert not imu(g,1.02)
    assert not g.imu.fresh(1.4)


def test_unavailable_angular_velocity_is_not_trusted():
    g=ready();g.imu.update(quaternion(0),0.,1.03,1.03,available=False)
    assert not g.imu.fresh(1.03)


def test_invalid_forward_target_stops():
    g=ready()
    assert g.evaluate(.1,np.array([-1.,.2]),2.,.1,1.03,.02)[2]
    with pytest.raises(ValueError):g.accept_path(np.array([[-1.,0.],[1.,0.],[2.,0.]]),1.03,1.03)


def test_disable_restores_legacy_proposal_without_imu_cap():
    c=SteeringGuardConfig(enabled=False);g=SteeringGuard(c)
    steering,cap,stop,d=g.evaluate(.4,np.array([1.,0.]),7.,0.,1.,.02)
    assert steering==.4 and cap==7. and not stop and d['state']=='DISABLED'


def test_braking_hold_does_not_reopen_low_speed_steering_after_command_zero():
    g=ready();g.braking_hold=True;g.recovery=True
    steering,cap,stop,d=g.evaluate(.5,np.array([1.,.5]),0.,0.,1.03,.02,motion_speed=6.)
    assert stop and steering==0. and cap==0.
    assert d['braking_stability_hold'] and d['motion_speed_bound']==6.
    for i in range(1,7):
        t=1.02+i*.03;imu(g,t);camera(g,path(),t);g.accept_path(path(),t,t)
    assert g.braking_hold  # stable images alone do not prove braking finished
    g.evaluate(0.,np.array([1.,0.]),0.,0.,1.21,.02,motion_speed=1.9)
    for i in range(5):
        t=1.23+i*.02;imu(g,t);camera(g,path(),t);g.accept_path(path(),t,t)
    assert not g.braking_hold


def test_a_new_red_signal_does_not_erase_physical_braking_hold():
    g=ready();g.braking_hold=True;g.reset_mission()
    assert g.braking_hold


def test_stable_braking_can_resume_bounded_pp_without_heading_alignment_deadlock():
    g=ready();g.braking_hold=True;g.recovery=True
    for i in range(1,15):
        t=1.02+i*.02;imu(g,t,20.,0.)
        p=path();p[:,1]=-math.tan(math.radians(20.))*p[:,0]
        camera(g,p,t);g.accept_path(p,t,t)
        g.evaluate(-.1,np.array([1.,-.3]),0.,0.,t,.02,motion_speed=0.)
    assert not g.braking_hold
    assert g.recovery
    steering,cap,stop,d=g.evaluate(-.3,np.array([1.,-.3]),0.,0.,t,.02,motion_speed=0.)
    assert not stop and cap==2. and -math.radians(8.)<=steering<0.


@pytest.mark.parametrize('sign',[-1,1])
def test_confirmed_sharp_corner_transition_updates_course_without_false_wrong_direction(sign):
    g=ready();x=path()[:,0];a=np.minimum(x,.9)
    y=sign*((1-np.sqrt(1-a*a))+np.maximum(0,x-.9)*.9/math.sqrt(1-.9**2))
    p=np.column_stack((x,y))
    for i in range(1,25):
        t=1.02+i*.02;imu(g,t,sign*i,sign*50.)
        camera(g,p,t);g.set_corner('CORNER',t);g.accept_path(p,t,t)
        _,_,stop,d=g.evaluate(sign*.32,np.array([1.,sign*.5]),1.,sign*.25,t,.02)
        assert not stop and not g.recovery,d
        assert d['state']=='CORNER' and not d['wrong_direction_steering']


def test_sharp_corner_exit_message_before_camera_packet_does_not_false_reverse():
    g=ready();x=path()[:,0];a=np.minimum(x,.9)
    p=np.column_stack((x,(1-np.sqrt(1-a*a))+np.maximum(0,x-.9)*.9/math.sqrt(1-.9**2)))
    imu(g,1.03);camera(g,p,1.03);g.set_corner('CORNER',1.03);g.accept_path(p,1.03,1.03)
    g.set_corner('CORNER_EXIT',1.04)
    _,_,stop,d=g.evaluate(.3,np.array([1.,.5]),1.,.3,1.04,.02)
    assert not stop and d['state_steering_limit_deg']==8. and d['awaiting_corner_exit_geometry'],d
    imu(g,1.05);camera(g,path(),1.05)
    assert not g.exit_reanchor
    assert abs(g.reference)<1e-8


def test_stationary_gyro_numerical_noise_does_not_count_as_heading_growth():
    g=ready();g.reference=math.radians(20.)
    for i in range(1,10):
        t=1.02+i*.01;imu(g,t,0.,-.00001)
        _,_,_,d=g.evaluate(-.1,np.array([1.,-.1]),1.,-.1,t,.01)
        assert not d['straight_overrotation']


def test_pp_counter_steer_is_not_suppressed_by_old_same_direction_steering():
    g=ready();g.recovery=True
    for i in range(1,10):imu(g,1.02+i*.01,i*.6,60.)
    steering,cap,stop,d=g.evaluate(-.1,np.array([1.,-.2]),1.,.1,1.12,.02)
    assert steering<0 and abs(steering)<=math.radians(8.) and not stop


def test_flat_path_reference_cannot_follow_slow_spin():
    g=ready()
    for i in range(1,120):
        t=1.02+i*.05;imu(g,t,i*.2,4.);camera(g,path(),t);g.accept_path(path(),t,t)
        g.evaluate(.15,np.array([1.,.4]),2.,.1,t,.05)
    assert abs(math.degrees(g.reference))<1.
    assert g.recovery


def test_camera_confirmed_gentle_bend_can_change_course_without_false_reverse():
    g=ready();p=path(curve=.03)
    for i in range(1,150):
        t=1.02+i*.05;imu(g,t,i*.6,12.);camera(g,p,t);g.accept_path(p,t,t)
        _,_,stop,d=g.evaluate(.015,np.array([2.,.06]),7.,.015,t,.05)
        assert not stop and not g.recovery,d
    assert math.degrees(g.reference)>60.


def test_first_avoidance_path_does_not_replace_camera_road_course_reference():
    g=SteeringGuard(SteeringGuardConfig(yaw_safety_enabled=True))
    for t in (1.,1.01,1.02):imu(g,t)
    camera(g,path(),1.02)
    assert g.accept_path(path(curve=.2),1.02,1.02)
    assert abs(g.reference)<1e-8


def test_selected_avoidance_turn_is_not_wrong_direction_against_road_anchor():
    g=ready()
    for i in range(1,16):
        t=1.02+i*.1;imu(g,t,i,10.)
    t+=.01
    base=path();base[:,1]=-math.tan(math.radians(15.))*base[:,0]
    selected=base.copy();selected[:,1]=np.interp(selected[:,0],[.1,1.5,3.,5.],[.02,.30,-.30,base[-1,1]])
    camera(g,base,t)
    for j in range(3):
        tt=t+j*.02;imu(g,tt,15.,10.);camera(g,base,tt);g.accept_path(selected,tt,tt)
    _,_,stop,d=g.evaluate(.1,np.array([1.5,.3]),2.,.1,tt,.02)
    assert not g.recovery and not stop and not d['wrong_direction_steering'],d


def test_recovery_does_not_restart_same_wrong_turn_when_yaw_rate_settles():
    g=ready()
    for i in range(1,50):imu(g,1.02+i*.02,20.,0.)
    t=2.02;camera(g,path(),t);g.recovery=True
    g.accept_path(path(offset=1.5),t,t)
    g.accept_path(path(offset=1.5),t+.02,t+.02)
    steering,cap,stop,d=g.evaluate(.3,np.array([1.,1.5]),2.,0.,t+.02,.02)
    assert steering==0. and cap==2. and not stop,d


def test_corner_permission_survives_one_flat_frame_until_confirmed_exit():
    g=ready(p=path(curve=.2),corner='CORNER')
    imu(g,1.05);camera(g,path(),1.05);g.set_corner('CORNER',1.05)
    assert g.corner_authorized(1.05)
    g.accept_path(path(),1.05,1.05)
    _,_,stop,d=g.evaluate(0.,np.array([1.,0.]),1.,0.,1.05,.02)
    assert not stop and d['state_steering_limit_deg']==20.
    g.set_corner('CORNER_EXIT',1.06)
    assert not g.corner_authorized(1.06)
    assert g.evaluate(0.,np.array([1.,0.]),1.,0.,1.06,.02)[3]['state_steering_limit_deg']==8.


@pytest.mark.parametrize('sign',[-1,1])
def test_high_speed_rising_yaw_brakes_before_fixed_rate_threshold(sign):
    g=ready();current=sign*math.radians(1.)
    recovery=None
    for i,rate in enumerate((3.,6.,10.,15.,20.,24.),1):
        t=1.02+.01*i
        imu(g,t,sign*.1*i,sign*rate)
        bounded,cap,stop,d=g.evaluate(current,np.array([2.5,.1*sign]),7.,current,t,.01)
        if cap<=2.:
            recovery=d;break
    assert recovery is not None
    assert abs(recovery['imu_yaw_rate_filtered_deg_s'])<40.
    assert recovery['reason']=='YAW_LATERAL_ACCELERATION'
    assert abs(bounded)<abs(current)
    assert not stop


@pytest.mark.parametrize('severity,speed,limit',[
    ('STRAIGHT',7.,8.),('MILD_CORNER',5.,14.),('MEDIUM_CORNER',3.5,20.),('SHARP_CORNER',2.5,25.)])
def test_camera_severity_owns_authority_without_any_imu(severity,speed,limit):
    g=SteeringGuard();p=path(curve=.35)
    for t in (1.,1.05):
        camera(g,p,t);g.set_severity(severity,speed,limit,t);g.accept_path(p,t,t)
    bounded,cap,stop,d=g.evaluate(math.radians(22.),np.array([1.,.5]),1.,0.,1.06,.02)
    assert math.degrees(bounded)==pytest.approx(min(22.,limit))
    assert cap==speed and not stop and not d['yaw_safety_enabled']


def test_camera_mode_yaw_and_gyro_cannot_change_a_confirmed_sharp_limit():
    g=SteeringGuard();p=path(curve=.35)
    for t in (1.,1.05):
        camera(g,p,t);g.set_severity('SHARP_CORNER',2.5,25.,t);g.accept_path(p,t,t)
        imu(g,t,170.,400.)
    bounded,cap,stop,d=g.evaluate(math.radians(22),np.array([1.,.5]),1.,0.,1.06,.02)
    assert math.degrees(bounded)==pytest.approx(22.) and cap==2.5 and not stop


def test_camera_mode_does_not_grant_large_steering_from_stale_or_low_confidence_geometry():
    g=SteeringGuard();p=path(curve=.35)
    camera(g,p,1.,.68);g.set_severity('SHARP_CORNER',2.5,25.,1.);g.accept_path(p,1.,1.)
    for t in (1.01,1.4):
        bounded,cap,stop,d=g.evaluate(.5,np.array([1.,.5]),1.,0.,t,.02)
        assert abs(math.degrees(bounded))<=8. and cap<=2.


def test_camera_mode_retains_single_frame_heading_jump_hold_in_sharp():
    g=SteeringGuard();p=path()
    camera(g,p,1.);g.set_severity('SHARP_CORNER',2.5,25.,1.);assert g.accept_path(p,1.,1.)
    x=p[:,0];bad=np.column_stack((x,1.3*x))
    assert not g.accept_path(bad,1.05,1.05)
    assert g.path_hold and g.heading_jump


def test_decision_validated_single_wall_keeps_confirmed_corner_authority():
 g=SteeringGuard();p=path(curve=.35)
 camera(g,p,1.,.68);g.set_severity('SHARP_CORNER',2.5,25.,1.);g.accept_path(p,1.,1.)
 g.single_wall_reacquired=True
 _,_,stop,d=g.evaluate_camera(.5,np.array([1.,.2]),1.01)
 assert not stop and d['state_steering_limit_deg']==25.
 g.single_wall_reacquired=False
 assert g.evaluate_camera(.5,np.array([1.,.2]),1.01)[3]['state_steering_limit_deg']==8.


def test_resampled_near_endpoint_does_not_create_false_curvature_jump():
    # Captured failure: centimetre changes at the first sample generated a
    # >0.5/m endpoint curvature spike while heading/lateral stayed unchanged.
    g=SteeringGuard();p=path()
    assert g.accept_path(p,1.,1.)
    moved=p.copy();moved[0,1]+=.012
    assert g.accept_path(moved,1.05,1.05)
    assert not g.path_hold


def test_supported_interior_curvature_jump_still_requires_distinct_frames():
    g=SteeringGuard();p=path()
    assert g.accept_path(p,1.,1.)
    x=p[:,0]
    # A genuine local bend keeps continuous heading/lateral but changes
    # curvature over several fully supported interior samples.
    bend=np.column_stack((x,.025*np.exp(-((x-.9)/.18)**2)))
    assert not g.accept_path(bend,1.05,1.05)
    assert g.path_hold
    assert not g.accept_path(bend,1.05,1.07)
    assert g.accept_path(bend,1.10,1.10)
    assert not g.path_hold


@pytest.mark.parametrize('start',[1.6,2.5,3.0])
def test_forward_lidar_prefix_beyond_near_window_remains_controllable(start):
    from team_01_race.steering_guard import path_features
    x=np.arange(start,5.01,.1);p=np.column_stack((x,np.zeros(len(x))))
    features=path_features(p)
    assert all(math.isfinite(v) for v in features.values())
    g=SteeringGuard()
    assert g.accept_path(p,1.,1.) and not g.path_hold


def test_millimetre_bev_resampling_noise_does_not_hold_a_continuous_road():
    rng=np.random.default_rng(1010);g=SteeringGuard();base=path(curve=.06)
    assert g.accept_path(base,1.,1.)
    for i in range(1,31):
        measured=base.copy();measured[:,1]+=rng.uniform(-.005,.005,len(measured))
        stamp=1.+i*.05
        assert g.accept_path(measured,stamp,stamp)
        assert not g.path_hold


@pytest.mark.parametrize('direction',[-1.,1.])
def test_guard_geometric_curvature_retains_true_two_metre_radius_turn(direction):
    from team_01_race.steering_guard import path_features
    theta=np.arange(.05,1.,.025)
    p=np.column_stack((2.*np.sin(theta),direction*2.*(1.-np.cos(theta))))
    features=path_features(p)
    assert features['curvature']==pytest.approx(.5,abs=.01)
    assert features['signed_curvature']==pytest.approx(direction*.5,abs=.01)
