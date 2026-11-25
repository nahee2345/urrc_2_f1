import math
from pathlib import Path
import sys
import cv2
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from team_01_race.vision import VisionConfig, CameraVision, TrafficLightDetector
from team_01_race.navigation import valid_path, DecisionEngine, Obstacle, ObstacleTracker, pure_pursuit, scan_candidates
from team_01_race.geometry import vehicle_geometry


def image_fixture(red=False, green=False):
    camera = CameraVision(VisionConfig())
    k = np.array([[320.,0,320],[0,320,240],[0,0,1]])
    image = np.zeros((480,640,3), np.uint8)
    image[:] = (100,170,100)
    road = camera.project([[.3,1.2],[.3,-1.2],[30,-1.2],[30,1.2]], k).astype(np.int32)
    cv2.fillPoly(image, [road], (65,65,65))
    for y in [-1.2,1.2]:
        cv2.polylines(image, [camera.project([[.3,y],[30,y]], k).astype(np.int32)], False, (255,255,255), 6)
    for i in range(5):
        cv2.circle(image, (275+22*i, 70), 7, (0,0,255) if red else (0,255,0) if green else (0,0,0), -1)
    return image, k


def corridor(offset=0., curve=0., half=1.2):
    x = np.arange(.1, 5.01, .1)
    path = np.column_stack((x, offset+curve*x*x))
    left, right = path.copy(), path.copy()
    left[:,1] += half
    right[:,1] -= half
    return path, left, right


def ready_engine(**kwargs):
    e = DecisionEngine(vehicle_width=.242, wheelbase=.34, require_official=False, **kwargs)
    e.controller_ready = True
    e.set_light('LIGHTS_OUT')
    p, l, r = corridor()
    for i in range(3):
        e.camera(p,l,r,1.,(True,True),i,i*.05)
        e.lidar([],i*.05)
    return e


def test_full_straight_camera_path_can_request_ten_metres_per_second():
    e=ready_engine(cruise=10.)
    out=e.step(.1)
    assert out['enabled'] and out['speed']==pytest.approx(10.)
    assert out['base_speed_limiter']=='CRUISE'


def test_endpoint_resampling_noise_does_not_set_speed_from_a_derivative_spike():
    from team_01_race.navigation import path_geometry
    e=ready_engine(cruise=10.)
    p=e.path.copy();p[0,1]+=.012
    e.path=p;e.last_selected=None
    out=e.step(.1)
    raw=float(np.max(abs(path_geometry(out['path'])[1])))
    assert out['supported_path_curvature']<raw
    assert out['speed']>math.sqrt(e.max_lateral_accel/raw)
    assert out['speed']**2*out['supported_path_curvature']<=e.max_lateral_accel+1e-8


def test_real_tight_corner_keeps_lateral_acceleration_limit_below_configured_cap():
    e=ready_engine(cruise=10.)
    for i in range(3):
        stamp=1.+i*.05;p,left,right=corridor(curve=.25)
        e.camera(p,left,right,1.,(True,True),stamp,stamp,stamp=stamp);e.lidar([],stamp,stamp=stamp)
    out=e.step(stamp)
    assert out['enabled'] and 0.<out['speed']<3.
    assert out['speed']**2*out['supported_path_curvature']<=e.max_lateral_accel+1e-8


def test_dark_without_seen_red_never_grants_start():
    d = TrafficLightDetector(VisionConfig())
    hsv = np.zeros((480,640,3),np.uint8)
    for _ in range(10):
        assert d.update(hsv) == 'UNKNOWN'


def test_five_red_then_all_off_requires_confirmation():
    d = TrafficLightDetector(VisionConfig())
    red,k = image_fixture(red=True)
    dark,_ = image_fixture()
    for _ in range(2):
        assert d.update(cv2.cvtColor(red,cv2.COLOR_BGR2HSV)) != 'LIGHTS_OUT'
    assert d.update(cv2.cvtColor(red,cv2.COLOR_BGR2HSV)) == 'RED'
    assert len(d.armed_boxes) == 5
    partial = red.copy()
    cv2.circle(partial,(275,70),8,(0,0,0),-1)
    for _ in range(4):
        assert d.update(cv2.cvtColor(partial,cv2.COLOR_BGR2HSV)) == 'RED'
    for _ in range(2):
        assert d.update(cv2.cvtColor(dark,cv2.COLOR_BGR2HSV)) == 'RED'
    assert d.update(cv2.cvtColor(dark,cv2.COLOR_BGR2HSV)) == 'LIGHTS_OUT'
    for _ in range(3):
        state = d.update(cv2.cvtColor(red,cv2.COLOR_BGR2HSV))
    assert state == 'RED'


def test_green_is_detected_but_not_substituted_for_red_lights_out():
    d = TrafficLightDetector(VisionConfig())
    image,_ = image_fixture(green=True)
    for _ in range(3):
        state = d.update(cv2.cvtColor(image,cv2.COLOR_BGR2HSV))
    assert state == 'GREEN'
    e = ready_engine()
    e.set_light('RED')
    e.set_light('GREEN')
    assert not e.visual_go


def test_bev_centerline_from_camera_and_optical_sign():
    c = CameraVision(VisionConfig())
    image,k = image_fixture()
    result = c.process(image,k,np.zeros(5),0)
    assert result['observed'] == (True,True)
    assert result['confidence'] == 1.0
    assert len(result['path']) == 50
    assert np.max(np.abs(result['path'][:,1])) < .03
    assert c.project([[2.,.5]], k)[0,0] < c.project([[2.,-.5]], k)[0,0]
    assert result['bev'].ndim == 3


def test_blank_camera_does_not_invent_a_path():
    c = CameraVision(VisionConfig())
    _,k = image_fixture()
    r = c.process(np.zeros((480,640,3),np.uint8),k,np.zeros(5),0)
    assert r['path'] is None
    assert r['confidence'] == 0


def test_three_distinct_valid_camera_frames_required():
    e = DecisionEngine(vehicle_width=.242,wheelbase=.34,require_official=False)
    e.controller_ready = True
    e.set_light('LIGHTS_OUT')
    p,l,r = corridor()
    e.lidar([],0)
    for i in range(2):
        e.camera(p,l,r,1.,(True,True),i,i*.05)
        assert not e.step(i*.05)['enabled']
    e.camera(p,l,r,1.,(True,True),1,.07)
    assert e.ready_count == 2
    e.camera(p,l,r,1.,(True,True),2,.10)
    assert e.step(.10)['enabled']
    assert e.step(.10)['speed'] == pytest.approx(10.0)


def test_invalid_frame_resets_start_counter():
    e = DecisionEngine(vehicle_width=.242,wheelbase=.34,require_official=False)
    e.set_light('LIGHTS_OUT')
    p,l,r = corridor()
    e.camera(p,l,r,1.,(True,True),1,0)
    e.camera(np.empty((0,2)),l,r,0.,(False,False),2,.05)
    assert e.ready_count == 0


def test_official_interlock_and_red_stop():
    e = DecisionEngine(vehicle_width=.242,wheelbase=.34,require_official=True)
    e.controller_ready = True
    e.set_light('LIGHTS_OUT')
    p,l,r = corridor()
    for i in range(3):
        e.camera(p,l,r,1.,(True,True),i,i*.05)
        e.lidar([],i*.05)
    assert not e.step(.10)['enabled']
    e.set_official(True)
    for i in range(3,6):
        e.camera(p,l,r,1.,(True,True),i,i*.05)
        e.lidar([],i*.05)
    assert e.step(.25)['enabled']
    e.set_light('RED')
    assert e.step(.26)['speed'] == 0
    assert e.step(.26)['mode'] == 'STOP_RED'


def test_lidar_loss_does_not_replace_or_erase_camera_path():
    e = ready_engine()
    p = e.path.copy()
    assert e.step(.15)['enabled']
    assert e.step(.40)['reason'] == 'lidar_timeout'
    assert np.array_equal(e.path,p)


def test_camera_history_expires_and_no_lidar_path_fallback_exists():
    e = ready_engine()
    e.lidar([], .65)
    assert e.step(.65)['speed'] == 0
    assert e.step(.65)['reason'] == 'camera_path_lost'


def test_no_obstacle_always_center_and_wall_false_positive_rejected():
    e = ready_engine()
    e.lidar([Obstacle('wall',2.5,1.15,.2,.3,1.)], .1)
    result = e.step(.1)
    assert result['mode'] == 'CENTER'
    assert result['obstacle_count'] == 0


def test_gap_clearance_hysteresis_and_smooth_return():
    e = ready_engine()
    obstacle = Obstacle('car',2.5,0,.3,.3,1.)
    e.lidar([obstacle], .1)
    for _ in range(10):
        result = e.step(.1)
    assert result['mode'] in ('LEFT_PASS','RIGHT_PASS')
    side = result['mode']
    assert result['left_gap'] >= e.width+2*e.margin
    assert e.step(.1)['mode'] == side
    old = e.last_selected.copy()
    e.lidar([], .15)
    assert e.step(.15)['mode'] == side
    p,l,r = corridor()
    e.update_motion(.15,(2.,0.,0.))
    previous=e.last_selected.copy();max_step=0.
    for i in range(10):
        now=.15+(i+1)*.2
        e.update_motion(now,(2.,0.,0.))
        previous=e.last_selected.copy()
        e.camera(p,l,r,1.,(True,True),99+i,now)
        e.lidar([],now,stamp=now)
        result=e.step(now)
        if result['path'] is not None:
            common=(result['path'][:,0]>=previous[0,0])&(result['path'][:,0]<=previous[-1,0]-.5)
            if common.any():max_step=max(max_step,float(np.max(abs(result['path'][common,1]-np.interp(result['path'][common,0],previous[:,0],previous[:,1])))))
    assert result['mode'] == 'MERGE_CENTER'
    assert np.max(np.abs(result['path'][:,1])) < np.max(np.abs(old[:,1]))
    assert max_step < .3


def test_narrow_gap_stops_instead_of_squeezing():
    e = ready_engine()
    p,l,r = corridor(half=.5)
    e.path,e.left,e.right = p,l,r
    e.lidar([Obstacle('car',2.,0,.6,.3,1.)], .1)
    assert e.step(.1)['mode'] == 'STOP_BLOCKED'
    assert not e.step(.1)['enabled']


def test_critical_ttc_emergency():
    e = ready_engine()
    e.actual_speed = 10.
    e.lidar([Obstacle('car',1.5,0,.3,.3,1.,10.)], .1)
    assert e.step(.1)['mode'] == 'EMERGENCY_STOP'


def test_pure_pursuit_sign_dynamic_lookahead_and_invalid_target():
    left,_,_ = corridor(offset=.2)
    right,_,_ = corridor(offset=-.2)
    dl,_,low = pure_pursuit(left,0,.34)
    dr,_,high = pure_pursuit(right,10,.34)
    assert dl > 0 and dr < 0
    assert high > low
    bad = left.copy();bad[0,0] = -1
    with pytest.raises(ValueError):pure_pursuit(bad,0,.34)
    bad = left.copy();bad[4,1] = np.nan
    with pytest.raises(ValueError):pure_pursuit(bad,0,.34)


def test_tracker_requires_confirmation_and_expires():
    tracker = ObstacleTracker()
    candidate = [(3.,0.,.3,.1)]
    assert tracker.update(candidate,0) == []
    assert tracker.update(candidate,.05) == []
    assert len(tracker.update(candidate,.10)) == 1
    assert tracker.update([], .50) == []


def test_lidar_filter_has_no_wall_or_centerline_output():
    ranges = [math.inf]*181
    valid,candidates = scan_candidates(ranges,-math.pi/2,math.pi/180,.04,6.,lidar_x=.195,lidar_y=0,lidar_yaw=0)
    assert valid and candidates == []
    valid,_ = scan_candidates([math.nan]*181,-math.pi/2,math.pi/180,.04,6.,lidar_x=.195,lidar_y=0,lidar_yaw=0)
    assert not valid


def test_geometry_read_from_actual_sdf():
    root = Path(__file__).resolve().parents[1]
    g = vehicle_geometry(root/'models/team_01_car/model.sdf')
    assert .24 < g['vehicle_width_m'] < .25
    assert g['camera_pose'][2] > g['lidar_pose'][2]
    assert g['max_steering_angle_rad'] == pytest.approx(.523599)


def test_small_red_wall_fragment_cannot_prevent_verified_lights_out():
    d = TrafficLightDetector(VisionConfig())
    red,_ = image_fixture(red=True)
    dark,_ = image_fixture()
    cv2.rectangle(dark,(520,115),(530,130),(0,0,255),-1)
    for _ in range(3):d.update(cv2.cvtColor(red,cv2.COLOR_BGR2HSV))
    for _ in range(3):state=d.update(cv2.cvtColor(dark,cv2.COLOR_BGR2HSV))
    assert state == 'LIGHTS_OUT'


def test_image_codec_rgb_stride_and_invalid_data():
    from sensor_msgs.msg import Image
    from team_01_race.image_codec import to_bgr,from_bgr
    message=Image();message.width=2;message.height=2;message.step=8;message.encoding='rgb8'
    message.data=bytes([255,0,0,0,255,0,99,99,0,0,255,255,255,255,99,99])
    bgr=to_bgr(message)
    assert bgr[0,0].tolist() == [0,0,255]
    assert bgr[1,0].tolist() == [255,0,0]
    assert np.array_equal(to_bgr(from_bgr(bgr)),bgr)
    message.data=bytes([1,2])
    with pytest.raises(ValueError):to_bgr(message)


def test_four_lamps_plus_large_red_wall_cannot_rearm_wrong_locations():
    d = TrafficLightDetector(VisionConfig())
    red,_ = image_fixture(red=True)
    for _ in range(3):d.update(cv2.cvtColor(red,cv2.COLOR_BGR2HSV))
    armed = list(d.armed_boxes)
    partial=red.copy()
    cv2.circle(partial,(275,70),8,(0,0,0),-1)
    cv2.rectangle(partial,(510,40),(540,85),(0,0,255),-1)
    for _ in range(5):d.update(cv2.cvtColor(partial,cv2.COLOR_BGR2HSV))
    assert d.armed_boxes == armed
    dark,_ = image_fixture()
    cv2.rectangle(dark,(510,40),(540,85),(0,0,255),-1)
    for _ in range(3):state=d.update(cv2.cvtColor(dark,cv2.COLOR_BGR2HSV))
    assert state == 'LIGHTS_OUT'


def test_camera_reacquires_current_road_after_bad_previous_lateral_seed():
    c = CameraVision(VisionConfig())
    c.previous_path = corridor(offset=1.0)[0]
    image = cv2.imread(str(Path(__file__).parent/'fixtures/observed_corridor.png'))
    k = np.array([[320.,0,320],[0,320,240],[0,0,1]])
    result = c.process(image,k,np.zeros(5),0)
    result = c.process(image,k,np.zeros(5),.05)
    assert result['path'] is not None
    assert float(np.interp(1.6,result['path'][:,0],result['path'][:,1])) < 0


def test_one_bad_camera_frame_uses_bounded_history_without_rearming():
    e = ready_engine()
    p,l,r = corridor()
    e.camera(np.empty((0,2)),l,r,0.,(False,False),3,.15)
    e.lidar([], .15)
    assert e.step(.15)['enabled']
    e.camera(p,l,r,1.,(True,True),4,.20)
    e.lidar([], .20)
    assert e.step(.20)['enabled']


def test_one_camera_wall_uses_recent_measured_width_with_lower_confidence():
    c = CameraVision(VisionConfig())
    image,k = image_fixture()
    assert c.process(image,k,np.zeros(5),0)['confidence'] == 1.0
    one = cv2.imread(str(Path(__file__).parent/'fixtures/left_boundary_visible.png'))
    cv2.rectangle(one,(440,130),(639,300),(0,0,0),-1)
    c.previous_path = None
    result = c.process(one,k,np.zeros(5),.6)
    assert result['observed'] == (True,False)
    assert result['path'] is not None
    assert .40 < result['confidence'] <= .68
    c.previous_path = None
    result = c.process(one,k,np.zeros(5),3.)
    assert result['path'] is None


def test_rear_clearance_uses_observed_relative_motion_not_commanded_speed():
    e = ready_engine(rear_overhang=.21)
    e.lidar([Obstacle('car',2.5,0,.3,.3,1.,1.)], .1)
    for _ in range(10):result=e.step(.1)
    side=result['mode']
    assert side in ('LEFT_PASS','RIGHT_PASS')
    e.actual_speed=10.0
    p,l,r=corridor()
    e.camera(p,l,r,1.,(True,True),99,.6)
    e.lidar([], .6)
    assert e.step(.6)['mode'] == side


def test_persistent_camera_offset_updates_but_one_frame_spike_does_not():
    e = ready_engine()
    shifted = corridor(offset=.30)
    e.camera(*shifted, .68, (True,False), 3, .15)
    assert e.camera_time == .10
    assert np.max(np.abs(e.path[:,1])) < .01
    e.camera(*shifted, .68, (True,False), 4, .20)
    assert e.camera_time == .20
    assert np.allclose(e.path[:,1], .30*.30)
    assert e.observed == (True,False)
    e.lidar([], .20)
    assert e.step(.20)['enabled']


def test_opposite_camera_spikes_never_renew_valid_history():
    e = ready_engine()
    for i in range(3,10):
        e.camera(*corridor(offset=.4 if i%2 else -.4), 1., (True,True), i, .05*i)
    assert e.camera_time == .10
    e.lidar([], .65)
    assert e.step(.65)['speed'] == 0.


@pytest.mark.parametrize('bend', [-.03, .03])
def test_camera_curved_road_moves_pp_target_and_reduces_speed(bend):
    camera = CameraVision(VisionConfig())
    k = np.array([[320.,0,320],[0,320,240],[0,0,1]])
    image = np.full((480,640,3), (100,170,100), np.uint8)
    x = np.linspace(.3,15.,300)
    left = np.column_stack((x,bend*x*x+1.2))
    right = np.column_stack((x,bend*x*x-1.2))
    polygon = camera.project(np.vstack((left,right[::-1])),k).astype(np.int32)
    cv2.fillPoly(image,[polygon],(65,65,65))
    for wall in (left,right):
        cv2.polylines(image,[camera.project(wall,k).astype(np.int32)],False,(255,255,255),5)
    result = camera.process(image,k,np.zeros(5),0.)
    assert result['path'] is not None
    steer,target,_ = pure_pursuit(result['path'],3.,.34)
    assert steer*bend > 0
    assert target[1]*bend > 0
    engine = DecisionEngine(vehicle_width=.242,wheelbase=.34,require_official=False)
    engine.controller_ready=True
    engine.set_light('LIGHTS_OUT')
    for i in range(3):
        engine.camera(result['path'],result['left'],result['right'],result['confidence'],result['observed'],i,.05*i)
        engine.lidar([],.05*i)
    output = engine.step(.10)
    assert output['mode'] == 'CENTER'
    assert 0 < output['speed'] < 10.


def test_empty_scan_does_not_force_a_false_spatial_skew_stop():
    e = ready_engine()
    e.camera_stamp = 10.
    e.lidar([], .15, stamp=10.20)
    assert e.step(.15)['enabled']
    e.lidar([Obstacle('car',2.,0.,.3,.4,1.)], .15, stamp=10.20)
    result = e.step(.15)
    assert not result['enabled']
    assert result['reason'] == 'sensor_timestamp_skew'


def test_shaded_white_panel_is_not_a_separate_road_corridor():
    camera = CameraVision(VisionConfig())
    image = cv2.imread(str(Path(__file__).parent/'fixtures/shaded_wall.png'))
    k = np.array([[184.7521,0,320],[0,184.7521,240],[0,0,1.]])
    result = camera.process(image,k,np.zeros(5),0.)
    # Only one physical wall is visible in this actual Gazebo image. The wide
    # shaded white panel must not generate another road run inside the wall.
    assert not all(result['observed'])
    assert result['path'] is None


def test_camera_yaw_change_does_not_expire_a_valid_moving_corridor():
    e = ready_engine()
    for i in range(3,7):
        p,l,r = corridor(offset=.03*(i-2))
        for path in (p,l,r):path[:,1] += .10*(i-2)*path[:,0]
        e.camera(p,l,r,1.,(True,True),i,.05*i)
        e.lidar([],.05*i)
    assert e.camera_time == pytest.approx(.30)
    assert e.step(.30)['enabled']


def test_split_wall_end_attached_to_long_returns_is_not_a_car():
    angles = np.linspace(-math.pi/2,math.pi/2,721)
    ranges = []
    for angle in angles:
        denominator = math.sin(angle)-.2*math.cos(angle)
        distance = .6/denominator if denominator > 0 else math.inf
        x = distance*math.cos(angle)
        ranges.append(distance if 1. <= x <= 5. and not 2. <= x <= 2.20 else math.inf)
    options = dict(lidar_x=0.,lidar_y=0.,lidar_yaw=0.)
    valid, without_context = scan_candidates(ranges,-math.pi/2,math.pi/720,.04,6.,wall_attachment_gap=0.,**options)
    assert valid and without_context
    valid, candidates = scan_candidates(ranges,-math.pi/2,math.pi/720,.04,6.,wall_attachment_gap=.30,**options)
    assert valid and not candidates


def test_observed_sharp_prefix_survives_invalid_far_extrapolation():
    import json
    from dataclasses import replace
    source=Path(__file__).parent/'fixtures'
    image=cv2.imread(str(source/'sharp_visible_prefix.png'))
    k=np.asarray(json.loads((source/'sharp_visible_prefix.json').read_text())['k']).reshape(3,3)
    config=VisionConfig()
    old=CameraVision(replace(config,allow_partial_trajectory=False)).process(image,k,np.zeros(5),0.)
    assert old['path'] is None and old['observed']==(True,True)
    new=CameraVision(config).process(image,k,np.zeros(5),0.)
    assert new['reason']=='partial_camera_trajectory' and valid_path(new['path'])
    assert new['confidence']==pytest.approx(.85) and new['path'][-1,0]<5.
    assert new['path'][-1,0]<=min(max(x for x,y in new[key]) for key in ('left_samples','right_samples'))+.05
    assert pure_pursuit(new['path'],1.2,.34)[1][0]>0


def test_curved_wall_width_is_checked_on_observed_support():
    import json
    source=Path(__file__).parent/'fixtures'
    k=np.asarray(json.loads((source/'sharp_width_support.json').read_text())['k']).reshape(3,3)
    r=CameraVision(VisionConfig()).process(cv2.imread(str(source/'sharp_width_support.png')),k,np.zeros(5),0.)
    assert r['observed']==(True,True) and valid_path(r['path'])
    assert r['reason']=='partial_camera_trajectory' and r['confidence']==pytest.approx(.85)
    assert r['path'][-1,0]<3.


def test_ttc_uses_safe_selected_path_instead_of_rejected_smoothing_candidate():
    e=ready_engine();obstacle=Obstacle('car',2.5,0.,.3,.3,1.)
    e.lidar([obstacle],.1)
    for _ in range(12):result=e.step(.1)
    assert result['enabled'] and e.pass_planner.safe_path is not None
    assert not e._blocks_path(obstacle,e.pass_planner.safe_path)
    e.last_selected=corridor()[0].copy()  # A candidate that has not been commanded.
    obstacle.closing_speed=10.;e.lidar([obstacle],.15)
    result=e.step(.15)
    assert result['enabled'] and result['reason']!='critical_ttc'
    # Moving the same real footprint onto the safe commanded trajectory keeps TTC.
    obstacle.y=float(np.interp(obstacle.x,e.pass_planner.safe_path[:,0],e.pass_planner.safe_path[:,1]))
    e.lidar([obstacle],.18)
    result=e.step(.18)
    assert not result['enabled'] and result['reason']=='critical_ttc'


def test_blackout_after_armed_red_is_not_lights_out():
    d=TrafficLightDetector(VisionConfig())
    red,_=image_fixture(red=True)
    for _ in range(3):d.update(cv2.cvtColor(red,cv2.COLOR_BGR2HSV))
    for _ in range(8):
        assert d.update(np.zeros((480,640,3),np.uint8))!='LIGHTS_OUT'
    dark,_=image_fixture()
    for _ in range(3):state=d.update(cv2.cvtColor(dark,cv2.COLOR_BGR2HSV))
    assert state=='LIGHTS_OUT'


def test_different_red_rows_cannot_accumulate_arm_confirmation():
    d=TrafficLightDetector(VisionConfig())
    red,_=image_fixture(red=True)
    hsv=cv2.cvtColor(red,cv2.COLOR_BGR2HSV)
    for shift in (0,12,0,12,0,12):
        d.update(np.roll(hsv,shift,axis=1))
    assert not d.armed_boxes


def test_rendered_black_sky_and_visible_road_can_confirm_lights_out():
    dark=cv2.imread(str(Path(__file__).parent/'fixtures/monza_lights_off.png'))
    assert dark is not None
    red=dark.copy()
    for i in range(5):cv2.rectangle(red,(257+10*i,83),(263+10*i,91),(0,0,255),-1)
    d=TrafficLightDetector(VisionConfig())
    for _ in range(3):state=d.update(cv2.cvtColor(red,cv2.COLOR_BGR2HSV))
    assert state=='RED' and len(d.armed_boxes)==5
    for _ in range(3):state=d.update(cv2.cvtColor(dark,cv2.COLOR_BGR2HSV))
    assert state=='LIGHTS_OUT'
