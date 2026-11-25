import math
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.lidar_filter import CameraWallFilter, WallFilterConfig, scan_points
from team_01_race.navigation import ObstacleTracker, DecisionEngine, Obstacle, scan_candidates


def corridor(kind='straight',half=1.2):
    x=np.arange(.1,5.01,.05)
    y={'straight':lambda x:0*x,'left':lambda x:.22*x*x,
       'right':lambda x:-.22*x*x,'s':lambda x:.6*np.sin(1.4*x)}[kind](x)
    path=np.column_stack((x,y));left=path.copy();right=path.copy()
    left[:,1]+=half;right[:,1]-=half
    return path,left,right


def test_same_scan_lidar_corridor_keeps_real_obstacles_when_camera_is_delayed():
    p,left,right=corridor()
    f=CameraWallFilter(.2416506478)
    assert f.update_camera(p,left,right,1.,(True,True),1.,0.)
    assert f.update_lidar_corridor(p,left,right,.85,(True,True),1.2,.2)
    ranges,angle,increment=ray_scan(left,right,[(2.,0.,.3,.3)])
    valid,points,indices=scan_points(ranges,angle,increment,.04,6.,np.eye(3),[.195,0,0])
    assert valid
    ready,candidates,debug=f.classify(points,indices,1.2,.21)
    assert ready and candidates and debug['state']=='LIDAR_CORRIDOR'
    assert not f.classify(points,indices,1.25,.22)[0]  # Other scans cannot reuse it.
    assert not f.classify(points,indices,1.2,.46)[0]  # Original age still expires.


def ray_scan(left,right,objects=(),beams=1441):
    """Ray/segment intersections, including wall occlusion and object faces."""
    origin=np.array([.195,0.]);angles=np.linspace(-math.pi/2,math.pi/2,beams)
    rays=np.column_stack((np.cos(angles),np.sin(angles)))
    starts=[left[:-1],right[:-1]];ends=[left[1:],right[1:]]
    for x,y,depth,width in objects:
        box=np.array([[x-depth/2,y-width/2],[x+depth/2,y-width/2],
                      [x+depth/2,y+width/2],[x-depth/2,y+width/2],[x-depth/2,y-width/2]])
        starts.append(box[:-1]);ends.append(box[1:])
    a=np.vstack(starts)-origin;v=np.vstack(ends)-np.vstack(starts)
    cross=lambda p,q:p[...,0]*q[...,1]-p[...,1]*q[...,0]
    den=cross(rays[:,None,:],v[None,:,:])
    with np.errstate(divide='ignore',invalid='ignore'):
        distance=cross(a,v)[None,:]/den
        u=cross(a[None,:,:],rays[:,None,:])/den
    distance[(u<0)|(u>1)|(distance<.04)|(distance>6)|(~np.isfinite(distance))]=np.inf
    return np.min(distance,axis=1).tolist(),-math.pi/2,math.pi/(beams-1)


def classify(kind='straight',objects=(),confidence=1.,observed=(True,True),selected=None):
    path,left,right=corridor(kind)
    f=CameraWallFilter(.2416506478)
    assert f.update_camera(path,left,right,confidence,observed,1.,0.)
    if selected is not None:f.update_selected(selected,1.,0.)
    ranges,angle,increment=ray_scan(left,right,objects)
    valid,points,indices=scan_points(ranges,angle,increment,.04,6.,np.eye(3),[.195,0,0])
    assert valid
    ready,candidates,debug=f.classify(points,indices,1.05,.05)
    return f,candidates,debug,points


@pytest.mark.parametrize('kind',['straight','left','right','s'])
def test_camera_walls_are_removed_before_obstacle_clustering(kind):
    f,candidates,debug,points=classify(kind)
    assert len(points)>20
    assert debug['wall_rejected_points']>20
    assert not candidates,debug
    tracker=ObstacleTracker()
    for t in [0.,.05,.1]:assert not tracker.update(candidates,t)


@pytest.mark.parametrize('kind',['left','right','s'])
def test_real_car_in_corner_survives_wall_rejection(kind):
    path,_,_=corridor(kind);y=float(np.interp(1.7,path[:,0],path[:,1]))
    _,candidates,debug,_=classify(kind,[(1.7,y,.30,.30)])
    assert len(candidates)==1,debug
    tracker=ObstacleTracker()
    assert not tracker.update(candidates,0.)
    assert not tracker.update(candidates,.05)
    assert len(tracker.update(candidates,.1))==1


def test_near_wall_car_is_not_deleted_as_wall_and_selected_path_is_used():
    selected=corridor()[0].copy();selected[:,1]=.75
    _,candidates,debug,_=classify(objects=[(2.,1.055,.30,.25)],selected=selected)
    assert len(candidates)==1,debug
    item=next(d for d in debug['details'] if d['classification']=='PATH_BLOCKING_OBSTACLE')
    assert not item['wall_connected']


def test_nonblocking_internal_car_is_debugged_but_not_sent_to_decision():
    _,candidates,debug,_=classify(objects=[(2.,.85,.3,.3)])
    assert not candidates
    assert any(d['classification']=='NON_BLOCKING_OBJECT' for d in debug['details'])


def test_one_wall_camera_estimate_uses_existing_inferred_other_wall():
    _,candidates,debug,_=classify('left',confidence=.68,observed=(True,False))
    assert not candidates and debug['wall_rejected_points']>20


def test_low_confidence_does_not_delete_objects_using_untrusted_walls():
    f=CameraWallFilter(.242);path,left,right=corridor()
    assert not f.update_camera(path,left,right,.2,(True,True),1.,0.)
    ready,objects,debug=f.classify(np.array([[1.,0.]]),np.array([0]),1.,.05)
    assert not ready and not objects and debug['state']=='NO_FRESH_CAMERA_CORRIDOR'


def test_invalid_boundary_short_hold_then_watchdog_not_raw_range_fallback():
    f=CameraWallFilter(.242);path,left,right=corridor()
    assert f.update_camera(path,left,right,1.,(True,True),1.,0.)
    assert not f.update_camera(None,None,None,0.,(False,False),1.05,.05)
    assert f.context(1.10,.10) is not None
    assert f.latest_invalid
    assert f.context(1.30,.30) is None


def test_camera_scan_timestamp_skew_is_not_ignored():
    f=CameraWallFilter(.242);path,left,right=corridor()
    f.update_camera(path,left,right,1.,(True,True),1.,0.)
    assert f.context(1.16,.05) is None
    assert f.context(.84,.05) is None
    assert f.context(1.10,.10) is not None


def test_camera_replay_does_not_refresh_geometry():
    f=CameraWallFilter(.242);path,left,right=corridor()
    f.update_camera(path,left,right,1.,(True,True),1.,0.)
    assert not f.update_camera(path,left,right,1.,(True,True),1.,.2)
    assert f.context(1.,.3) is None


def test_vehicle_frame_rotation_and_translation_precede_classification():
    rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    valid,points,_=scan_points([1.]*21,0.,.001,.04,6.,rotation,[.195,.1,0])
    assert valid
    assert np.allclose(points[0],[.195,1.1])


def test_invalid_scan_is_not_fresh_clear_space():
    valid,points,_=scan_points([math.nan]*181,-math.pi/2,math.pi/180,.04,6.,np.eye(3),[.195,0,0])
    assert not valid and not len(points)


def test_confirmation_is_consecutive_and_release_has_three_clear_frames():
    t=ObstacleTracker(release_frames=3);c=[(2.,0.,.3,.3)]
    assert not t.update(c,0.)
    assert not t.update([], .03)
    assert not t.update(c,.06)
    assert not t.update(c,.09)
    assert len(t.update(c,.12))==1
    assert len(t.update([],.15))==1
    assert len(t.update([],.18))==1
    assert not t.update([],.21)


def engine(kind):
    e=DecisionEngine(vehicle_width=.242,wheelbase=.34,cruise=7.,require_official=False)
    e.controller_ready=True;e.set_light('LIGHTS_OUT')
    p,l,r=corridor(kind)
    for i in range(3):e.camera(p,l,r,1.,(True,True),i,.05*i);e.lidar([],.05*i)
    return e


@pytest.mark.parametrize('kind',['left','right','s'])
def test_corner_walls_keep_center_without_stop(kind):
    e=engine(kind);_,candidates,_,_=classify(kind)
    assert not candidates
    result=e.step(.1)
    assert result['mode']=='CENTER' and result['enabled']


def test_fixed_front_rectangle_does_not_trigger_ttc_off_curve_path():
    e=engine('left');e.actual_speed=7.
    e.lidar([Obstacle('off_path',2.5,0.,.2,.2,1.,7.)],.1)
    result=e.step(.1)
    assert result['mode']=='CENTER' and math.isinf(result['ttc']),result


def test_validated_close_obstacle_on_curved_path_keeps_critical_ttc():
    e=engine('left');e.actual_speed=7.
    y=float(np.interp(1.5,e.path[:,0],e.path[:,1]))
    e.lidar([Obstacle('car',1.5,y,.3,.3,1.,7.)],.1)
    result=e.step(.1)
    assert result['mode']=='EMERGENCY_STOP' and result['reason']=='critical_ttc'


def test_stationary_real_car_allows_existing_pass_logic():
    e=engine('straight');e.lidar([Obstacle('car',2.5,0.,.3,.3,1.)],.1)
    for _ in range(10):result=e.step(.1)
    assert result['mode'] in ('LEFT_PASS','RIGHT_PASS'),result


@pytest.mark.parametrize('kind',['left','right'])
def test_isolated_curved_wall_fragment_reproduces_old_false_candidate(kind):
    path,left,right=corridor(kind)
    ranges,angle,increment=ray_scan(left,right)
    angles=angle+np.arange(len(ranges))*increment
    # A short visible wall portion, separated by missing returns from longer
    # wall segments, has a compact bbox despite being camera-labelled wall.
    ranges=np.where(np.abs(angles)<.09,ranges,np.inf).tolist()
    valid,old=scan_candidates(ranges,angle,increment,.04,6.,lidar_x=.195,lidar_y=0,lidar_yaw=0)
    assert valid and len(old)==1,old
    valid,points,indices=scan_points(ranges,angle,increment,.04,6.,np.eye(3),[.195,0,0])
    f=CameraWallFilter(.242);f.update_camera(path,left,right,1.,(True,True),1.,0.)
    ready,new,debug=f.classify(points,indices,1.05,.05)
    assert ready and not new and debug['wall_rejected_points']==len(points),debug


def test_wide_transverse_blocker_is_not_discarded_as_non_vehicle():
 _,candidates,debug,_=classify(objects=[(3.,0.,.4,1.8)])
 assert len(candidates)==1,debug
 assert candidates[0][2]>1.7


def test_half_metre_emergency_uses_wall_filtered_confirmed_cluster():
    for half,objects,expected in ((1.2,[(.5,0.,.1,.2)],True),(.4,[],False)):
        path,left,right=corridor(half=half)
        ranges,angle,increment=ray_scan(left,right,objects)
        valid,points,indices=scan_points(ranges,angle,increment,.04,6.,np.eye(3),[.195,0,0])
        assert valid
        f=CameraWallFilter(.2416506478)
        tracker=ObstacleTracker()
        e=DecisionEngine(vehicle_width=.2416506478,wheelbase=.34,require_official=False)
        e.controller_ready=True;e.set_light('LIGHTS_OUT')
        for i in range(4):
            t=1.+i*.05
            assert f.update_lidar_corridor(path,left,right,.89,(True,True),t,t)
            ready,candidates,debug=f.classify(points,indices,t,t)
            assert ready,debug
            obstacles=tracker.update(candidates,t)
            e.camera(path,left,right,.89,(True,True),t,t,stamp=t);e.lidar(obstacles,t,stamp=t)
        out=e.step(t)
        if expected:
            assert out['reason']=='FRONT_OBSTACLE_0P5M' and out['speed']==0.,(out,debug)
        else:
            assert not obstacles and out['enabled'] and out['speed']>0.,(out,debug)
