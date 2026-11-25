"""Role-specific confidence never bypasses physical curve/obstacle limits."""
import numpy as np
from team_01_race.navigation import DecisionEngine
from team_01_race.braking_preview import BrakingPreview
from team_01_race.corner import CornerConfig
from team_01_race.far_geometry import analyze_preview


def engine(curve=0.):
    e=DecisionEngine(vehicle_width=.24,wheelbase=.34,cruise=10.,require_official=False)
    e.controller_ready=True;e.set_light('LIGHTS_OUT');e.path_source='LIDAR_PATH'
    x=np.arange(.1,5.01,.1);p=np.column_stack((x,curve*x*x))
    for i in range(5):
        t=1.+i*.03;e.camera(p,p+[0,1.2],p-[0,1.2],.89,(True,True),t,t,stamp=t);e.lidar([],t,stamp=t)
    return e,t


def test_both_wall_saturation_retains_raw_confidence_and_lateral_limit():
    e,t=engine();limited=e.step(t)
    assert limited['speed']<9.
    e.full_speed_confidence=.85
    assert e.step(t)['speed']==10. and e.confidence==.89
    e,t=engine(.1);e.full_speed_confidence=.85
    out=e.step(t)
    assert out['speed']<5.
    assert out['speed']**2*out['supported_path_curvature']<=e.max_lateral_accel+1e-8


def test_far_camera_brakes_without_changing_lidar_trajectory():
    e,t=engine();before=e.path.copy()
    c=CornerConfig(straight_max_speed_mps=10.)
    preview=BrakingPreview(corner=c,normal_speed=10.)
    x=np.arange(.5,22.01,.1);p=np.column_stack((x,.12*np.maximum(0.,x-12.)**2))
    d=dict(frame_id='team_01/base_link',confidence=.95,path=p.tolist(),left=(p+[0,1.2]).tolist(),right=(p-[0,1.2]).tolist())
    d['geometry']=analyze_preview(p,p+[0,1.2],p-[0,1.2],.95,'CAMERA_PREVIEW')
    for i in range(3):
        d['stamp']=t+i*.05;assert preview.update(d,t+i*.05,t+i*.05)
    warning=preview.context(t+.1)
    assert warning is not None and warning['speed_limit']<=5.
    assert 10.<warning['distance_to_corner']<14.
    assert np.array_equal(e.path,before)


def test_extending_observed_curve_does_not_count_unobserved_tail_as_a_jump():
    e=DecisionEngine(vehicle_width=.24,wheelbase=.34,cruise=10.,require_official=False)
    x=np.arange(.1,5.01,.1);p=np.column_stack((x,.05*x*x));short=p[p[:,0]<=3.]
    for i in range(3):
        t=1.+i*.05;e.camera(short,short+[0,1.2],short-[0,1.2],.89,(True,True),t,t,stamp=t)
    # A flat np.interp continuation of the old prefix would falsely report .8m.
    assert np.max(abs(p[:,1]-np.interp(p[:,0],short[:,0],short[:,1])))>.7
    t+=.05;e.camera(p,p+[0,1.2],p-[0,1.2],.89,(True,True),t,t,stamp=t)
    assert e.last_geometry_delta is not None
    assert e.last_geometry_delta['overlap_points']>=3
    assert e.last_geometry_delta['lateral_jump']<1e-10
    assert e.last_geometry_delta['heading_jump']<1e-10


def test_confirmed_front_footprint_half_metre_stop_precedes_ttc_and_gap():
    from team_01_race.navigation import Obstacle
    e,t=engine();e.full_speed_confidence=.85
    # Front edge at .45m; both sides have passable gaps. TTC at rest is safe.
    e.lidar([Obstacle('front',.50,0.,.15,.10,1.)],t,stamp=t)
    out=e.step(t)
    assert out['speed']==0. and not out['enabled']
    assert out['reason']=='FRONT_OBSTACLE_0P5M'
    assert e.pass_planner.state=='CENTER'  # No desperate last-second pass.


def test_side_wall_point_and_unconfirmed_cluster_do_not_emergency_stop():
    from team_01_race.navigation import Obstacle
    for obstacle in (Obstacle('side',.2,.4,.05,.1,1.),Obstacle('unconfirmed',.5,0.,.15,.1,.5)):
        e,t=engine();e.full_speed_confidence=.85
        e.lidar([obstacle],t,stamp=t)
        out=e.step(t)
        assert out['enabled'] and out['speed']>0.,out


def test_no_current_or_previous_path_and_real_scan_timeout_stop():
    e,t=engine();e.path=None
    assert not e.step(t)['enabled']
    e,t=engine();e.lidar_time=t-.26
    out=e.step(t)
    assert not out['enabled'] and out['reason']=='lidar_timeout'


def test_runtime_corner_policy_caps_are_10_7_5p5_4p5():
    from pathlib import Path
    import yaml
    from team_01_race.corner import CornerDetector
    settings=yaml.safe_load((Path(__file__).parents[1]/'config/corner_control.yaml').read_text())['/**']['ros__parameters']['corner_control']
    c=CornerConfig(**settings);c.validate(10.)
    assert c.speeds(10.)==[10.,7.,5.5,4.5]
    for level,limit in enumerate(c.speeds(10.)):
        detector=CornerDetector(c,normal_speed=10.);detector.level=level
        # Geometry-independent cap: additional safety restrictions may reduce it.
        e,t=engine();out=e.step(t)
        result=detector.constrain(out,t,0.)
        assert result['corner_speed_limit']==limit
        assert result['speed']<=limit


def test_default_profile_allows_twelve_straight_and_ten_corners_with_safety_intact():
    from pathlib import Path
    import yaml
    from team_01_race.corner import CornerDetector
    folder=Path(__file__).parents[1]/'config'
    base=yaml.safe_load((folder/'corner_control.yaml').read_text())['/**']['ros__parameters']['corner_control']
    profile=yaml.safe_load((folder/'speed_profile_standard.yaml').read_text())['/**']['ros__parameters']
    c=CornerConfig(**dict(base,**profile['corner_control']));c.validate(12.)
    assert c.speeds(12.)==[12.,10.,10.,10.]
    assert profile['control']['max_speed_mps']==profile['planner']['cruise_speed_mps']==12.
    e,t=engine();e.cruise=12.;e.full_speed_confidence=.85;e.supported_preview_distance=40.
    for level,limit in enumerate(c.speeds(12.)):
        detector=CornerDetector(c,normal_speed=12.);detector.level=level
        out=detector.constrain(e.step(t),t,0.)
        assert out['speed']==limit
    from team_01_race.navigation import Obstacle
    e.lidar([Obstacle('front',.5,0.,.15,.1,1.)],t,stamp=t)
    assert e.step(t)['reason']=='FRONT_OBSTACLE_0P5M'
    assert e.step(t)['speed']==0.
    e.lidar([],t,stamp=t);e.supported_preview_distance=0.;e.path=e.path[:-1]
    assert e.step(t)['speed']<12.  # Observed braking distance still limits speed.


def test_front_stop_covers_vehicle_to_first_observed_sample():
    from team_01_race.navigation import Obstacle
    e,t=engine();e.path=e.path[e.path[:,0]>1.5]
    e.lidar([Obstacle('front',.5,0.,.15,.1,1.)],t,stamp=t)
    out=e.step(t)
    assert out['reason']=='FRONT_OBSTACLE_0P5M' and out['speed']==0.


def test_emergency_distance_is_also_enforced_during_previous_path_hold():
    from team_01_race.navigation import Obstacle
    e,t=engine()
    context=dict(paths=dict(center=e.path,selected=e.path,left=e.left,right=e.right),projection_stamp=t,
                 hold_speed=1.2,source_confidence=.89,issue='temporary_loss')
    e.lidar([Obstacle('front',.5,0.,.2,.1,1.)],t,stamp=t)
    out=e.recovery_step(t,context)
    assert not out['enabled'] and out['reason']=='FRONT_OBSTACLE_0P5M' and out['speed']==0.


def test_passable_gaps_hysteresis_and_blocked_width():
    from team_01_race.navigation import Obstacle,pure_pursuit,valid_path
    from team_01_race.obstacle_pass import ObstaclePassPlanner
    e,t=engine();p,l,r=e.path,e.left,e.right
    for y,expected in ((-.6,'LEFT_PASS'),(.6,'RIGHT_PASS'),(0.,None)):
        planner=ObstaclePassPlanner(e.width,e.wheelbase,margin=e.margin)
        o=Obstacle('car',3.8,y,.7 if y else .3,.3,1.)
        q,mode,_=planner.plan(p,l,r,[o],[o],1.,1.)
        assert valid_path(q) and mode in ('LEFT_PASS','RIGHT_PASS')
        if expected:assert mode==expected
        assert pure_pursuit(q,2.,e.wheelbase)[1][0]>0.
        for i in range(1,5):
            o.y=y+(-1)**i*.01
            assert planner.plan(p,l,r,[o],[o],i+1,1.+i*.05)[1]==mode
    planner=ObstaclePassPlanner(e.width,e.wheelbase,margin=e.margin)
    o=Obstacle('wide',3.8,0.,1.8,.4,1.)
    q,mode,reason=planner.plan(p,l,r,[o],[o],1.,1.)
    assert q is None and mode=='STOP_BLOCKED' and reason=='no_vehicle_width_gap'
    assert planner.debug['left_gap']<e.width+2*e.margin
    assert planner.debug['right_gap']<e.width+2*e.margin


def test_ten_corner_ceiling_is_reachable_on_wide_bend_but_keeps_tight_bend_limit():
    from team_01_race.corner import CornerDetector
    c=CornerConfig(straight_max_speed_mps=12.,mild_max_speed_mps=10.,
                   medium_max_speed_mps=10.,sharp_max_speed_mps=10.)
    targets=[]
    for curve in (.0125,.1):
        e,t=engine(curve);e.cruise=12.;e.full_speed_confidence=.85;e.supported_preview_distance=32.
        detector=CornerDetector(c,normal_speed=12.);detector.level=1
        out=detector.constrain(e.step(t),t,0.)
        targets.append(out['speed'])
        assert out['speed']**2*out['supported_path_curvature']<=3.+1e-8
    assert targets[0]==10.
    assert targets[1]<5.
