import math
import numpy as np
from team_01_race.depth_geometry import *
from team_01_race.depth_corner import *

SPEEDS=[10.,7.,5.5,4.5]
def packet(t,level=0,distance=12.,direction=1):
    return dict(stamp=t,depth_valid=True,confidence=.9,depth_corner_candidate=level>0,
        level=level,direction=direction,distance=distance if level else None,
        straight_geometry=level==0,depth_supported_distance_m=16.,depth_curvature_supported_distance_m=15.,segments=[])
def update(track,t,level=0,travel=0.,distance=12.,lidar=True):
    return track.update(packet(t,level,distance),t,t,travel,True,lidar)
def policy(track,t,travel=0.):return track.policy(t,travel,True,10.,SPEEDS,4.,.75,.1)

def test_actual_pinhole_projection_and_nonfinite_filter():
    k=[100.,0,1,0,100.,1,0,0,1]
    d=np.array([[np.nan,0,np.inf],[.01,5,50],[4,6,7.]])
    q=project_depth(d,k,[.23,0,.08,0,0,0],.03,50)
    assert len(q)==4
    assert np.allclose(q[0],[5.23,0,.08])

def test_straight_and_far_isolated_noise():
    a=np.column_stack((np.arange(1.,17.),np.zeros(16)))
    for noise in (False,True):
        q=a.copy()
        if noise:q[-2,1]=.5
        r=analyze_center(q,[20]*len(q))
        assert r['depth_valid'] and r['straight_geometry'] and not r['depth_corner_candidate']

def test_continuous_curves_and_supported_onset():
    for radius,expected in [(8.,1),(4.,2),(2.7,3)]:
        # Straight to x12, then a real circular arc, sampled in forward metres.
        x=np.arange(1.,12+radius,.5);dx=np.maximum(0,x-12)
        y=np.where(x>12,radius-np.sqrt(np.maximum(0,radius*radius-dx*dx)),0.)
        c=DepthConfig(station_m=.5,min_curve_length_m=1.)
        r=analyze_center(np.column_stack((x,y)),[30]*len(x),c)
        assert r['depth_valid']
        assert r['level']==expected,r
        assert abs(r['distance']-12)<=1.5,r

def test_confirmation_and_single_frame_false_sharp():
    t=DepthCornerTrack()
    for x in (.1,.2,.3,.4):update(t,x)
    assert policy(t,.4)['depth_corner_speed_limit']==10
    update(t,.5,3);assert not t.latched
    update(t,.6);assert not t.latched
    for x in (.7,.8,.9):update(t,x,3)
    assert t.latched and policy(t,.9)['depth_corner_speed_limit']==4.5

def test_severity_speed_mapping():
    for level,speed in enumerate(SPEEDS):
        t=DepthCornerTrack()
        for x in (.1,.2,.3,.4):update(t,x,level)
        assert policy(t,.4)['depth_corner_speed_limit']==speed

def test_odom_arc_and_latch_loss_reacquisition_exit():
    odom=CornerTravel()
    assert odom.update(.1,.1,0,(0,0,0),10)
    # Non-axis-aligned motion accumulates total path length, not delta x.
    for i in range(1,31):assert odom.update(.1+i*.1,.1+i*.1,0,(i*.06,i*.08,0),1)
    assert abs(odom.distance-3)<1e-9
    t=DepthCornerTrack()
    for x in (.1,.2,.3):update(t,x,3)
    assert abs(policy(t,.31,3)['remaining_corner_distance']-9)<1e-9
    assert abs(policy(t,.32,7)['remaining_corner_distance']-5)<1e-9
    lost=policy(t,.8,7);assert lost['corner_latched'] and lost['depth_target_speed']<=4.5
    update(t,.9,3,7,5.1);update(t,1.,3,7,5.1);update(t,1.1,3,7,5.1)
    assert abs(t.remaining(7)-5.1)<.1
    before=t.remaining(7)
    update(t,1.2,3,7,17.);assert t.remaining(7)==before
    for x in (1.3,1.4,1.5,1.6):update(t,x,0,12.,lidar=False)
    assert t.latched
    update(t,1.7,0,12.,lidar=True);assert t.latched is None and t.state=='CORNER_EXIT'
    update(t,1.8,0,12.);assert policy(t,1.8,12.)['depth_corner_speed_limit']==10

def test_stale_depth_never_keeps_ten_and_duplicates_not_confirmations():
    t=DepthCornerTrack()
    for _ in range(5):update(t,.1)
    assert not t.straight
    for x in (.2,.3,.4):update(t,x)
    assert policy(t,.4)['depth_target_speed']==10
    assert policy(t,1.)['depth_target_speed']<=3.5
    t.update(dict(packet(1.1),depth_valid=False),1.1,1.1,0.,True,True)
    assert policy(t,1.1)['depth_target_speed']<=3.5

def test_short_depth_support_and_single_wall_not_high_speed():
    c=DepthConfig();points=[]
    for x in np.arange(1,5,.1):
        for z in (.0,.05,.1):points.append([x,1,z])
    a,n=center_geometry(points,-.085,c)
    assert not len(a)
    r=analyze_center([[1,0],[2,0],[3,0]],[20]*3)
    assert not r['depth_valid']

def test_odom_teleport_invalidates_travel_and_latch_cannot_allow_high_speed():
    o=CornerTravel();o.update(.1,.1,0,(0,0,0),1)
    assert not o.update(.2,.2,0,(100,0,0),1)
    assert o.distance==0 and not o.fresh(.2)

def render_wall_depth(center):
    """Analytic pinhole rays intersect real 3-D vertical wall segments."""
    h,w=480,640;f=w/(2*math.tan(math.pi/3));k=[f,0,w/2,0,f,h/2,0,0,1]
    v,u=np.indices((h,w));ray=np.stack(((u-w/2)/f,(v-h/2)/f,np.ones((h,w))),axis=-1)
    pose=[.23,0,.08,0,math.radians(12),0]
    ray=ray@(rotation(*pose[3:])@rotation(-math.pi/2,0,-math.pi/2)).T
    origin=np.array(pose[:3]);depth=np.full((h,w),np.inf)
    def cross(a,b):return a[...,0]*b[...,1]-a[...,1]*b[...,0]
    for side in (-1,1):
        wall=np.array(center)+[0,1.2*side]
        for a,b in zip(wall[:-1],wall[1:]):
            segment=b-a;den=cross(ray[...,:2],segment)
            with np.errstate(divide='ignore',invalid='ignore'):
                t=cross(a-origin[:2],segment)/den
                along=cross(a-origin[:2],ray[...,:2])/den
                z=origin[2]+ray[...,2]*t
            valid=(t>.03)&(t<50)&(along>=0)&(along<=1)&(z>-.085)&(z<.215)
            depth[valid]=np.minimum(depth[valid],t[valid])
    return depth,k,pose

def test_depth_image_to_metric_straight_not_rgb_pixel_curve():
    x=np.arange(.1,22,.25);d,k,pose=render_wall_depth(np.column_stack((x,np.zeros(len(x)))))
    q=project_depth(d,k,pose,.03,50)
    a,n=center_geometry(q,-.085,DepthConfig(min_boundary_points=2))
    r=analyze_center(a,n)
    assert r['depth_valid'] and r['straight_geometry'] and not r['depth_corner_candidate'],r
    assert r['depth_supported_distance_m']<=18

def test_odom_expired_corner_cannot_hold_ten_indefinitely():
    t=DepthCornerTrack()
    for x in (.1,.2,.3):update(t,x,3)
    assert policy(t,1.)['depth_corner_confirmed']
    p=t.policy(1.,0.,False,10.,SPEEDS,4.,.75,.1)
    assert p['depth_target_speed']<=3.5
    assert not policy(t,3.)['depth_corner_confirmed']

def test_future_and_reordered_depth_cannot_renew_freshness():
    t=DepthCornerTrack()
    for x in (.1,.2,.3,.4):update(t,x)
    assert not t.update(packet(.3),.5,.5,0.,True,True)
    assert not t.update(packet(2.),.5,.5,0.,True,True)
    assert not t.fresh(1.)

def test_real_projection_corner_onset_and_wall_support():
    x=np.arange(.1,14.6,.1);dx=np.maximum(0.,x-12.)
    y=np.where(x>12.,2.7-np.sqrt(np.maximum(0.,2.7**2-dx**2)),0.)
    d,k,p=render_wall_depth(np.column_stack((x,y)))
    r=wall_preview(project_depth(d,k,p,.03,50),-.085)
    assert r and r['depth_corner_candidate'] and r['level']>=2
    assert abs(r['distance']-12.)<=1.,r
    assert r['confidence']<.9
    assert r['depth_curvature_supported_distance_m']<=r['depth_supported_distance_m']

def test_invalid_or_stale_straight_requires_fresh_reconfirmation():
    for invalid in (False,True):
        t=DepthCornerTrack()
        for x in (.1,.2,.3,.4):update(t,x)
        assert policy(t,.4)['depth_corner_speed_limit']==10.
        if invalid:
            t.update(dict(packet(.5),depth_valid=False),.5,.5,0.,True,True)
            t.update(dict(packet(.8),depth_valid=False),.8,.8,0.,True,True)
        start=.9 if invalid else 1.
        for i in range(3):
            x=start+i*.1;update(t,x)
            assert policy(t,x)['depth_target_speed']<=3.5
        update(t,start+.3)
        assert policy(t,start+.3)['depth_corner_speed_limit']==10.

def test_continuous_source_frames_do_not_double_count_processing_age():
    t=DepthCornerTrack()
    for stamp in (.1,.2,.3,.4):
        # Every input is fresh (.30<.35), with normal .10s capture cadence.
        t.update(packet(stamp),stamp+.3,stamp+.3,0.,True,True)
    assert policy(t,.7)['depth_corner_speed_limit']==10.
    # Once the last actual observation ages out, authority is still revoked.
    assert policy(t,1.)['depth_target_speed']<=3.5


def test_extended_depth_only_counts_measured_stations_and_detects_far_corner():
    c=DepthConfig(horizon_m=32.);c.validate()
    x=np.arange(1.,32.01,.5)
    straight=analyze_center(np.column_stack((x,np.zeros_like(x))),[20]*len(x),c)
    assert straight['depth_valid'] and straight['straight_geometry']
    assert straight['depth_supported_distance_m']==32.
    assert straight['depth_curvature_supported_distance_m']==31.5
    assert max(s['end_m'] for s in straight['segments'])>30.
    # Enlarging configured range cannot extend an 8m measurement.
    short=x[x<=8.]
    limited=analyze_center(np.column_stack((short,np.zeros_like(short))),[20]*len(short),c)
    assert limited['depth_supported_distance_m']==8.
    dx=np.maximum(0.,x-24.)
    curve=analyze_center(np.column_stack((x,.06*dx*dx)),[20]*len(x),c)
    assert curve['depth_corner_candidate'] and curve['distance']>18.,curve

def test_flat_imu_removes_five_cap_without_bypassing_real_ramp_or_staleness():
    from team_01_race.ramp import RampDetector
    d=RampDetector(normal_speed=12.)
    base=dict(speed=12.,enabled=True,mode='CENTER')
    assert d.constrain(base,0.)['speed']==5.
    for i in range(4):d.update((0.,0.,0.,1.),i*.01,i*.01)
    assert d.constrain(base,.03)['speed']==12.
    pitch=math.radians(12)/2
    for i in range(4,25):d.update((0.,math.sin(pitch),0.,math.cos(pitch)),i*.01,i*.01)
    assert d.active and d.constrain(base,.24)['speed']==5.
    assert d.constrain(base,1.)['speed']==5.


def test_early_depth_wall_band_preserves_exact_measured_points_for_arbitrary_mount():
    rng=np.random.default_rng(17)
    d=rng.uniform(.01,50.5,(91,143));d[0,:3]=[0.,np.nan,np.inf]
    k=[120.,0.,71.,0.,115.,45.,0.,0.,1.]
    for pose in ([.23,0.,.08,0.,.21,0.],[.2,.01,.08,.08,-.1,.05]):
        all_points=project_depth(d,k,pose,.03,50.)
        keep=(all_points[:,2]>-.045)&(all_points[:,2]<.235)&(all_points[:,0]>=.5)&(all_points[:,0]<=32.)
        band=project_depth(d,k,pose,.03,50.,z_bounds=(-.045,.235),x_bounds=(.5,32.))
        assert np.array_equal(band,all_points[keep])


def test_station_indexing_preserves_support_counts_and_geometry_under_cloud_shuffle():
    rng=np.random.default_rng(42);points=[]
    for x in np.arange(.5,32.01,.25):
        for side in (-1.,1.):
            for z in (-.02,.04,.12):points.append([x,side*1.2+.002*x*x,z])
    points=np.asarray(points)
    a,n=center_geometry(points,-.085)
    b,m=center_geometry(points[rng.permutation(len(points))],-.085)
    assert np.array_equal(a,b) and n==m
    assert a[-1,0]>25.
