import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.preview_speed import preview_speed


def line(length):
    x=np.linspace(.1,length,max(3,int(length*10)))
    return np.column_stack((x,np.zeros(len(x))))


def test_bands_and_high_speed_gates():
    for length,speed in ((4.9,3.5),(5,5),(8,7),(12,8),(15,9),(20,(5*19.7)**.5),(25,10)):
        assert preview_speed(line(length),1.,0.,True,True)['preview_speed_limit']==speed
    for confidence,age,stable,clear in ((.8,0,True,True),(1.,.26,True,True),
                                      (1.,0,False,True),(1.,0,True,False)):
        assert preview_speed(line(22),confidence,age,stable,clear)['preview_speed_limit']<10


def test_curve_invalid_and_stale_never_grant_ten():
    p=line(22);p[:,1]=.003*p[:,0]**2
    assert preview_speed(p,1.,0.,True,True)['preview_speed_limit']<10
    for p in (None,np.array([[1.,0.],[2.,0.],[float('nan'),0.]])):
        assert preview_speed(p,1.,0.,True,True)['supported_preview']==0


def test_far_preview_keeps_measured_length_and_requires_new_frames():
    from team_01_race.braking_preview import BrakingPreview, BrakingPreviewConfig
    b=BrakingPreview(BrakingPreviewConfig(horizon_m=25.))
    def packet(stamp,p):
        return dict(stamp=stamp,frame_id='team_01/base_link',confidence=1.,path=p.tolist())
    assert b.update(packet(1.,line(22)),1.,1.)
    assert b.speed_context(1.,True)['preview_speed_limit']==9.
    assert not b.update(packet(1.,line(22)),1.,1.)
    assert b.speed_context(1.,True)['preview_speed_limit']==9.
    assert b.update(packet(1.04,line(22)),1.04,1.04)
    assert b.speed_context(1.04,True)['preview_speed_limit']==10.
    assert b.speed_context(1.30,True) is None
    assert b.update(packet(1.32,line(6)),1.32,1.32)
    assert b.speed_context(1.32,True)['preview_speed_limit']==5.
    assert b.speed_context(1.32,True)['supported_preview']==6.


def test_future_bend_detected_before_near_path_changes():
    from team_01_race.braking_preview import BrakingPreview, BrakingPreviewConfig
    b=BrakingPreview(BrakingPreviewConfig(horizon_m=25.))
    x=np.arange(.1,21.01,.1)
    p=np.column_stack((x,.175*np.maximum(0,x-18.5)**2))
    assert np.all(p[p[:,0]<18.,1]==0.)
    for stamp in (1.,1.04):
        assert b.update(dict(stamp=stamp,frame_id='team_01/base_link',confidence=1.,path=p.tolist()),stamp,stamp)
    assert b.context(1.04)['speed_limit']==2.5
    assert b.context(1.04)['distance_to_corner']>18.


def test_observed_straight_camera_pixels_allow_ten_despite_resampling_noise():
    from test_camera_navigation import image_fixture
    from team_01_race.vision import CameraVision, VisionConfig
    image,k=image_fixture()
    camera=CameraVision(VisionConfig(horizon=25.,bev_resolution=.05,
                                    bev_half_width=4.5,road_roi_top=.38))
    measured=camera.process(image,k,None,1.)
    assert measured['path'][-1,0]>=20.
    assert preview_speed(measured['path'],measured['confidence'],0.,True,True)['preview_speed_limit']==10.


def test_measured_far_preview_removes_short_steering_visibility_cap_only():
    from test_camera_navigation import ready_engine
    e=ready_engine(cruise=10.)
    e.path=e.path[:-1];e.left=e.left[:-1];e.right=e.right[:-1]
    baseline=e.step(.1)
    assert baseline['speed']<5.
    e.supported_preview_distance=22.
    assert e.step(.1)['speed']==10.
    e.supported_preview_distance=0.
    assert e.step(.1)['speed']<5.


def test_source_acceptance_ignores_derivative_spikes_but_keeps_real_jump_guard():
    from test_camera_navigation import ready_engine,corridor
    e=ready_engine();p,l,r=corridor();p[2,1]=.03
    e.camera(p,l,r,1.,(True,True),3.,.15,stamp=3.)
    assert e.raw_jump_curvature>.75 and e.supported_jump_curvature<.75
    assert e.camera_stamp==3. and not e.last_camera_issue
    e=ready_engine();p,l,r=corridor(offset=.4)
    e.camera(p,l,r,1.,(True,True),3.,.15,stamp=3.)
    assert e.camera_stamp!=3. and e.last_camera_issue=='trajectory_jump'


def test_configured_twelve_requires_real_stopping_support_and_confirmation():
    import math
    assert preview_speed(line(32),1.,0.,True,True,horizon=32.,max_speed=12.)['preview_speed_limit']==12.
    short=preview_speed(line(22),1.,0.,True,True,max_speed=12.)
    assert short['preview_speed_limit']==math.sqrt(2*2.5*(22-.3))
    assert short['preview_speed_limit']<12.
    for stable,clear in ((False,True),(True,False)):
        assert preview_speed(line(32),1.,0.,stable,clear,horizon=32.,max_speed=12.)['preview_speed_limit']<=9.
    assert preview_speed(line(32),1.,.3,True,True,horizon=32.,max_speed=12.)['preview_speed_limit']<=3.5
