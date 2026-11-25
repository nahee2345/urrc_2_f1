import sys,math
from pathlib import Path
import cv2
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.far_geometry import analyze_preview
from team_01_race.vision import CameraVision,VisionConfig


def pixel_bend(start,scale=1):
    config=VisionConfig(horizon=25.,bev_half_width=4.5,bev_resolution=.05,
                        road_roi_top=.38,far_local_boundary_fit=True,
                        camera_x=.23,camera_height=.165,camera_pitch=math.radians(12.))
    camera=CameraVision(config)
    k=np.array([[640/(2*math.tan(math.pi/3)),0,320],[0,640/(2*math.tan(math.pi/3)),240],[0,0,1.]])
    k[:2]*=scale
    x=np.arange(.4,25.01,.025);y=.09*np.maximum(x-start,0)**2
    image=np.zeros((480*scale,640*scale,3),np.uint8);image[:]=(100,170,100)
    left=np.column_stack((x,y+1.2));right=np.column_stack((x,y-1.2))
    polygon=np.concatenate((camera.project(left,k),camera.project(right[::-1],k))).astype(np.int32)
    cv2.fillPoly(image,[np.clip(polygon,-10000,10000)],(65,65,65))
    for wall in (left,right):cv2.polylines(image,[np.clip(camera.project(wall,k),-10000,10000).astype(np.int32)],False,(255,255,255),scale,cv2.LINE_AA)
    result=camera.process(image,k,None,1.)
    pixels=tuple(camera.project(samples,k) for samples in (result['left_samples'],result['right_samples']))
    geometry=analyze_preview(result['path'],result['left_samples'],result['right_samples'],result['confidence'],source_pixels=pixels)
    return result,geometry,image


@pytest.mark.parametrize('start',[5.,8.,10.,12.,15.,18.])
def test_projected_pixels_preserve_future_corner_start(start):
    result,g,image=pixel_bend(start)
    distance=g['distance_to_first_significant_curvature']
    if distance is None:pytest.xfail(f'Camera pixels do not support the requested {start}m bend; curvature support ends at {g["curvature_supported_preview_m"]:.2f}m')
    assert abs(distance-start)<2.,(start,distance)
    assert g['curvature_supported_preview_m']>=distance+.8


def test_no_wall_support_cannot_grant_curvature_support():
    x=np.arange(.1,25.01,.1);p=np.column_stack((x,np.zeros(len(x))))
    assert analyze_preview(p)['curvature_supported_preview_m']==0.
    walls=np.column_stack((x[x<7.],np.ones(sum(x<7.))))
    g=analyze_preview(p,walls,walls-[0,2],1.)
    assert g['curvature_supported_preview_m']<7.
    assert g['distance_to_first_significant_curvature'] is None


@pytest.mark.parametrize('start',[5.,8.,10.,12.,15.,18.])
def test_measured_wall_fixture_preserves_bend_distance(start):
    from team_01_race.braking_preview import BrakingPreview
    x=np.arange(.2,24.9,.1);y=.09*np.maximum(0,x-start)**2
    p=np.column_stack((x,y));left=p+[0,1.2];right=p-[0,1.2]
    g=analyze_preview(p,left,right,1.)
    assert abs(g['distance_to_first_significant_curvature']-start)<1.
    # Exercise the same packet-to-warning route used by Camera/LiDAR callbacks.
    # The valid body-frame forward prefix ends before a fold/steep bend.
    end=np.flatnonzero(abs(np.diff(p[:,1]))>=.35)
    p=p[:end[0]+1] if len(end) else p
    b=BrakingPreview()
    for stamp in (1.,1.04):
        assert b.update(dict(stamp=stamp,frame_id='team_01/base_link',confidence=1.,path=p.tolist(),geometry=g),stamp,stamp)
    assert b.context(1.04) is not None
    assert abs(b.context(1.04)['distance_to_corner']-start)<1.


def test_single_spike_and_straight_pixels_do_not_create_a_corner():
    r,g,_=pixel_bend(100.)
    assert g['distance_to_first_significant_curvature'] is None
    x=np.arange(.2,24.9,.1);p=np.column_stack((x,np.zeros(len(x))))
    p[120,1]=.2
    g=analyze_preview(p,p+[0,1.2],p-[0,1.2],1.)
    assert g['distance_to_first_significant_curvature'] is None


def test_replicated_source_pixels_do_not_create_far_curvature_support():
    x=np.arange(.2,24.9,.1);p=np.column_stack((x,.09*np.maximum(0,x-12.)**2));left=p+[0,1.2];right=p-[0,1.2]
    # All virtual BEV samples were copied from just one camera pixel.
    uv=np.zeros((len(x),2))
    g=analyze_preview(p,left,right,1.,source_pixels=(uv,uv))
    assert g['curvature_supported_preview_m']==0.
    assert g['distance_to_first_significant_curvature'] is None


def test_independent_pixel_range_query_matches_brute_force():
    from team_01_race.far_geometry import PixelSupport
    rng=np.random.default_rng(52);x=np.arange(.1,25.,.1);wall=np.column_stack((x,np.ones(len(x))))
    uv=rng.integers(0,12,size=(len(x),2));support=PixelSupport(wall,uv)
    lower=rng.uniform(0,24,100);upper=lower+rng.uniform(0,6,100)
    expected=[len(np.unique(uv[(x>=a)&(x<=b)],axis=0)) for a,b in zip(lower,upper)]
    assert np.array_equal(support.counts(lower,upper),expected)
