import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.far_geometry import analyze_preview
from team_01_race.preview_fusion import fuse_packets


def packet(length,offset=0.):
    x=np.arange(.2,length,.1);p=np.column_stack((x,np.full(len(x),offset)));l=p+[0,1.2];r=p-[0,1.2]
    return dict(stamp=1.,frame_id='team_01/base_link',confidence=.9,path=p.tolist(),left=l.tolist(),right=r.tolist(),geometry=analyze_preview(p,l,r,.9))


def test_fusion_intersects_measured_support_and_rejects_disagreement():
    a=packet(24.9);b=packet(6.)
    fused=fuse_packets(a,b)
    assert fused and fused['geometry']['curvature_supported_preview_m']<6.
    assert fuse_packets(a,packet(24.9,.5)) is None
    b['stamp']=1.2
    assert fuse_packets(a,b) is None


def test_short_lidar_does_not_shrink_positional_preview_or_fake_curvature():
    from team_01_race.braking_preview import BrakingPreview
    from team_01_race.preview_fusion import choose_visibility_preview
    camera=BrakingPreview();lidar=BrakingPreview()
    a=packet(24.9);b=packet(6.)
    a['geometry']['curvature_supported_preview_m']=8.
    for stamp in (1.,1.04):
        a['stamp']=b['stamp']=stamp
        assert camera.update(a,stamp,stamp) and lidar.update(b,stamp,stamp)
    assert choose_visibility_preview(camera,lidar,1.04) is camera
    assert camera.speed_context(1.04,True)['preview_speed_limit']==9.


def test_gap_in_far_curvature_support_does_not_grant_ten():
    from team_01_race.braking_preview import BrakingPreview
    preview=BrakingPreview();data=packet(24.9)
    data['geometry']['curvature_support_intervals_m']=[[.2,4.8],[5.2,24.]]
    for stamp in (1.,1.04):
        data['stamp']=stamp;assert preview.update(data,stamp,stamp)
    assert preview.speed_context(1.04,True)['preview_speed_limit']==9.
