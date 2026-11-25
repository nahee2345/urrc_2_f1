"""Fuse only fresh agreeing observed wall geometry; never obstacle clusters."""
import numpy as np
from .path_manager import geometry_difference
from .far_geometry import analyze_preview


def fuse_packets(camera,lidar):
    if abs(camera['stamp']-lidar['stamp'])>.15:return None
    geometries=[]
    for packet in (camera,lidar):
        d={k:np.asarray(packet.get(k,[]),dtype=float) for k in ('path','left','right')}
        if any(p.ndim!=2 or p.shape[1]!=2 or len(p)<6 for p in d.values()):return None
        geometries.append(d)
    diff,aligned=geometry_difference(*geometries)
    if diff is None or diff['lateral']>.25 or diff['heading_deg']>15. or diff['width']>.3 or diff['curvature']>.15:return None
    c,l,cl,cr,ll,lr=aligned
    weight=camera['confidence']/(camera['confidence']+lidar['confidence'])
    path=weight*c+(1-weight)*l;left=weight*cl+(1-weight)*ll;right=weight*cr+(1-weight)*lr
    confidence=min(camera['confidence'],lidar['confidence'])
    # Intersect curvature authority as well as positional support.
    end=min(camera['geometry']['curvature_supported_preview_m'],lidar['geometry']['curvature_supported_preview_m'])
    path=path[path[:,0]<=end];left=left[left[:,0]<=end];right=right[right[:,0]<=end]
    if len(path)<6:return None
    geometry=analyze_preview(path,left,right,confidence,'FUSED_PREVIEW')
    geometry.update(camera_far_valid=True,lidar_far_valid=True)
    return dict(stamp=min(camera['stamp'],lidar['stamp']),frame_id='team_01/base_link',confidence=confidence,
        path=path.tolist(),left=left.tolist(),right=right.tolist(),geometry=geometry)


def choose_preview(camera,lidar,fused,now,ros_now):
    available=[p for p in (camera,lidar) if p.speed_context(now) is not None]
    if len(available)==2 and camera.packet is not None and lidar.packet is not None:
        packet=fuse_packets(camera.packet,lidar.packet)
        if packet is not None:
            fused.update(packet,ros_now,now)
            if fused.speed_context(now) is not None:
                # Corroborating a short overlap must not replace a longer
                # independently measured Camera preview with LiDAR's range.
                best=max(available,key=lambda p:p.speed_data['confidence']*float((p.geometry or {}).get('curvature_supported_preview_m',0.)))
                fused_end=float((fused.geometry or {}).get('curvature_supported_preview_m',0.))
                best_end=float((best.geometry or {}).get('curvature_supported_preview_m',0.))
                if fused_end>=best_end-.5:return fused
    if not available:return camera
    return max(available,key=lambda p:p.speed_data['confidence']*float((p.geometry or {}).get('curvature_supported_preview_m',0.)))


def choose_visibility_preview(camera,lidar,now):
    candidates=[p for p in (camera,lidar) if p.speed_context(now) is not None]
    return max(candidates,key=lambda p:float(p.speed_path[-1,0])) if candidates else camera
