"""Serializable measurements for source decisions, without driving authority."""
import math
import numpy as np
from .path_manager import geometry_valid, geometry_difference


def source_snapshot(camera,lidar,keys,now,selector,previous_path_available=False):
    diff,_=geometry_difference(camera,lidar) if geometry_valid(camera) and geometry_valid(lidar) else (None,None)
    def age(key):
        stamp=keys.get(key)
        return max(0.,now-stamp) if stamp is not None and math.isfinite(stamp) else None
    def length(d):return float(d['path'][-1,0]) if geometry_valid(d) else 0.
    current='camera' if selector.source=='CAMERA_PATH' else 'lidar'
    candidate='camera' if selector.pending=='CAMERA_PATH' else 'lidar'
    return dict(switch_reason=selector.reason,camera_valid=geometry_valid(camera),
        camera_confidence=camera['confidence'] if camera else 0.,camera_age=age('camera'),
        lidar_valid=geometry_valid(lidar),lidar_confidence=lidar['confidence'] if lidar else 0.,lidar_age=age('lidar'),
        previous_path_available=previous_path_available,camera_supported_length=length(camera),lidar_supported_length=length(lidar),
        camera_lidar_center_difference=diff['lateral'] if diff else None,
        camera_lidar_lateral_difference=diff['lateral'] if diff else None,
        camera_lidar_heading_difference=diff['heading_deg'] if diff else None,
        camera_lidar_curvature_difference=diff.get('curvature') if diff else None,
        current_source_age=age(current),candidate_source_age=age(candidate) if selector.pending else None,
        candidate_source_confirm_frames=selector.count if selector.pending else getattr(selector,'last_confirmation',0),trajectory_difference=None,
        state=getattr(selector,'state',selector.source),geometry_equivalent=getattr(selector,'geometry_equivalent',False))
