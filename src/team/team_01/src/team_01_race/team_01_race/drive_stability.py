"""Same-cycle diagnostics; never changes a path, speed or steering command."""
import math
import numpy as np
from .navigation import valid_path,path_geometry,supported_curvature

def finite_payload(value):
    if isinstance(value,dict):return {k:finite_payload(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [finite_payload(v) for v in value]
    if isinstance(value,(float,np.floating)) and not math.isfinite(value):return None
    return value

def reduction_reason(limiter,stop_reason='',source=''):
    s=(limiter+' '+stop_reason).upper()
    if any(k in s for k in ('TIMEOUT','CLOCK','STALE')):return 'SENSOR_TIMEOUT'
    if 'TTC' in s:return 'TTC'
    if any(k in s for k in ('OBSTACLE','PASS','FRONT_OBSTACLE')):return 'OBSTACLE'
    if any(k in s for k in ('NO_VALID','PATH_LOST','INVALID_SELECTED','NO_PATH')):return 'NO_VALID_PATH'
    if 'PREVIOUS' in source:return 'PREVIOUS_PATH'
    if any(k in s for k in ('GUARD','CORNER_SEVERITY','LATERAL_ACCELERATION')):return 'STEERING_GUARD'
    if 'RAMP' in s:return 'RAMP'
    if 'DEPTH_CORNER' in s:return 'DEPTH_CORNER'
    if any(k in s for k in ('PREVIEW','DEPTH_SUPPORTED','DEPTH_UNAVAILABLE','VISIBLE_PATH')):return 'CORNER_PREVIEW'
    if any(k in s for k in ('CONFIDENCE','PATH_SOURCE')):return 'PATH_CONFIDENCE'
    return 'OTHER'

class DriveStability:
    def __init__(self):self.previous_severity=None;self.path=None;self.previous_features=None
    def snapshot(self,trace,path,*,stamp,requested,final_target,final,actual,braking,clamp,limiter,stop_reason,
                 source,desired_lookahead,effective_lookahead,lookahead_speed,target,raw_steering,steering):
        severity=trace.get('depth_corner_severity','UNKNOWN')
        previous=self.previous_severity;self.previous_severity=severity
        metrics=dict(lidar_center_y=None,path_heading=None,path_curvature=None,raw_path_curvature=None,lateral_jump=None,heading_jump=None,curvature_jump=None)
        if valid_path(path):
            heading,curvature=path_geometry(path)
            metrics.update(lidar_center_y=float(np.interp(1.5,path[:,0],path[:,1])),path_heading=float(np.median(heading[:15])),
                path_curvature=float(np.max(abs(supported_curvature(path)))),raw_path_curvature=float(np.max(abs(curvature))))
            if self.path is not None:
                lo=max(path[0,0],self.path[0,0]);hi=min(path[-1,0],self.path[-1,0])
                x=np.arange(lo,hi,.1)
                if len(x):metrics['lateral_jump']=float(np.max(abs(np.interp(x,path[:,0],path[:,1])-np.interp(x,self.path[:,0],self.path[:,1]))))
            if self.previous_features is not None:
                metrics['heading_jump']=metrics['path_heading']-self.previous_features['path_heading']
                metrics['curvature_jump']=metrics['path_curvature']-self.previous_features['path_curvature']
            self.path=path.copy();self.previous_features=metrics.copy()
        reduction=trace.get('speed_reduction_reason',limiter) if limiter=='SPEED_INCREASE_CONFIRMATION' else limiter
        if limiter=='DEPTH_UNAVAILABLE' and 'TIMEOUT' in trace.get('depth_unavailable_reason',''):reduction='SENSOR_TIMEOUT'
        record=dict(trace,stamp=stamp,requested_speed=requested,decision_target_speed=requested,
            final_target_speed=final_target,final_speed=final,actual_speed=actual,
            previous_corner_severity=previous,severity_change_reason='UNCHANGED' if previous==severity else trace.get('severity_change_reason',trace.get('depth_state','SOURCE_UNAVAILABLE')),
            distance_to_corner=trace.get('depth_distance_to_corner_m'),remaining_corner_distance=trace.get('depth_remaining_corner_distance'),
            corner_preview_speed_limit=trace.get('corner_preview_braking_limit'),
            corner_braking=braking,corner_braking_clamp=clamp,active_speed_limiter=limiter,
            braking_reason=reduction_reason(reduction,stop_reason,source),stop_reason=stop_reason,
            selected_steering_source=source,lookahead=effective_lookahead,desired_lookahead=desired_lookahead,
            effective_lookahead=effective_lookahead,lookahead_speed=lookahead_speed,
            lidar_target_y=float(target[1]) if target is not None else None,
            target_x=float(target[0]) if target is not None else None,target_y=float(target[1]) if target is not None else None,
            raw_pp_steering=raw_steering,final_steering=steering,
            braking_invariant_ok=not (braking or clamp) or final<=requested+1e-6,**metrics)
        return finite_payload(record)
