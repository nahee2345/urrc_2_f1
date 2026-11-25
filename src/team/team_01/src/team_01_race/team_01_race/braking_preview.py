"""Measured long-range Camera visibility and corner warnings; no steering authority."""
from dataclasses import dataclass
import math
import numpy as np
from .corner import CornerConfig,preview_geometry
from .navigation import valid_path

@dataclass
class BrakingPreviewConfig:
    enabled: bool=True
    horizon_m: float=25.
    min_confidence: float=.40
    timeout_sec: float=.25
    confirm_frames: int=2

class BrakingPreview:
    def __init__(self,config=None,corner=None,normal_speed=10.):
        self.c=config or BrakingPreviewConfig();self.corner=corner or CornerConfig();self.normal=normal_speed
        if not 5<=self.c.horizon_m<=25 or not .4<=self.c.min_confidence<=1 or not 0<self.c.timeout_sec<=.5 or self.c.confirm_frames<2:raise ValueError('invalid braking preview config')
        self.stamp=-math.inf;self.time=-math.inf;self.count=0;self.level=0;self.data=None;self.confirmed=None;self.confirmed_time=-math.inf;self.speed_data=None;self.speed_path=None;self.geometry=None;self.packet=None
    def update(self,data,ros_now,now):
        try:
            stamp=float(data['stamp']);confidence=float(data['confidence']);p=np.asarray(data['path'],dtype=float)
            if (not self.c.enabled or data['frame_id']!='team_01/base_link' or not math.isfinite(stamp) or stamp<=self.stamp or
                not -.05<=ros_now-stamp<=self.c.timeout_sec):return False
            self.stamp=stamp
            if (not math.isfinite(confidence) or confidence<self.c.min_confidence or not valid_path(p,self.c.horizon_m)):
                self.count=0;return False
            m=preview_geometry(p,self.corner,horizon=self.c.horizon_m)
            self.speed_path=p.copy();self.speed_data=dict(confidence=confidence,time=now-max(0.,ros_now-stamp))
            self.packet=data
            geometry=data.get('geometry')
            self.geometry=geometry
            level=m['candidate_level']
            if m['max_future_curvature']>=self.corner.sharp_curvature_enter and m['confirmation_heading_deg']>=self.corner.mild_heading_enter_deg:level=3
            if geometry is not None:
                distance=geometry.get('distance_to_first_significant_curvature')
                level=0
                if distance is not None and geometry.get('curvature_supported_preview_m',0)>=distance+.8:
                    supported=[abs(s['curvature']) for s in geometry.get('curvature_samples',[]) if distance<=s['distance']<=distance+1.2 and s.get('wall_agreement')]
                    if len(supported)>=6:
                        k=float(np.percentile(supported,90))
                        level=max(1,sum(k>=threshold for threshold in self.corner.curvature_enters()))
                        m['distance_to_corner']=distance
            self.count=self.count+1 if level==self.level and now-self.time<=self.c.timeout_sec else 1
            self.level=level;self.time=now-max(0.,ros_now-stamp)
            self.data=dict(speed_limit=self.corner.speeds(self.normal)[level],level=level,source_stamp=stamp,
                           confidence=confidence,curvature_supported_preview_m=geometry.get('curvature_supported_preview_m',0.) if geometry else 0.,distance_to_corner=m['distance_to_corner'] if math.isfinite(m['distance_to_corner']) else None,horizon_m=self.c.horizon_m)
            if self.count>=self.c.confirm_frames:
                self.confirmed=self.data.copy() if level>0 else None
                self.confirmed_time=self.time
            return True
        except (ValueError,TypeError,KeyError,IndexError):return False
    def context(self,now):
        return self.confirmed if self.c.enabled and self.confirmed is not None and 0<=now-self.confirmed_time<=self.c.timeout_sec else None

    def speed_context(self,now,clear=False):
        from .preview_speed import preview_speed
        if self.speed_data is None:return None
        age=now-self.speed_data['time']
        if not 0<=age<=self.c.timeout_sec:return None
        return preview_speed(self.speed_path,self.speed_data['confidence'],age,
                             stable=(self.count>=self.c.confirm_frames and self.level==0 and
                                     (self.geometry is None or (self.geometry.get('curvature_supported_preview_m',0.)>=min(20.,float(self.speed_path[-1,0])) and
                                      any(lo<=self.corner.preview_distance_m and hi>=20. for lo,hi in self.geometry.get('curvature_support_intervals_m',[]))))),
                             clear=clear,horizon=self.c.horizon_m,max_speed=self.normal)
