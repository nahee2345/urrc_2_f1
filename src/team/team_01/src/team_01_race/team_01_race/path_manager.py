"""Camera/LiDAR geometry agreement and confirmed source selection."""
from dataclasses import dataclass
import math
import numpy as np
from .navigation import valid_path,valid_boundary,path_geometry
from .path_curvature import supported_curvature
from .lidar_track import LidarTrack
from .corridor_geometry import same_x,x_ordered
from .obstacle_pass import inside_corridor

@dataclass
class PathManagerConfig:
    enable_lidar_path: bool=True
    enable_camera_lidar_fusion: bool=True
    lidar_only_max_speed_mps: float=2.
    fused_max_speed_mps: float=2.5
    previous_path_max_speed_mps: float=1.2
    source_switch_confirm_frames: int=3
    camera_recovery_confirm_frames: int=3
    camera_recovery_min_duration_sec: float=.20
    minimum_source_hold_time: float=.30
    source_switch_confidence_margin: float=.05
    source_geometry_agreement_threshold: float=.25
    camera_degradation_confirm_frames: int=3
    max_curvature_difference: float=.15
    max_lateral_difference_m: float=.25
    max_heading_difference_deg: float=15.
    max_width_difference_m: float=.3
    max_source_stamp_skew_sec: float=.15

def lidar_corridor(points,indices):
    return LidarTrack().update(points,indices,1.,1.)

def geometry_valid(d):
    good=bool(d is not None and math.isfinite(d['confidence']) and .4<=d['confidence']<=1 and valid_path(d['path']) and all(valid_boundary(d[k]) for k in ('left','right')))
    if not good:return False
    return bool(inside_corridor(d['path'],d['left'],d['right'],0.))

def geometry_difference(camera,lidar):
    if not all(x_ordered(d[k]) for d in (camera,lidar) for k in ('left','right')):return None,None
    aligned=same_x([camera['path'],lidar['path'],camera['left'],camera['right'],lidar['left'],lidar['right']])
    if aligned is None:return None,None
    c,l,cl,cr,ll,lr=aligned
    if len(c)<6:return None,None
    ch,_=path_geometry(c);lh,_=path_geometry(l)
    ck=supported_curvature(c);lk=supported_curvature(l)
    d=dict(curvature=float(np.max(abs(np.percentile(ck,[10,50,90])-np.percentile(lk,[10,50,90])))),lateral=float(np.max(abs(c[:,1]-l[:,1]))),heading_deg=float(np.degrees(np.max(abs(np.arctan2(np.sin(ch-lh),np.cos(ch-lh)))))),width=float(np.max(abs((cl[:,1]-cr[:,1])*np.cos(ch)-(ll[:,1]-lr[:,1])*np.cos(lh)))))
    return d,aligned

def select_geometry(camera,lidar,config=None):
    cfg=config or PathManagerConfig();cg=geometry_valid(camera);lg=cfg.enable_lidar_path and geometry_valid(lidar)
    if cg and camera['confidence']>=.75:return camera,'CAMERA_PATH'
    if cg and lg and cfg.enable_camera_lidar_fusion:
        diff,a=geometry_difference(camera,lidar)
        if diff is not None and diff['lateral']<=cfg.max_lateral_difference_m and diff['heading_deg']<=cfg.max_heading_difference_deg and diff['width']<=cfg.max_width_difference_m:
            c,l,cl,cr,ll,lr=a;wc=camera['confidence']/(camera['confidence']+lidar['confidence']);d=dict(camera)
            for key,x,y in [('path',c,l),('left',cl,ll),('right',cr,lr)]:d[key]=wc*x+(1-wc)*y
            if geometry_valid(d):
                d.update(confidence=min(.9,.5*(camera['confidence']+lidar['confidence'])+.05),path_source='FUSED_PATH',reason='agreeing_camera_lidar',agreement=diff)
                return d,'FUSED_PATH'
    if lg:return lidar,'LIDAR_PATH'
    if cg:return camera,'CAMERA_PATH'
    return None,'NO_PATH'

class SourceSelector:
    STATES={'CAMERA_PATH':'CAMERA_ACTIVE','LIDAR_PATH':'LIDAR_ACTIVE',
            'FUSED_PATH':'FUSED_ACTIVE','NO_PATH':'SOURCE_RECOVERY'}
    def __init__(self,config=None):
        self.c=config or PathManagerConfig()
        if (min(self.c.source_switch_confirm_frames,self.c.camera_recovery_confirm_frames,self.c.camera_degradation_confirm_frames)<2 or
                not 0<self.c.previous_path_max_speed_mps<=1.5 or not 0<self.c.max_source_stamp_skew_sec<=.25 or
                not 0<=self.c.camera_recovery_min_duration_sec<=.5 or not 0<=self.c.minimum_source_hold_time<=1. or not 0<=self.c.source_switch_confidence_margin<.25 or
                self.c.source_geometry_agreement_threshold<=0):raise ValueError('invalid source policy')
        self.source='NO_PATH';self.pending=None;self.count=0;self.last_key=None;self.reason='no_geometry'
        self.candidate_since=None;self.last_confirmation=0;self.since=-math.inf;self.degraded=0;self.camera_key=None;self.geometry_equivalent=False
    @property
    def state(self):return self.STATES[self.source]
    def _commit(self,geometry,source,now,reason):
        if source!=self.source:self.since=now
        self.last_confirmation=self.count
        self.source=source;self.reason=reason;self.pending=None;self.count=0;self.last_key=None
        return geometry,source
    def select(self,camera,lidar,keys,now=None):
        now=max((k for k in keys.values() if k is not None),default=0.) if now is None else now
        # Absolute freshness is authoritative. Inter-sensor skew constrains
        # fusion, not the validity of an independently fresh Camera path.
        def fresh(d,key):
            stamp=keys.get(key)
            return d if geometry_valid(d) and stamp is not None and -.05<=now-stamp<=.25 else None
        camera=fresh(camera,'camera');lidar=fresh(lidar,'lidar') if self.c.enable_lidar_path else None
        diff,_=geometry_difference(camera,lidar) if camera is not None and lidar is not None else (None,None)
        agreeing=bool(diff is not None and diff['lateral']<=self.c.source_geometry_agreement_threshold and
            diff['heading_deg']<=self.c.max_heading_difference_deg and diff['width']<=self.c.max_width_difference_m and
            diff['curvature']<=self.c.max_curvature_difference)
        self.geometry_equivalent=agreeing
        skew=abs(keys['camera']-keys['lidar']) if camera is not None and lidar is not None else math.inf
        fused=None
        if agreeing and skew<=self.c.max_source_stamp_skew_sec and self.c.enable_camera_lidar_fusion:
            fused,kind=select_geometry(dict(camera,confidence=min(.74,camera['confidence'])),lidar,self.c)
            if kind!='FUSED_PATH':fused=None
        available={'CAMERA_PATH':camera,'LIDAR_PATH':lidar,'FUSED_PATH':fused}
        key=keys.get('camera')
        if key!=self.camera_key:
            self.degraded=self.degraded+1 if camera is not None and camera['confidence']<.75 else 0
            self.camera_key=key
        desired,source=select_geometry(camera,lidar,self.c)
        if source=='FUSED_PATH' and fused is None:desired,source=(lidar,'LIDAR_PATH') if lidar is not None else (camera,'CAMERA_PATH')
        current=available.get(self.source)
        if desired is None:return self._commit(None,'NO_PATH',now,'no_current_geometry')
        if self.source=='NO_PATH':
            # New startup authority still requires distinct sensor frames.
            pass
        elif current is None:
            # Safety descent bypasses hold time and promotion confirmation.
            return self._commit(desired,source,now,'sensor_stale' if self.source=='CAMERA_PATH' and camera is None else 'current_source_invalid')
        elif self.source=='CAMERA_PATH' and (camera['confidence']>=.75 or self.degraded<self.c.camera_degradation_confirm_frames):
            self.pending=None;self.count=0;self.last_key=None;self.reason='camera_active_valid' if camera['confidence']>=.75 else 'transient_camera_degradation'
            return camera,self.source
        elif source==self.source:
            self.pending=None;self.count=0;self.last_key=None;self.reason='current_source_valid';return current,self.source
        elif now-self.since<self.c.minimum_source_hold_time:
            self.pending=None;self.count=0;self.last_key=None;self.reason='minimum_source_hold';return current,self.source
        if source=='CAMERA_PATH' and self.source in ('LIDAR_PATH','FUSED_PATH'):
            if camera['confidence']<.75+self.c.source_switch_confidence_margin or (lidar is not None and not agreeing):
                self.pending=None;self.count=0;self.last_key=None;self.reason='camera_recovery_disagreement' if not agreeing else 'camera_recovery_confidence'
                return current,self.source
        candidate_key=(keys.get('camera'),keys.get('lidar')) if source=='FUSED_PATH' else keys.get('camera' if source=='CAMERA_PATH' else 'lidar')
        if self.pending!=source:self.pending=source;self.count=0;self.last_key=None;self.candidate_since=now
        new_measurement=candidate_key!=self.last_key
        if new_measurement:self.count+=1;self.last_key=candidate_key
        required=self.c.camera_recovery_confirm_frames if source=='CAMERA_PATH' and self.source!='NO_PATH' else self.c.source_switch_confirm_frames
        duration_ready=(source!='CAMERA_PATH' or self.source=='NO_PATH' or now-(self.candidate_since if self.candidate_since is not None else now)+1e-8>=self.c.camera_recovery_min_duration_sec)
        if self.count>=required and duration_ready and new_measurement:return self._commit(desired,source,now,'confirmed_camera_recovery' if source=='CAMERA_PATH' else 'confirmed_source')
        self.reason='confirming_preferred_source'
        return (current,self.source) if current is not None else (None,'NO_PATH')
