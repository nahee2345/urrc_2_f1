"""Bounded, provenance-preserving selected trajectory memory.

Motion uses fresh odometry twist, or existing commands when unavailable.
No IMU yaw or world pose is read. Live
LiDAR scan health and the source sensor health constrain temporary holds.
"""
from dataclasses import dataclass
import math
import numpy as np
from .navigation import valid_path,valid_boundary,pure_pursuit,path_geometry
from .corner import SEVERITIES
from .corridor_geometry import resample_x,same_x,drive_prefix,normal_hit,supported_prefix
from .obstacle_pass import inside_corridor


@dataclass
class RecoveryConfig:
    enabled: bool = True
    allow_general_path_hold: bool = False
    corner_path_hold_sec: float = .50
    corner_path_hold_speed_mps: float = 1.2
    path_loss_confirm_frames: int = 3
    reacquire_confirm_frames: int = 3
    memory_min_confidence: float = .60
    commit_min_confidence: float = .75
    enable_single_wall_fallback: bool = True
    enable_previous_trajectory_fallback: bool = True

    def validate(self):
        if (not .0<self.corner_path_hold_sec<=.5 or not 0<self.corner_path_hold_speed_mps<=1.5 or
            not 0<self.memory_min_confidence<=self.commit_min_confidence<=1 or
            min(self.path_loss_confirm_frames,self.reacquire_confirm_frames)<2):
            raise ValueError('invalid corner path recovery configuration')


def transformed_paths(paths,motion):
    x,y,heading=motion;c,s=math.cos(heading),math.sin(heading)
    rotation=np.array([[c,-s],[s,c]])
    projected={}
    for name,path in paths.items():
        p=(path-np.array([x,y]))@rotation
        keep=(p[:,0]>.05)&(p[:,0]<=5.)
        p=p[keep]
        if name in ('center','selected'):
            p=drive_prefix(resample_x(p))
            if p is None:raise ValueError('no_forward_path')
        if len(p)<3:raise ValueError('no_forward_path')
        projected[name]=p
    for name in ('center','selected'):
        projected[name]=drive_prefix(supported_prefix(projected[name],projected['left'],projected['right']))
    pair=same_x([projected['center'],projected['selected']])
    if pair is None:raise ValueError('no_forward_path')
    result=dict(projected,center=pair[0],selected=pair[1])
    if not all((valid_path if name in ('center','selected') else valid_boundary)(p) for name,p in result.items()):raise ValueError('no_forward_path')
    return result


class CornerRecovery:
    def __init__(self,config=None,wheelbase=.34,vehicle_width=.24165,camera_timeout=.25):
        self.c=config or RecoveryConfig();self.c.validate()
        self.wheelbase=wheelbase;self.width=vehicle_width;self.camera_timeout=camera_timeout
        self.image_ready=False;self.image_time=-math.inf;self.scan_ready=False;self.scan_time=-math.inf
        self.memory=None;self.committed=False;self.expired=False;self.phase='STRAIGHT'
        self.last_input_stamp=-math.inf;self.last_input_time=-math.inf;self.input_good=False
        self.current_corridor_ok=True
        self.issue='no_camera_path';self.loss_frames=0;self.forward_bad_frames=0;self.reacquire_frames=0
        self.last_saved_stamp=-math.inf;self.motion=np.zeros(3);self.motion_time=None
        self.entry_time=None;self.direction='UNKNOWN';self.track_width=None;self.width_time=-math.inf
        self.last_context=None;self.stop_reason='';self.first_failure=None

    def health(self,ready,now):self.image_ready=bool(ready);self.image_time=now

    def lidar_health(self,ready,now):self.scan_ready=bool(ready);self.scan_time=now

    def image_fresh(self,now):return self.image_ready and 0<=now-self.image_time<=self.camera_timeout

    def reset(self):
        self.memory=None;self.committed=False;self.expired=False;self.phase='STRAIGHT'
        self.loss_frames=self.forward_bad_frames=self.reacquire_frames=0
        self.last_context=None;self.stop_reason='';self.motion=np.zeros(3);self.entry_time=None

    def integrate(self,now,speed,steering,twist=None):
        if self.motion_time is not None:
            dt=max(0.,min(now-self.motion_time,self.c.corner_path_hold_sec))
            vx,vy,yaw_rate=twist if twist is not None else (max(0.,speed),0.,max(0.,speed)*math.tan(steering)/self.wheelbase)
            heading=self.motion[2]+.5*yaw_rate*dt
            self.motion[:2]+=dt*np.array([vx*math.cos(heading)-vy*math.sin(heading),vx*math.sin(heading)+vy*math.cos(heading)])
            self.motion[2]+=yaw_rate*dt
        self.motion_time=now

    def note_input(self,stamp,now,accepted,confidence,observed,issue,forward_bad=False,corridor_ok=True):
        if stamp<=self.last_input_stamp:return
        self.last_input_stamp=stamp;self.last_input_time=now
        self.input_good=bool(accepted and math.isfinite(confidence) and confidence>=self.c.memory_min_confidence)
        if accepted:self.current_corridor_ok=bool(corridor_ok)
        self.issue=issue if not self.input_good else ''
        self.loss_frames=0 if self.input_good else self.loss_frames+1
        self.forward_bad_frames=self.forward_bad_frames+1 if forward_bad else 0
        self.reacquire_frames=self.reacquire_frames+1 if self.input_good else 0
        if self.committed and not self.input_good and self.first_failure is None:
            self.first_failure=dict(stamp=stamp,issue=issue,left_wall_valid=observed[0],right_wall_valid=observed[1],confidence=confidence)

    def prediction(self):
        if self.memory is None:return None
        return transformed_paths(self.memory['paths'],self.motion)

    def note_width(self,width,age,now):
        if math.isfinite(width) and self.width<width<=4. and 0<=age<=2.:
            self.track_width=width;self.width_time=now-age

    def observe_safe(self,result,corner,engine,now,stamp):
        if (not self.c.enabled or not result['enabled'] or not self.input_good or
            engine.camera_stamp!=stamp or stamp<=self.last_saved_stamp or not any(engine.observed) or not valid_path(result['path'])):return
        if not self.committed:
            single_recent=(any(engine.observed) and not all(engine.observed) and self.track_width is not None and 0<=now-self.width_time<=2.)
            entry_confidence=self.c.memory_min_confidence if single_recent else self.c.commit_min_confidence
            independent=engine.path_source in ('LIDAR_PATH','FUSED_PATH') and self.scan_ready and now-self.scan_time<=self.camera_timeout
            if ((not self.c.allow_general_path_hold and (corner.state!='CORNER_COMMIT' or corner.level==0)) or engine.confidence<entry_confidence or
                not (self.image_fresh(now) or independent)):return
            self.committed=True;self.entry_time=now;self.phase='CORNER_COMMIT';self.first_failure=None
        if corner.state=='CORNER_EXIT':
            self.phase='CORNER_EXIT'
            if corner.level==0:return
        elif corner.state=='CORNER_COMMIT':self.phase='CORNER_COMMIT'
        if corner.level==0 and corner.state=='STRAIGHT':
            if not self.c.allow_general_path_hold:self.reset();return
            self.phase='STRAIGHT'
        try:_,target,_=pure_pursuit(result['path'],engine.actual_speed,self.wheelbase)
        except ValueError:return
        paths=dict(center=engine.path.copy(),left=engine.left.copy(),right=engine.right.copy(),selected=result['path'].copy())
        if not all((valid_path if name in ('center','selected') else valid_boundary)(p) for name,p in paths.items()):return
        if all(engine.observed):
            from .corridor_geometry import normal_distances
            widths=normal_distances(engine.path,engine.left,1)-normal_distances(engine.path,engine.right,-1)
            widths=widths[np.isfinite(widths)&(widths>=.55)&(widths<=4.)]
            if len(widths):
                measured=float(np.median(widths))
                self.track_width=measured if self.track_width is None else .8*self.track_width+.2*measured
                self.width_time=engine.camera_time
        h,k=path_geometry(engine.path);indices=np.flatnonzero(np.abs(k)>=corner.c.mild_curvature_enter)
        start=int(indices[0]) if len(indices) else 0
        turn=float(np.median(k[start:min(start+5,len(k))]))
        if abs(turn)>.02:self.direction='LEFT' if turn>0 else 'RIGHT'
        self.memory=dict(paths=paths,time=engine.camera_time,stamp=stamp,target=target.copy(),confidence=engine.confidence,
                         severity=result['corner_severity'],observed=engine.observed,source=engine.path_source)
        self.last_saved_stamp=stamp;self.motion=np.zeros(3);self.motion_time=max(engine.camera_time,engine.geometry_motion_time or engine.camera_time)
        self.expired=False;self.stop_reason=''

    def prepare(self,now,ros_now,speed,steering,twist=None,force_hold=False,independent_geometry=False):
        self.integrate(now,speed,steering,twist);self.last_context=None;self.stop_reason=''
        if not self.committed or self.memory is None or not self.c.enabled:return None
        source_age=now-self.memory['time']
        current_good=self.input_good and now-self.last_input_time<=self.camera_timeout
        if current_good and independent_geometry and not force_hold:return None
        independent_memory=self.memory.get('source') in ('LIDAR_PATH','FUSED_PATH') and self.scan_ready and now-self.scan_time<=self.camera_timeout
        if not (self.image_fresh(now) or independent_memory):self.expired=True;self.stop_reason='camera_timeout';return None
        if not self.current_corridor_ok:self.expired=True;self.stop_reason='corridor_lost';return None
        if current_good and not force_hold:
            if self.expired and self.reacquire_frames<self.c.reacquire_confirm_frames:self.stop_reason='recovering_path_confirmation'
            return None
        if self.forward_bad_frames>=self.c.path_loss_confirm_frames:self.expired=True;self.stop_reason='reverse_risk_persistent';return None
        if source_age>self.c.corner_path_hold_sec or not self.c.enable_previous_trajectory_fallback:
            self.expired=True;self.stop_reason='path_lost_timeout';return None
        if self.expired:
            self.stop_reason='recovering_path_confirmation';return None
        try:
            paths=self.prediction()
            left=normal_hit(np.array([.2,0.]),np.array([0.,1.]),paths['left'],1)
            right=normal_hit(np.array([.2,0.]),np.array([0.,1.]),paths['right'],-1)
            if left is not None and right is not None and min(left,-right)<self.width/2:
                self.expired=True;self.stop_reason='corridor_lost';return None
            _,target,_=pure_pursuit(paths['selected'],speed,self.wheelbase)
        except (ValueError,FloatingPointError):self.stop_reason='no_forward_target';return None
        context=dict(holding=True,phase='PATH_HOLD' if self.memory['severity']=='STRAIGHT' else 'CORNER_COMMIT',source_sensor=self.memory.get('source','CAMERA_PATH'),frame_id='team_01/base_link',source_stamp=self.memory['stamp'],
                     projection_stamp=ros_now,hold_age=source_age,source_confidence=self.memory['confidence'],
                     camera_alive=True,severity=self.memory['severity'],corner_direction=self.direction,
                     hold_speed=self.c.corner_path_hold_speed_mps,paths=paths,
                     target=target,track_width=self.track_width,entry_time=self.entry_time,
                     issue=self.issue or 'camera_path_stale')
        self.last_context=context
        return context

    def diagnostics(self,now):
        return dict(corner_state=self.phase,corner_direction=self.direction,corner_entry_timestamp=self.entry_time,
            last_valid_path_timestamp=self.memory['stamp'] if self.memory else None,estimated_track_width=self.track_width,
            using_previous_trajectory=self.last_context is not None,trajectory_hold_age=now-self.memory['time'] if self.memory else None,
            path_loss_frames=self.loss_frames,path_loss_confirmed=self.loss_frames>=self.c.path_loss_confirm_frames,
            recovery_expired=self.expired,reacquire_frames=self.reacquire_frames,first_path_failure=self.first_failure)


def wire_context(context):
    if context is None:return dict(holding=False)
    return {k:({n:p.tolist() for n,p in v.items()} if k=='paths' else v.tolist() if isinstance(v,np.ndarray) else v) for k,v in context.items()}


class RecoveryReceiver:
    """Verify age at both clocks; replay never renews the source deadline."""
    def __init__(self,config=None,camera_timeout=.25):
        self.c=config or RecoveryConfig();self.c.validate();self.camera_timeout=camera_timeout
        self.data=None;self.received=-math.inf;self.origin_stamp=-math.inf;self.origin_time=-math.inf
        self.image_ready=False;self.image_time=-math.inf;self.scan_ready=False;self.scan_time=-math.inf

    def health(self,ready,now):self.image_ready=bool(ready);self.image_time=now

    def lidar_health(self,ready,now):self.scan_ready=bool(ready);self.scan_time=now

    def update(self,data,ros_now,now):
        if not isinstance(data,dict):self.data=None;return False
        if not data.get('holding'):self.data=None;return False
        try:
            source=float(data['source_stamp']);projection=float(data['projection_stamp']);age=float(data['hold_age'])
            confidence=float(data['source_confidence'])
            if (data['phase'] not in ('CORNER_COMMIT','PATH_HOLD') or data['frame_id']!='team_01/base_link' or data['severity'] not in SEVERITIES or
                (data['severity']=='STRAIGHT' and data['phase']!='PATH_HOLD') or
                not all(math.isfinite(v) for v in (source,projection,age,confidence)) or
                not 0<=age<=self.c.corner_path_hold_sec or not -.05<=ros_now-source<=self.c.corner_path_hold_sec or
                not -.05<=ros_now-projection<=.15 or confidence<self.c.memory_min_confidence or source<self.origin_stamp):return False
            paths={name:np.asarray(data['paths'][name],dtype=float) for name in ('center','left','right','selected')}
            if not all((valid_path if name in ('center','selected') else valid_boundary)(p) for name,p in paths.items()) or not inside_corridor(paths['center'],paths['left'],paths['right'],0.) or not inside_corridor(paths['selected'],paths['left'],paths['right'],0.):return False
            if source>self.origin_stamp:self.origin_stamp=source;self.origin_time=now-age
            self.data=dict(data,paths=paths);self.received=now
            return self.context(ros_now,now) is not None
        except (ValueError,KeyError,TypeError,IndexError):return False

    def context(self,ros_now,now):
        d=self.data
        alive=(self.image_ready and now-self.image_time<=self.camera_timeout) or (d is not None and d.get('source_sensor') in ('LIDAR_PATH','FUSED_PATH') and self.scan_ready and now-self.scan_time<=self.camera_timeout)
        if (not self.c.enabled or d is None or not alive or
            now-self.received>.15 or now-self.origin_time>self.c.corner_path_hold_sec or
            ros_now-d['source_stamp']>self.c.corner_path_hold_sec or ros_now-d['projection_stamp']>.15):return None
        return d

    def wall_context(self,ros_now,now):
        d=self.context(ros_now,now)
        if d is None:return None
        p=d['paths'];lo=max(v[0,0] for v in p.values());hi=min(v[-1,0] for v in p.values())
        return dict(**p,lo=lo,hi=hi,confidence=min(.68,d['source_confidence']),observed=(False,False),
                    stamp=d['projection_stamp'],time=now-max(0.,ros_now-d['projection_stamp']),
                    recovery=True,source_stamp=d['source_stamp'],source_age=now-self.origin_time)
