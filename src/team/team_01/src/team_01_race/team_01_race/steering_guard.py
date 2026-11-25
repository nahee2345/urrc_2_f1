"""Restrict Camera/Pure-Pursuit commands using time-aligned IMU feedback.

No map, absolute heading target, or IMU-generated steering is used.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np
from .navigation import valid_path, valid_boundary, path_geometry
from .path_curvature import supported_curvature


def angle(value):
    return math.atan2(math.sin(value),math.cos(value))


@dataclass
class SteeringGuardConfig:
    enabled: bool = True
    # Legacy yaw protection is opt-in only; production IMU is pitch-only.
    yaw_safety_enabled: bool = False
    straight_max_steering_deg: float = 8.0
    corner_max_steering_deg: float = 20.0
    recovery_max_steering_deg: float = 8.0
    imu_yaw_rate_alpha: float = .30
    imu_yaw_alpha: float = .30
    imu_timeout_sec: float = .25
    imu_stop_timeout_sec: float = 1.0
    imu_max_plausible_yaw_rate_deg_s: float = 720.0
    imu_spike_delta_deg_s: float = 90.0
    imu_spike_confirm_frames: int = 2
    yaw_rate_prediction_sec: float = .20
    max_observed_lateral_accel_mps2: float = 3.0
    straight_max_yaw_rate_deg_s: float = 40.0
    corner_max_yaw_rate_deg_s: float = 120.0
    straight_heading_warning_deg: float = 12.0
    straight_heading_stop_deg: float = 45.0
    corner_heading_warning_deg: float = 30.0
    corner_heading_stop_deg: float = 80.0
    recovery_speed_mps: float = 2.0
    recovery_exit_confirm_frames: int = 5
    recovery_heading_deg: float = 5.0
    recovery_yaw_rate_deg_s: float = 10.0
    heading_growth_min_yaw_rate_deg_s: float = 1.0
    gentle_curve_min_curvature: float = .02
    gentle_curve_confirm_frames: int = 5
    steering_relaxation_deg_s: float = 60.0
    target_jump_m: float = .35
    path_heading_jump_deg: float = 30.0
    path_curvature_jump: float = .35
    path_jump_confirm_frames: int = 2
    path_hold_sec: float = .25
    camera_timeout_sec: float = .25
    camera_min_confidence: float = .40
    recovery_min_confidence: float = .75
    corner_current_curvature_threshold: float = .10

    def validate(self):
        numeric=[v for v in vars(self).values() if not isinstance(v,bool)]
        if (not all(math.isfinite(v) and v>0 for v in numeric) or
                not 0 < self.imu_yaw_rate_alpha <= 1 or not 0 < self.imu_yaw_alpha <= 1 or
                not self.recovery_max_steering_deg <= self.corner_max_steering_deg <= 30 or
                self.straight_max_steering_deg > 30 or
                not self.recovery_heading_deg < self.straight_heading_warning_deg < self.straight_heading_stop_deg < 90 or
                self.imu_stop_timeout_sec < self.imu_timeout_sec or
                min(self.imu_spike_confirm_frames,self.path_jump_confirm_frames,self.recovery_exit_confirm_frames)<2):
            raise ValueError('invalid steering guard configuration')


class ImuRotation:
    def __init__(self,config):
        self.c=config
        self.last_stamp=self.last_valid_time=-math.inf
        self.yaw=self.unwrapped=self.rate=self.raw_rate=None
        self.raw_yaw=None
        self.valid=False;self.error='no_imu';self.spike_suspect=False
        self.pending_rate=None;self.rate_count=0;self.pending_yaw=None;self.yaw_count=0
        self.history=deque(maxlen=300)
        self.accepted_samples=0
        self.rate_accel=0.

    def invalidate(self,reason):
        self.valid=False;self.error=reason
        self.pending_rate=self.pending_yaw=None;self.rate_count=self.yaw_count=0
        self.accepted_samples=0

    def fresh(self,now):
        return self.valid and 0 <= now-self.last_valid_time <= self.c.imu_timeout_sec

    def update(self,quaternion,rate,stamp,now,age=0.,available=True):
        if not all(math.isfinite(v) for v in (stamp,now,age)) or age < -.05:
            self.invalidate('invalid_imu_time');return False
        if stamp<=self.last_stamp:
            return False
        dt=stamp-self.last_stamp;self.last_stamp=stamp
        values=np.asarray(quaternion,dtype=float)
        norm=np.linalg.norm(values)
        if (not available or age>self.c.imu_timeout_sec or not np.isfinite(values).all() or
                not .9<=norm<=1.1 or not math.isfinite(rate)):
            self.invalidate('invalid_or_stale_imu');return False
        x,y,z,w=values/norm
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        raw=math.degrees(rate);self.raw_rate=raw
        fresh_before=self.fresh(now)
        if now-self.last_valid_time>self.c.imu_timeout_sec:
            self.accepted_samples=0
        self.spike_suspect=False
        accepted_rate=raw
        if self.rate is not None and fresh_before and abs(raw-self.rate)>self.c.imu_spike_delta_deg_s:
            consistent=(self.pending_rate is not None and raw*self.pending_rate>0 and
                        abs(raw-self.pending_rate)<=max(self.c.imu_spike_delta_deg_s,abs(raw)*.5))
            self.rate_count=self.rate_count+1 if consistent else 1;self.pending_rate=raw
            if self.rate_count<self.c.imu_spike_confirm_frames:
                accepted_rate=self.rate;self.spike_suspect=True
        else:
            self.rate_count=0;self.pending_rate=None
        if abs(raw)>self.c.imu_max_plausible_yaw_rate_deg_s:
            corroborated=(self.raw_yaw is not None and dt>0 and
                abs(math.degrees(angle(yaw-self.raw_yaw))/dt)>.5*self.c.imu_max_plausible_yaw_rate_deg_s)
            if self.rate_count<self.c.imu_spike_confirm_frames or not corroborated:
                self.invalidate('implausible_imu_rate');return False
        previous_rate=self.rate
        accepted_yaw=yaw
        if self.raw_yaw is not None and fresh_before and abs(math.degrees(angle(yaw-self.raw_yaw)))>max(8.,2*self.c.imu_max_plausible_yaw_rate_deg_s*dt):
            consistent=self.pending_yaw is not None and abs(math.degrees(angle(yaw-self.pending_yaw)))<5.
            self.yaw_count=self.yaw_count+1 if consistent else 1;self.pending_yaw=yaw
            if self.yaw_count<self.c.imu_spike_confirm_frames:
                accepted_yaw=self.raw_yaw;self.spike_suspect=True
        else:
            self.pending_yaw=None;self.yaw_count=0
        self.raw_yaw=accepted_yaw
        if self.yaw is None or not fresh_before:
            self.yaw=accepted_yaw
            if self.unwrapped is None:self.unwrapped=self.yaw
            else:self.unwrapped+=angle(self.yaw-angle(self.unwrapped))
            self.rate=accepted_rate
        else:
            change=self.c.imu_yaw_alpha*angle(accepted_yaw-self.yaw)
            self.yaw=angle(self.yaw+change);self.unwrapped+=change
            self.rate=self.c.imu_yaw_rate_alpha*accepted_rate+(1-self.c.imu_yaw_rate_alpha)*self.rate
        if previous_rate is not None and fresh_before and 0<dt<=self.c.imu_timeout_sec and not self.spike_suspect:
            acceleration=(self.rate-previous_rate)/dt
            self.rate_accel=.3*acceleration+.7*self.rate_accel
        else:
            self.rate_accel=0.
        self.accepted_samples+=1
        self.last_valid_time=now-max(0.,age);self.valid=self.accepted_samples>=2;self.error='' if self.valid else 'imu_warming_up'
        self.history.append((stamp,self.unwrapped))
        return self.valid

    def yaw_at(self,stamp):
        if not self.history or stamp<self.history[0][0]-.05 or stamp>self.history[-1][0]+.05:
            return None
        return float(np.interp(stamp,[p[0] for p in self.history],[p[1] for p in self.history]))


def path_features(path):
    if not valid_path(path):
        raise ValueError('invalid_forward_path')
    heading,_=path_geometry(path)
    # A measured LiDAR prefix can begin beyond 1.5 m. It is still a valid
    # forward path; use its first supported samples instead of reducing an
    # empty near-window and clearing the controller's accepted trajectory.
    near_end=max(min(1.5,path[-1,0]),path[min(2,len(path)-1),0])
    near=path[:,0]<=near_end
    # Compare actual bending over 30 cm chords. A maximum of two numerical
    # gradients magnifies millimetre resampling/BEV noise into false jumps.
    # This feature only guards path acceptance; PP and speed curvature stay
    # unchanged. Short measured prefixes use their available geometric span.
    support=path[path[:,0]<=max(near_end,path[0,0]+.6)+1e-8]
    curvature=supported_curvature(support)
    return dict(heading=float(np.median(heading[near])),curvature=float(np.max(np.abs(curvature))),signed_curvature=float(np.median(curvature)),
                lateral=float(np.interp(1.5,path[:,0],path[:,1])))


class SteeringGuard:
    def __init__(self,config=None,normal_speed=7.,brake_decel=2.5):
        self.c=config or SteeringGuardConfig();self.c.validate()
        self.normal_speed=normal_speed;self.imu=ImuRotation(self.c)
        self.brake_decel=brake_decel
        self.camera=None;self.corner='STRAIGHT';self.corner_time=-math.inf
        self.reference=None;self.straight_yaw=None;self.was_corner=False
        self.last_features=None;self.last_path_time=-math.inf;self.last_path_stamp=-math.inf
        self.pending=None;self.pending_count=0;self.pending_stamp=-math.inf
        self.target_jump=self.heading_jump=self.curvature_jump=self.path_hold=False
        self.recovery=self.stop_latched=False;self.exit_count=0
        self.first_evaluation=None;self.last_eval_imu=-math.inf;self.abnormal_count=0
        self.last_result=None
        self.exit_reanchor=False
        self.braking_hold=False;self.motion_speed=0.;self.brake_exit_count=0
        self.gentle_frames=0
        self.corner_geometry_time=-math.inf
        self.recovery_context=None;self.recovery_limits=(1.2,8.);self.single_wall_reacquired=False
        self.severity='STRAIGHT';self.severity_time=-math.inf;self.severity_limits=(7.,8.)

    def reset_mission(self):
        self.reference=self.straight_yaw=None;self.was_corner=False
        self.last_features=None;self.last_path_time=-math.inf;self.last_path_stamp=-math.inf
        self.pending=None;self.pending_count=0;self.pending_stamp=-math.inf
        self.target_jump=self.heading_jump=self.curvature_jump=self.path_hold=False
        self.recovery=self.stop_latched=False;self.exit_count=self.abnormal_count=0
        self.exit_reanchor=False
        self.brake_exit_count=0
        self.gentle_frames=0
        # A new race does not make a physically moving car stationary.

    def camera_world_heading(self):
        if self.camera is None:return None
        yaw=self.imu.yaw_at(self.camera['stamp'])
        return angle(yaw+self.camera['heading']) if yaw is not None else None

    def exit_reference(self,now):
        if (self.exit_reanchor and not self.stop_latched and self.imu.fresh(now) and self.camera_fresh(now)
                and self.camera['curvature']<self.c.corner_current_curvature_threshold):
            heading=self.camera_world_heading()
            if heading is not None:
                self.reference=heading;self.straight_yaw=self.imu.unwrapped;self.exit_reanchor=False

    def set_corner(self,state,now):
        if (self.was_corner and state!='CORNER' and not self.recovery and not self.stop_latched and
                self.imu.fresh(now) and self.last_features is not None and self.last_features.get('world_heading') is not None):
            self.straight_yaw=self.imu.unwrapped;self.exit_reanchor=True
            self.was_corner=False
        self.corner=state;self.corner_time=now
        self.exit_reference(now)

    def camera_geometry(self,path,left,right,confidence,stamp,now):
        try:
            f=path_features(path)
            if not math.isfinite(confidence) or not self.c.camera_min_confidence<=confidence<=1:
                return
            if not all(valid_boundary(p) for p in (left,right)):
                return
            if self.camera is not None and stamp<=self.camera['stamp']:
                return
            f.update(confidence=confidence,stamp=stamp,time=now,
                     left_y=float(np.interp(1.5,left[:,0],left[:,1])),right_y=float(np.interp(1.5,right[:,0],right[:,1])),center_y=f['lateral'])
            self.camera=f
            if f['curvature']>=self.c.corner_current_curvature_threshold:
                self.corner_geometry_time=now
            self.exit_reference(now)
        except (ValueError,FloatingPointError):
            return

    def camera_fresh(self,now):
        return self.camera is not None and 0<=now-self.camera['time']<=self.c.camera_timeout_sec

    def corner_authorized(self,now):
        if not self.c.yaw_safety_enabled:
            return (self.severity!='STRAIGHT' and 0<=now-self.severity_time<=self.c.camera_timeout_sec and
                    self.camera_fresh(now) and self.camera['confidence']>=self.c.recovery_min_confidence)
        return (self.corner=='CORNER' and 0<=now-self.corner_time<=self.c.camera_timeout_sec and
                self.camera_fresh(now) and 0<=now-self.corner_geometry_time<=self.c.camera_timeout_sec)

    def accept_path(self,path,stamp,now,age=0.):
        f=path_features(path)
        if not self.c.enabled:
            self.last_features=f;self.last_path_time=now;self.last_path_stamp=stamp;return True
        if stamp<self.last_path_stamp:
            return False
        yaw=None
        if self.c.yaw_safety_enabled:
            yaw=self.imu.yaw_at(stamp)
            if yaw is None:yaw=self.imu.unwrapped
        f['world_heading']=angle(yaw+f['heading']) if yaw is not None else None
        old=self.last_features;corner=self.corner_authorized(now) and not self.recovery and self.c.yaw_safety_enabled
        comparable=old is not None and (now-self.last_path_time<=self.c.path_hold_sec or self.path_hold)
        dy=abs(f['lateral']-old['lateral']) if comparable else 0.
        dh=abs(math.degrees(angle((f['world_heading']-old['world_heading']) if
               f['world_heading'] is not None and old.get('world_heading') is not None else f['heading']-old['heading']))) if comparable else 0.
        dk=abs(f['curvature']-old['curvature']) if comparable else 0.
        self.target_jump=not corner and dy>self.c.target_jump_m
        self.heading_jump=not corner and dh>self.c.path_heading_jump_deg
        self.curvature_jump=not corner and dk>self.c.path_curvature_jump
        suspicious=self.target_jump or self.heading_jump or self.curvature_jump
        if suspicious:
            consistent=self.pending is not None and abs(f['lateral']-self.pending['lateral'])<.12 and abs(math.degrees(angle(f['heading']-self.pending['heading'])))<10.
            if stamp!=self.pending_stamp:
                self.pending_count=self.pending_count+1 if consistent else 1
                self.pending_stamp=stamp
            self.pending=f
            if self.pending_count<self.c.path_jump_confirm_frames:
                self.path_hold=True;self.exit_count=0;return False
        new_frame=stamp>self.last_path_stamp
        self.path_hold=False;self.pending=None;self.pending_count=0
        self.last_features=f
        if new_frame:
            self.last_path_time=now-max(0.,age);self.last_path_stamp=stamp
            if not self.c.yaw_safety_enabled and (self.camera_fresh(now) or self.recovery_context is not None):
                stable=(dy<.12 and dh<self.c.recovery_heading_deg and (self.recovery_context is not None or (self.camera is not None and (self.camera['confidence']>=self.c.recovery_min_confidence or self.single_wall_reacquired and self.camera['confidence']>=.60))))
                self.exit_count=self.exit_count+1 if stable else 0
                if self.motion_speed<=self.c.recovery_speed_mps and self.exit_count>=self.c.recovery_exit_confirm_frames:
                    self.braking_hold=self.recovery=self.stop_latched=False
            if self.c.yaw_safety_enabled and self.imu.fresh(now) and f['world_heading'] is not None and self.camera_fresh(now):
                if self.reference is None:
                    heading=self.camera_world_heading()
                    self.reference=f['world_heading'] if heading is None else heading
                    self.straight_yaw=self.imu.unwrapped
                elif not self.recovery and not self.stop_latched:
                    if corner:
                        heading=self.camera_world_heading()
                        heading=f['world_heading'] if heading is None else heading
                        change=angle(heading-self.reference)
                        if not self.was_corner:
                            # Existing Camera corner confirmation grants a new
                            # course; a genuine sharp bend is not a straight
                            # heading outlier. Recovery cannot use this branch.
                            self.reference=heading
                        else:
                            self.reference=angle(self.reference+.3*change)
                    elif self.was_corner:
                        heading=self.camera_world_heading()
                        self.reference=f['world_heading'] if heading is None else heading
                        self.straight_yaw=self.imu.unwrapped
                    else:
                        # Freeze a straight course: repeatedly adding current
                        # yaw to a bad flat path must not make its reference
                        # co-rotate with a slow spin. Only measured Camera
                        # curvature can advance the course on a gentle bend.
                        k=self.camera['signed_curvature']
                        gentle=(abs(k)>=self.c.gentle_curve_min_curvature and
                                self.camera['curvature']<self.c.corner_current_curvature_threshold and
                                k*self.imu.rate>=0 and abs(self.imu.rate)<self.c.straight_max_yaw_rate_deg_s)
                        self.gentle_frames=self.gentle_frames+1 if gentle else 0
                        heading=self.camera_world_heading()
                        if (self.gentle_frames>=self.c.gentle_curve_confirm_frames and heading is not None and
                                abs(math.degrees(angle(heading-self.reference)))<self.c.path_heading_jump_deg):
                            change=.3*angle(heading-self.reference)
                            self.reference=angle(self.reference+change)
                            self.straight_yaw+=change
                if not self.recovery and not self.stop_latched:self.was_corner=corner
                error=abs(math.degrees(angle(self.reference-self.imu.yaw)))
                braking_stable=(dy<.12 and dh<self.c.recovery_heading_deg and
                    abs(self.imu.rate)<=self.c.recovery_yaw_rate_deg_s and self.camera['confidence']>=self.c.recovery_min_confidence and
                    self.motion_speed<=self.c.recovery_speed_mps)
                self.brake_exit_count=self.brake_exit_count+1 if braking_stable else 0
                if self.braking_hold and self.brake_exit_count>=self.c.recovery_exit_confirm_frames:
                    # Allow PP's bounded correction at <=2 m/s once braking is
                    # stable; requiring aligned heading while commanding zero
                    # speed would deadlock a car that cannot turn in place.
                    self.braking_hold=False;self.recovery=True;self.exit_count=0
                stable=(dy<.12 and dh<self.c.recovery_heading_deg and error<=self.c.recovery_heading_deg and
                        abs(self.imu.rate)<=self.c.recovery_yaw_rate_deg_s and self.camera['confidence']>=self.c.recovery_min_confidence)
                self.exit_count=self.exit_count+1 if stable else 0
                if self.exit_count>=self.c.recovery_exit_confirm_frames and (not self.braking_hold or self.motion_speed<=self.c.recovery_speed_mps):
                    self.recovery=self.stop_latched=False
                    self.braking_hold=False
                    self.straight_yaw=self.imu.unwrapped
        return True

    def evaluate(self,raw,target,speed,current_steering,now,dt,motion_speed=None):
        self.motion_speed=speed if motion_speed is None else motion_speed
        if self.c.enabled and not self.c.yaw_safety_enabled:
            return self.evaluate_camera(raw,target,now)
        if self.first_evaluation is None:self.first_evaluation=now
        corner=self.corner_authorized(now)
        limit_deg=self.c.corner_max_steering_deg if corner else self.c.straight_max_steering_deg
        state='CORNER' if corner else 'STRAIGHT';reason='';cap=self.normal_speed;stop=False
        imu_fresh=self.imu.fresh(now)
        error=math.degrees(angle(self.reference-self.imu.yaw)) if self.reference is not None and self.imu.yaw is not None else 0.
        rate=self.imu.rate or 0.
        # The old corner look-ahead heading can be >45 degrees. Until the
        # first fresh exit geometry arrives, do not call that old heading a
        # reverse turn. Steering authority is already the straight 8 degrees.
        warning=self.c.corner_heading_warning_deg if corner or self.exit_reanchor else self.c.straight_heading_warning_deg
        heading_stop=self.c.corner_heading_stop_deg if corner or self.exit_reanchor else self.c.straight_heading_stop_deg
        max_rate=self.c.corner_max_yaw_rate_deg_s if corner else self.c.straight_max_yaw_rate_deg_s
        path_error=math.degrees(angle(self.last_features['world_heading']-self.imu.yaw)) if self.last_features and self.last_features.get('world_heading') is not None and self.imu.yaw is not None else error
        wrong=abs(path_error)>warning and math.degrees(raw)*path_error<0
        growth=rate*error<0
        intended_turn=abs(path_error)>=self.c.recovery_heading_deg and path_error*raw>0
        same_rotation=(raw*rate>0 or current_steering*rate>0)
        high_rate=abs(rate)>max_rate
        # A fixed yaw-rate threshold alone reacts late at 7 m/s. Forecast
        # filtered yaw-rate over the actuator response window, using actual
        # source time and only accepted IMU samples. This limits commands; it
        # never creates an IMU steering correction.
        predicted_rate=rate+self.imu.rate_accel*self.c.yaw_rate_prediction_sec
        predicted_rate=math.copysign(max(abs(rate),abs(predicted_rate)),rate) if rate*self.imu.rate_accel>0 else rate
        observed_lateral=self.motion_speed*abs(math.radians(predicted_rate))
        high_lateral=imu_fresh and not corner and observed_lateral>self.c.max_observed_lateral_accel_mps2
        moving_yaw=abs(rate)>self.c.heading_growth_min_yaw_rate_deg_s
        abnormal=imu_fresh and (high_lateral or high_rate or (abs(error)>warning and growth and same_rotation and moving_yaw and not intended_turn) or (wrong and moving_yaw))
        if self.imu.last_stamp!=self.last_eval_imu:
            self.abnormal_count=self.abnormal_count+1 if abnormal else 0
            self.last_eval_imu=self.imu.last_stamp
        overrotation=abnormal and self.abnormal_count>=2
        accumulated=abs(math.degrees(self.imu.unwrapped-self.straight_yaw)) if self.imu.unwrapped is not None and self.straight_yaw is not None else 0.
        reverse=imu_fresh and (abs(error)>=heading_stop or (not corner and accumulated>=self.c.straight_heading_stop_deg))
        if self.c.enabled:
            if overrotation:
                self.recovery=True;self.exit_count=0;reason='YAW_LATERAL_ACCELERATION' if high_lateral else ('STRAIGHT_OVERROTATION' if not corner else 'CORNER_OVERROTATION')
            if reverse:
                self.stop_latched=True;self.recovery=True;self.exit_count=0;reason='REVERSE_RISK'
            if self.recovery or self.stop_latched:
                state='RECOVERY';cap=self.c.recovery_speed_mps;limit_deg=self.c.recovery_max_steering_deg
            if self.path_hold:
                cap=min(cap,self.c.recovery_speed_mps);state='PATH_HOLD';reason=reason or ('PATH_HEADING_JUMP' if self.heading_jump else 'TARGET_JUMP')
                if now-self.last_path_time>self.c.path_hold_sec:stop=True;reason='NO_VALID_FORWARD_TRAJECTORY'
            if not imu_fresh:
                state='IMU_STALE';cap=self.c.recovery_speed_mps;limit_deg=self.c.recovery_max_steering_deg;reason=self.imu.error or 'imu_timeout'
                age=now-self.imu.last_valid_time if math.isfinite(self.imu.last_valid_time) else now-self.first_evaluation
                if age>self.c.imu_stop_timeout_sec:stop=True
            if not self.camera_fresh(now):
                cap=min(cap,self.c.recovery_speed_mps);reason=reason or 'CAMERA_GEOMETRY_UNAVAILABLE'
            if self.braking_hold:
                stop=True;state='RECOVERY';reason='BRAKING_STABILITY_HOLD'
            # At high speed, recovery braking takes time. Stop before a growing
            # heading error can consume the remaining safe angular margin.
            brake_time=max(0.,speed-self.c.recovery_speed_mps)/self.brake_decel
            if overrotation and growth and abs(error)+abs(rate)*brake_time>=heading_stop:
                stop=True;reason='PREDICTED_OVERROTATION';self.stop_latched=True
            if self.stop_latched:stop=True;state='STOP';reason=reason or 'REVERSE_RISK_LATCHED'
            if target is not None and (not np.isfinite(target).all() or target[0]<=0):
                stop=True;reason='INVALID_FORWARD_TARGET'
            bounded=float(np.clip(raw,-math.radians(limit_deg),math.radians(limit_deg)))
            opposes_course=abs(error)>warning and raw*error<0 and not intended_turn
            same_direction_relax=(raw*rate>0 and (high_lateral or high_rate or
                (moving_yaw and (wrong or (growth and abs(error)>warning and not intended_turn)))))
            if self.recovery and imu_fresh and (opposes_course or same_direction_relax):
                magnitude=max(0.,min(abs(current_steering),abs(bounded))-math.radians(self.c.steering_relaxation_deg_s)*dt)
                bounded=math.copysign(magnitude,bounded)
            if stop:cap=0.;bounded=0.
        else:
            bounded=raw;state='DISABLED'
        diagnostic=dict(state=state,reason=reason,raw_pp_steering_deg=math.degrees(raw),state_steering_limit_deg=limit_deg,
            final_guarded_steering_deg=math.degrees(bounded),imu_yaw_deg=math.degrees(self.imu.yaw) if self.imu.yaw is not None else None,
            imu_yaw_rate_raw_deg_s=self.imu.raw_rate,imu_yaw_rate_filtered_deg_s=self.imu.rate,
            path_heading_deg=math.degrees(self.last_features['world_heading']) if self.last_features and self.last_features.get('world_heading') is not None else None,
            path_heading_body_deg=math.degrees(self.last_features['heading']) if self.last_features else None,heading_error_deg=error,
            straight_reference_yaw_deg=math.degrees(angle(self.straight_yaw)) if self.straight_yaw is not None else None,
            reference_world_heading_deg=math.degrees(self.reference) if self.reference is not None else None,
            target_x=float(target[0]) if target is not None else None,target_y=float(target[1]) if target is not None else None,
            target_jump=self.target_jump,path_heading_jump=self.heading_jump,wrong_direction_steering=wrong,
            rejected_target_y=self.pending['lateral'] if self.pending is not None else None,
            straight_overrotation=overrotation and not corner,reverse_risk=reverse,speed_limit=cap,stop=stop,
            predicted_yaw_rate_deg_s=predicted_rate,observed_lateral_accel_mps2=observed_lateral,
            imu_fresh=imu_fresh,imu_spike_suspect=self.imu.spike_suspect,recovery_exit_count=self.exit_count,
            braking_stability_hold=self.braking_hold,motion_speed_bound=self.motion_speed,
            braking_exit_count=self.brake_exit_count,
            awaiting_corner_exit_geometry=self.exit_reanchor,
            current_path_heading_error_deg=path_error,
            camera_confidence=self.camera['confidence'] if self.camera else 0.,
            left_wall_y=self.camera['left_y'] if self.camera else None,right_wall_y=self.camera['right_y'] if self.camera else None,
            center_y=self.camera['center_y'] if self.camera else None)
        self.last_result=diagnostic
        return bounded,cap,stop,diagnostic


    def set_severity(self,severity,speed_limit,steering_limit,now):
        self.severity=severity;self.severity_limits=(speed_limit,steering_limit);self.severity_time=now

    def evaluate_camera(self,raw,target,now):
        fresh_policy=0<=now-self.severity_time<=self.c.camera_timeout_sec
        fresh_camera=self.camera_fresh(now)
        confidence=self.camera['confidence'] if fresh_camera else 0.
        valid=fresh_policy and fresh_camera and (confidence>=self.c.recovery_min_confidence or self.single_wall_reacquired and confidence>=.60)
        held=self.recovery_context is not None
        cap,limit=self.recovery_limits if held else self.severity_limits if valid else (self.c.recovery_speed_mps,self.c.straight_max_steering_deg)
        state='CORNER_PATH_HOLD' if held else self.severity if valid else 'CAMERA_RECOVERY'
        stop=False;reason='' if valid or held else 'CAMERA_POLICY_UNAVAILABLE_OR_LOW_CONFIDENCE'
        if self.path_hold:
            state='PATH_HOLD';cap=min(cap,self.c.recovery_speed_mps);limit=min(limit,self.c.recovery_max_steering_deg)
            reason='PATH_HEADING_JUMP' if self.heading_jump else 'TARGET_JUMP' if self.target_jump else 'PATH_CURVATURE_JUMP'
            if now-self.last_path_time>self.c.path_hold_sec:stop=True;reason='NO_VALID_FORWARD_TRAJECTORY'
        # This is a Camera forward-direction safety check, not an IMU yaw target.
        reverse=bool(self.last_features and abs(math.degrees(self.last_features['heading']))>=80.)
        if reverse:stop=True;reason='CAMERA_REVERSE_RISK'
        if self.braking_hold:stop=True;reason='BRAKING_STABILITY_HOLD';state='CAMERA_RECOVERY'
        if target is not None and (not np.isfinite(target).all() or target[0]<=0):stop=True;reason='INVALID_FORWARD_TARGET'
        bounded=float(np.clip(raw,-math.radians(limit),math.radians(limit)))
        if stop:cap=0.;bounded=0.;state='STOP'
        diagnostic=dict(state=state,reason=reason,corner_severity=self.recovery_context['severity'] if held else self.severity,yaw_safety_enabled=False,using_previous_trajectory=held,
            raw_pp_steering_deg=math.degrees(raw),state_steering_limit_deg=limit,corner_steering_limit_deg=limit,
            final_guarded_steering_deg=math.degrees(bounded),speed_limit=cap,stop=stop,
            target_x=float(target[0]) if target is not None else None,target_y=float(target[1]) if target is not None else None,
            target_jump=self.target_jump,path_heading_jump=self.heading_jump,
            path_curvature_jump=self.curvature_jump,
            path_curvature=self.last_features['curvature'] if self.last_features else None,
            pending_path_curvature=self.pending['curvature'] if self.pending else None,
            path_heading_body_deg=math.degrees(self.last_features['heading']) if self.last_features else None,
            reverse_risk=reverse,camera_confidence=confidence,corner_policy_fresh=fresh_policy,
            braking_stability_hold=self.braking_hold,motion_speed_bound=self.motion_speed,recovery_exit_count=self.exit_count,
            left_wall_y=self.camera['left_y'] if self.camera else None,right_wall_y=self.camera['right_y'] if self.camera else None,
            center_y=self.camera['center_y'] if self.camera else None)
        self.last_result=diagnostic
        return bounded,cap,stop,diagnostic
