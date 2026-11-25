"""Immediate safety decreases; increases require distinct accepted observations."""
import math

class SpeedIncreaseConfirmation:
    def __init__(self,frames=3):
        self.frames=frames;self.limit=None;self.pending=[];self.key=None;self.reason='STARTUP'
    def reset(self):self.limit=None;self.pending=[];self.key=None;self.reason='STARTUP'
    def update(self,speed,key,reason,enabled=True,*,verified_decrease=True):
        if not enabled:self.reset();return speed,False
        if not math.isfinite(speed) or speed<0:raise ValueError('invalid speed authority')
        if self.limit is not None and speed<self.limit-1e-6 and not verified_decrease:
            # Apply the candidate request this cycle, without latching an
            # unconfirmed one-frame hint as a persistent safety ceiling.
            self.pending=[];self.key=key
            return speed,False
        if self.limit is None or speed<self.limit-1e-6:
            self.limit=speed;self.pending=[];self.reason=reason;self.key=key
        elif speed>self.limit+1e-6:
            if key!=self.key:
                self.key=key;self.pending.append(speed);self.pending=self.pending[-self.frames:]
                if len(self.pending)>=self.frames:
                    self.limit=min(self.pending);self.reason=reason
            # Timer callbacks never count as independent sensor observations.
        else:self.pending=[];self.key=key
        return min(speed,self.limit),self.limit<speed-1e-6

VERIFIED_LIMITERS=frozenset(('DEPTH_CORNER','DEPTH_SUPPORTED_RANGE','DEPTH_UNAVAILABLE','SUPPORTED_PREVIEW',
    'CORNER_PREVIEW','CORNER_SEVERITY','CAMERA_BRAKING_PREVIEW','PATH_CONFIDENCE','PATH_CURVATURE',
    'PATH_SOURCE','VISIBLE_PATH_LENGTH','DEPTH_PREVIEW_HOLD','RAMP','RAMP_SPEED_LIMIT','TTC','OBSTACLE','SPEED_INCREASE_CONFIRMATION'))

def safety_decrease_priority(current,target,requested,guard_cap,steering_limit,*,command_fresh,limiter):
    if target>=current-1e-6:return False
    local=(guard_cap<requested-1e-6 or steering_limit is not None and steering_limit<min(guard_cap,requested)-1e-6)
    return bool(local or command_fresh and limiter in VERIFIED_LIMITERS)

class PrimaryLidarAdmission:
    """Hysteresis only for stability evidence; hard validity/confidence gates bypass it."""
    def __init__(self,frames=3):self.frames=frames;self.good=0;self.bad=0;self.confirmed=False;self.key=None
    def update(self,key,hard_valid,stable):
        if not hard_valid:
            self.good=self.bad=0;self.confirmed=False;self.key=key;return False
        if key==self.key:return self.confirmed
        self.key=key
        if stable:
            self.good+=1;self.bad=0
            if self.good>=self.frames:self.confirmed=True
        else:
            self.good=0;self.bad+=1
            if self.bad>=2:self.confirmed=False
        return self.confirmed


class ObservedPreviewHold:
    """Finite observed space lease; never renews a sensor timestamp or steering path."""
    def __init__(self,hold_sec=.5):
        if not 0<hold_sec<=.5:raise ValueError('preview hold exceeds existing recovery window')
        self.hold_sec=hold_sec;self.memory=None;self.ceiling=None
    def clear(self):self.memory=None;self.ceiling=None
    def record(self,stamp,source_time,travel,epoch,supported):
        if travel is None or not all(math.isfinite(v) for v in (stamp,source_time,travel,supported)) or supported<=0:return
        self.memory=dict(stamp=stamp,time=source_time,travel=travel,epoch=epoch,supported=supported);self.ceiling=None
    def evaluate(self,now,travel,epoch,*,odom_fresh,lidar_straight,command,decel,margin):
        m=self.memory
        if (m is None or not odom_fresh or not lidar_straight or epoch!=m['epoch'] or
            not 0<=now-m['time']<=self.hold_sec or command<=0):return None
        remaining=m['supported']-max(0.,travel-m['travel'])
        if remaining<=margin:return None
        if self.ceiling is None:self.ceiling=command
        # A stale preview cannot authorize acceleration. Its braking envelope
        # is measured-space stopping distance, not an extrapolated road shape.
        limit=min(self.ceiling,math.sqrt(2*decel*max(0.,remaining-margin)))
        return dict(limit=limit,remaining_m=remaining,source_stamp=m['stamp'],age=now-m['time'])
