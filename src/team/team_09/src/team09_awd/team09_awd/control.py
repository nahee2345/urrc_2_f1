from .config import G, SETTINGS
"""One steering controller and one speed governor, based on v6."""
import math
import numpy as np
from .core import speed_limit,steering_rate,reachable_curvature

def steer_step(previous,desired,dt,speed=0.):
    dt=max(0.,min(.15,dt))
    goal=math.atan(G.wheelbase*reachable_curvature(desired,speed))
    angle=math.atan(G.wheelbase*previous)
    angle+=float(np.clip(goal-angle,-steering_rate(speed)*dt,steering_rate(speed)*dt))
    return float(np.clip(math.tan(angle)/G.wheelbase,-G.max_curvature,G.max_curvature))

def govern_speed(r,command,k,speed,rate,dt,maximum,lateral,brake,accel,latency,reason,imu_used=True):
    if reason=='RECOVER_REVERSE':return max(-.20,min(0.,command)-accel*dt),'CONTACT_REVERSE'
    if reason not in ('RUN','RECOVER_CRAWL'):return 0.,reason
    if getattr(r, "road_guided", False):
        # Geometry of the road, not the controller's short alignment correction.
        limits = {
            "FREE_BRAKE": speed_limit(0., free=r.free, confidence=r.confidence,
                                      maximum=maximum, lateral=lateral, brake=brake, latency=latency),
            "ROAD_PREVIEW": r.preview_limit,
        }
    else:
        limits = {
            "CURVATURE_BRAKE": speed_limit(r.curvature, r.free, r.confidence, maximum, lateral, brake, latency),
            "PREVIEW": getattr(r, "preview_limit", float("inf")),
            "SIDE_APPROACH": getattr(r, "proximity_limit", .45),
            "CURRENT_STEER": math.sqrt(lateral/max(abs(k), .001)),
        }
    if r.mode=='RECOVER_CRAWL':limits['RECOVER']=min(.35,speed_limit(r.curvature,r.free,r.confidence,.35,lateral,brake,latency,reserve=.15))
    if getattr(r,'route_kind','REACTIVE') in ('REACTIVE','ROAD_FALLBACK'):limits['REACTIVE']=SETTINGS['control']['reactive_speed_mps']
    if not imu_used:limits['IMU_MISSING']=10.
    if abs(rate-speed*k)>SETTINGS['control']['yaw_residual_limit_radps']:limits['YAW_RESPONSE']=SETTINGS['control']['yaw_residual_speed_mps']
    if abs(rate*speed)>lateral*1.2:limits['MEASURED_LATERAL']=speed*.85
    limiter=min(limits,key=limits.get);target=max(0.,float(limits[limiter]))
    command+=float(np.clip(target-command,-brake*dt,accel*dt))
    # Preserve v6 acceleration/deceleration with its measured overspeed correction.
    if r.mode=='RECOVER_CRAWL' or speed>target+.5:command=min(command,target)
    return max(0.,command),limiter

class CommandWatchdog:
    """No geometric replanning; latch start once per explicit enable session."""
    def __init__(self,timeout=.35):
        self.enabled=False;self.started=False;self.at=None;self.v=0.;self.w=0.;self.timeout=timeout
    def enable(self,value):
        self.enabled=bool(value)
        if not value:self.started=False;self.at=None
    def signal(self,value):
        if value and self.enabled:self.started=True
    def command(self,v,w,now):
        if not all(math.isfinite(x) for x in (v,w)) or v<-.25:
            self.at=None;return
        self.v=v;self.w=w;self.at=now
    def output(self,now):
        if not self.enabled:return 0.,0.,'DISABLED'
        if not self.started:return 0.,0.,'WAIT_START_LIGHTS'
        if self.at is None or not 0<=now-self.at<=self.timeout:return 0.,0.,'COMMAND_TIMEOUT'
        return self.v,self.w,'PASS'
