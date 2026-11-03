"""Rate/acceleration-limited steering, independent of route selection."""
import math


class SteeringLimiter:
    def __init__(self,cfg):
        self.cfg=cfg
        self.angle=0.0
        self.rate=0.0

    def reset(self):
        self.angle=0.0
        self.rate=0.0

    def apply(self,target,dt,speed):
        dt=max(.005,min(.20,float(dt)))
        angle_limit=math.radians(float(self.cfg['max_center_steer_deg']))
        target=max(-angle_limit,min(angle_limit,float(target)))
        maxrate=math.radians(float(self.cfg.get('max_steer_rate_deg_s',35.)))
        maxacc=math.radians(float(self.cfg.get('max_steer_accel_deg_s2',170.)))
        # Smooth rate ramp, not repeated interpolation of noisy targets.
        requested=max(-maxrate,min(maxrate,(target-self.angle)/dt))
        dv=maxacc*dt
        self.rate=max(self.rate-dv,min(self.rate+dv,requested))
        self.angle=max(-angle_limit,min(angle_limit,self.angle+self.rate*dt))
        if abs(target-self.angle)<math.radians(.05) and abs(self.rate)<math.radians(1.):
            self.angle=target
            self.rate=0.
        return self.angle
