"""Rate/acceleration-limited steering with damped arrival at target angle."""
import math


class SteeringLimiter:
    def __init__(self,cfg):
        self.cfg=cfg
        self.angle=0.0
        self.rate=0.0
        self.filtered_target=0.0

    def reset(self):
        self.angle=0.0
        self.rate=0.0
        self.filtered_target=0.0

    def apply(self,target,dt,speed):
        dt=max(0.,min(.20,float(dt)))
        full=math.radians(float(self.cfg['max_center_steer_deg']))
        target=max(-full,min(full,float(target)))
        tau=max(0.,float(self.cfg.get('steer_target_filter_tau_s',.10)))
        alpha=dt/(tau+dt) if tau else 1.
        self.filtered_target+=alpha*(target-self.filtered_target)
        target=self.filtered_target
        error=target-self.angle
        maxrate=math.radians(float(self.cfg.get('max_steer_rate_deg_s',35.)))
        maxacc=math.radians(float(self.cfg.get('max_steer_accel_deg_s2',170.)))
        # Brake steering rate before target; a plain acceleration clamp can
        # overshoot and command repeated left-right counter-steering.
        allowed_rate=min(maxrate,math.sqrt(max(0.,2.*maxacc*abs(error))))
        requested=math.copysign(allowed_rate,error) if abs(error)>1e-8 else 0.
        step=maxacc*dt
        self.rate=max(self.rate-step,min(self.rate+step,requested))
        after=self.angle+self.rate*dt
        if (target-self.angle)*(target-after)<=0.:
            self.angle=target
            self.rate=0.
        else:
            self.angle=max(-full,min(full,after))
        if (abs(target-self.angle)<math.radians(.03) and
                abs(self.rate)<math.radians(.5)):
            self.angle=target
            self.rate=0.
        return self.angle
