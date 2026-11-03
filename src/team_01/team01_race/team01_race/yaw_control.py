"""Measured yaw-rate correction; omits stale, invalid, or near-zero odometry."""
import math


class YawFeedback:
    def __init__(self,cfg):
        self.cfg=cfg
        self.error=0.
        self.active=False

    def correct(self,angle,speed,yaw_rate):
        self.active=False
        self.error=0.
        if yaw_rate is None or not math.isfinite(float(yaw_rate)):
            return float(angle)
        speed=max(0.,float(speed))
        if not math.isfinite(speed) or speed<.25:
            return float(angle)
        wb=max(.1,float(self.cfg['wheel_base_m']))
        desired_rate=speed*math.tan(float(angle))/wb
        self.error=desired_rate-float(yaw_rate)
        limit=math.radians(float(self.cfg.get('yaw_feedback_max_correction_deg',4.)))
        gain=float(self.cfg.get('yaw_feedback_gain',.065))
        correction=max(-limit,min(limit,gain*self.error))
        cap=math.radians(float(self.cfg['max_center_steer_deg']))
        self.active=True
        return max(-cap,min(cap,float(angle)+correction))
