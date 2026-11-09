"""Steering wheel joint position is the actuator feedback; IMU measures yaw."""
import math


class SteeringJointFeedback:
    def __init__(self,cfg):
        self.gain=float(cfg.get('steering_joint_feedback_gain',.35))
        self.max_correction=math.radians(float(cfg.get('steering_joint_feedback_max_deg',3.)))
        self.error=None
        self.source='OPEN_LOOP'

    def correct(self,target,actual,speed):
        self.error=None
        self.source='OPEN_LOOP'
        if actual is None or not math.isfinite(float(actual)) or speed<.15:
            return float(target)
        error=float(target)-float(actual)
        self.error=error
        # Joint feedback is actual angle; no integral windup or guessed yaw.
        correction=max(-self.max_correction,min(self.max_correction,self.gain*error))
        self.source='JOINT_CLOSED_LOOP'
        return float(target)+correction
