"""Fail-closed IMU/steering-sign and response-delay calibration gate."""
import math


class FeedbackGuard:
    def __init__(self,cfg):
        self.approved=cfg.get('qa_approved_imu_joint_calibration',False) is True
        self.good=0
        self.status='QA_CALIBRATION_REQUIRED'
        self.max_age=float(cfg.get('imu_joint_max_sample_age_s',.25))
        self.command=0.
        self.command_t=None
        self.transition_t=None
        self.transition_target=None
        self.response_delay=None
        self.sign_match=None

    def command_sent(self,steer,t):
        steer=float(steer)
        if self.command_t is not None and abs(steer-self.command)>=math.radians(4.):
            if self.transition_t is None:
                self.transition_t=float(t)
                self.transition_target=steer
        self.command=steer
        self.command_t=float(t)

    def observe(self,sim_now,joint,yaw,speed,joint_stamp,imu_stamp):
        self.good=0 if self.command_t is None else self.good
        if (joint is not None and self.transition_t is not None and
                joint_stamp>=self.transition_t and
                abs(float(joint)-self.transition_target)<math.radians(2.)):
            self.response_delay=max(0.,joint_stamp-self.transition_t)
            self.transition_t=self.transition_target=None
        if self.transition_t is not None and sim_now-self.transition_t>.5:
            self.response_delay=max(0.,sim_now-self.transition_t)
            self.transition_t=self.transition_target=None
            self.good=0
            self.status='STEERING_RESPONSE_TIMEOUT'
            return False
        ages=(sim_now-joint_stamp,sim_now-imu_stamp)
        if (joint is None or yaw is None or self.command_t is None or
                not all(math.isfinite(float(x)) for x in (joint,yaw)) or
                not all(0.<=a<=self.max_age for a in ages)):
            self.good=0;self.status='SENSOR_MISSING_OR_STALE';return False
        if abs(self.command)<math.radians(4.) or speed<.30:
            self.good=0;self.status='CALIBRATION_NOT_EXCITED';return False
        if abs(joint)<math.radians(1.) or abs(yaw)<.025:
            self.good=0;self.status='ACTUATOR_RESPONSE_NOT_OBSERVED';return False
        self.sign_match=(self.command*joint>0 and self.command*yaw>0)
        if not self.sign_match:
            self.good=0;self.status='STEERING_OR_YAW_SIGN_MISMATCH';return False
        self.good+=1
        if self.response_delay is not None and self.response_delay>.5:
            self.good=0;self.status='STEERING_RESPONSE_DELAY';return False
        if not self.approved:
            self.status='QA_CALIBRATION_REQUIRED';return False
        self.status='VERIFIED' if self.good>=4 else 'VERIFYING_DIRECTION'
        return self.good>=4

    def diag(self):
        return {'approved':self.approved,'status':self.status,
                'matching_frames':self.good,'sign_match':self.sign_match,
                'observed_response_delay_s':self.response_delay}
