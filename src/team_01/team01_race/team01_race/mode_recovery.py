"""Fail-closed speed-recovery hysteresis and classified stop diagnostics."""
from collections import Counter
from team01_race.planning_gate import Motion


class RecoveryGuard:
    def __init__(self,cfg):
        self.cooldown=float(cfg.get('motion_restart_hold_s',.12))
        self.remaining=0.0
        self.reasons=Counter()
        self.transitions=0
        self.previous=None
        self.last_fault=None
        self.active=False

    def update(self,motion,dt):
        dt=max(.005,min(.25,float(dt)))
        if motion.mode!=self.previous:
            self.transitions+=1
            self.previous=motion.mode
        if motion.mode in ('STOP','BRAKE'):
            self.reasons[motion.reason]+=1
            self.remaining=self.cooldown
            self.last_fault=motion.reason
            self.active=False
            # Safety decisions always override comfort smoothing.
            return motion
        if self.remaining>0.:
            self.remaining=max(0.,self.remaining-dt)
            self.active=True
            return Motion(0.,motion.steering,'RECOVER',
                          'stable restart after '+str(self.last_fault))
        self.active=False
        return motion


class CaptureMode:
    """Schmitt trigger prevents 3.9/4.1 degree straight/corner chatter."""
    def __init__(self):
        self.straight=True

    def update(self,heading_deg,curvature,hazard=False):
        if hazard:
            self.straight=False
        elif self.straight:
            if abs(float(heading_deg))>5.5 or curvature>.045:
                self.straight=False
        elif abs(float(heading_deg))<3.0 and curvature<.025:
            self.straight=True
        return self.straight
