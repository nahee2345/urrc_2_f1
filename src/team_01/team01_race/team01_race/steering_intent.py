"""Reject unsupported rapid L-R-L steering intent, not smooth same-side turns."""
import math


class SteeringIntent:
    def __init__(self,cfg):
        self.confirm_frames=int(cfg.get('opposite_corner_confirm_frames',3))
        self.threshold=math.radians(float(cfg.get('steering_flip_threshold_deg',4.)))
        self.committed=0
        self.opposite_frames=0
        self.straight_time=0.
        self.phase='STRAIGHT'
        self.last_approved=0.

    def reset(self):
        self.committed=0
        self.opposite_frames=0
        self.straight_time=0.
        self.phase='STRAIGHT'
        self.last_approved=0.

    def update(self,target,road_heading_deg,curvature,dt):
        target=float(target);dt=max(0.,float(dt))
        if not all(math.isfinite(v) for v in (target,road_heading_deg,curvature,dt)):
            self.phase='INVALID_STEERING_INTENT'
            self.last_approved=0.
            return 0.
        sign=1 if target>self.threshold else -1 if target<-self.threshold else 0
        head=float(road_heading_deg)
        straight=abs(head)<3. and abs(float(curvature))<.025
        if straight:self.straight_time+=dt
        else:self.straight_time=0.
        # Preserve turn-side memory across long straights. Otherwise a
        # spurious -15 degree target after +15 -> 0 bypasses this guard.
        # A genuine opposing bend releases the guard using signed road evidence.
        if self.committed and sign and sign!=self.committed:
            if sign*head>=7.:
                self.opposite_frames+=1
            else:
                self.opposite_frames=0
            if self.opposite_frames<self.confirm_frames:
                self.phase='REVERSAL_HELD'
                self.last_approved=0.
                return 0.
            self.committed=sign
            self.opposite_frames=0
            self.phase='OPPOSITE_CORNER_CONFIRMED'
        else:
            self.opposite_frames=0
            if sign:self.committed=sign
            self.phase=('LEFT_CORNER' if sign>0 else
                        'RIGHT_CORNER' if sign<0 else 'CENTERING')
        # A steadily increasing turn (0,5,10,15 deg) passes unchanged.
        # The downstream physical steering rate limiter still applies.
        self.last_approved=target
        return target
