"""Restored pre-reset TEAM11 smooth coordinate-route follower for Ackermann tests.

Based on the earlier team11_race/planner.py and three_lane/BoundedTracker.hh.
Keep the CURRENT numbering: 1=right, 2=center, 3=left.
Old SetWorldPoseCmd kinematic demonstration is NOT used here: the output is a
bounded steering command for the real Ackermann vehicle in Gazebo.
"""
import math
from dataclasses import dataclass

WHEELBASE=.36
MAX_STEER=math.pi/6
MAX_STEER_RATE=1.57
MAX_YAW_RATE=1.2
MAX_LATERAL_A=2.5
LANE_SHIFT=18.0

def clamp(x,lo,hi):
    return max(lo,min(hi,x))

def wrap(x):
    return math.atan2(math.sin(x),math.cos(x))

def smoothstep5(t):
    x=clamp(t,0.0,1.0)
    return x*x*x*(10.0+x*(-15.0+6.0*x))

@dataclass
class Guidance:
    steer: float
    target_lane: int
    reference_x: float
    reference_y: float
    changing: bool
    progress: float
    yaw_rate_limit: float

class RestoredFollower:
    def __init__(self,track,initial_lane,wheelbase=WHEELBASE):
        if initial_lane not in (1,2,3):
            raise ValueError("initial_lane must be 1..3")
        self.track=track
        self.lane=initial_lane
        self.target=initial_lane
        self.wheelbase=wheelbase
        self.changing=False
        self.change_distance=0.0
        self.previous_index=None
        self.previous_steer=0.0
        self.last_time=None
        # Initial spawn is at a painted grid, not necessarily on lane line.
        # The old follower used an 18m smoothstep for lateral convergence.
        self.initial_reference=None
        self.initial_distance=0.0
        self.initial_join=True

    def point(self,lane,index):
        p=self.track.lanes[lane][index]
        return p["x_m"],p["y_m"]

    def guidance(self,x,y,yaw,speed,now,selected_lane):
        if selected_lane not in (1,2,3):
            raise ValueError("invalid route")
        if not all(math.isfinite(v) for v in (x,y,yaw,speed,now)):
            raise ValueError("nonfinite pose")
        dt=.05 if self.last_time is None else clamp(now-self.last_time,.01,.2)
        if self.last_time is not None and now<=self.last_time:
            raise ValueError("stale time")
        self.last_time=now
        idx=self.track.nearest(x,y,self.previous_index)
        distance=0.0
        if self.previous_index is not None:
            d=self.track.along(self.previous_index,idx)
            if d<self.track.length*.5:
                distance=d
        self.previous_index=idx

        if selected_lane!=self.target:
            # Capture the current blend, without commanding instantaneous
            # lane teleport; for typical manoeuvres one lane at a time.
            if self.changing:
                p=smoothstep5(self.change_distance/LANE_SHIFT)
                if p>=.5:
                    self.lane=self.target
            self.target=selected_lane
            self.changing=self.lane!=self.target
            self.change_distance=0.0
        elif self.changing:
            self.change_distance+=distance
            if self.change_distance>=LANE_SHIFT:
                self.lane=self.target
                self.changing=False
                self.change_distance=LANE_SHIFT

        if self.initial_reference is None:
            self.initial_reference=(x,y)
        if self.initial_join:
            self.initial_distance+=distance
            if self.initial_distance>=LANE_SHIFT:
                self.initial_join=False

        lookahead=clamp(2.5+.35*max(0.0,speed),3.0,7.5)
        aim=self.track.advance(idx,lookahead)
        a=self.point(self.lane,aim)
        b=self.point(self.target,aim)
        blend=smoothstep5((self.change_distance+lookahead)/LANE_SHIFT) if self.changing else 1.0
        tx=a[0]+blend*(b[0]-a[0])
        ty=a[1]+blend*(b[1]-a[1])
        if self.initial_join:
            # Join from the actual grid's lateral offset, preserving the
            # orientation-following movement rather than jumping to lane.
            startidx=self.track.advance(idx,lookahead)
            center=self.point(self.target,startidx)
            initialidx=self.track.nearest(*self.initial_reference)
            initial_target=self.point(self.target,initialidx)
            dx=self.initial_reference[0]-initial_target[0]
            dy=self.initial_reference[1]-initial_target[1]
            weight=1.0-smoothstep5((self.initial_distance+lookahead)/LANE_SHIFT)
            tx+=dx*weight
            ty+=dy*weight

        alpha=wrap(math.atan2(ty-y,tx-x)-yaw)
        distance_to_target=max(1.0,math.hypot(tx-x,ty-y))
        left=math.atan2(2.0*self.wheelbase*math.sin(alpha),distance_to_target)
        desired=clamp(left,-MAX_STEER,MAX_STEER)
        desired=clamp(desired,
                      self.previous_steer-MAX_STEER_RATE*dt,
                      self.previous_steer+MAX_STEER_RATE*dt)
        yaw_cap=math.atan2(MAX_YAW_RATE*self.wheelbase,max(.1,speed))
        steer_cap=min(MAX_STEER,yaw_cap)
        desired=clamp(desired,-steer_cap,steer_cap)
        self.previous_steer=desired
        return Guidance(desired,self.target,tx,ty,self.changing,
                        self.change_distance/LANE_SHIFT if self.changing else 1.0,
                        MAX_YAW_RATE)
