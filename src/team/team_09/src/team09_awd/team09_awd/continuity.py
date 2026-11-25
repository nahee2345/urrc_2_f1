from .config import G, SETTINGS
"""Bounded sensor synchronization and continuous commands; independent of ROS."""
import math
from collections import deque
from .core import interpolate_pose

class ScanBuffer:
    def __init__(self): self.frames=deque(maxlen=8)
    def add(self,stamp,points,arrival):
        if self.frames and stamp<self.frames[-1][0]: self.frames.clear()
        self.frames.append((stamp,points,arrival))
    def select(self,history,now,max_age=.20):
        if not history:return None
        for stamp,points,arrival in reversed(self.frames):
            # Never call a nearest future pose a synchronized measurement.
            if not 0<=now-stamp<=max_age or not history[0][0]<=stamp<=history[-1][0]:continue
            pose=interpolate_pose(history,stamp)
            if pose is not None:return stamp,points,arrival,pose
        return None

def command_step(previous,target,dt,accel,brake,hard_cap):
    value=previous+max(-brake*dt,min(accel*dt,target-previous))
    return max(0.,min(value,hard_cap))

def wheel_angles(k):
    k=max(-G.max_curvature,min(G.max_curvature,k))
    return tuple(math.degrees(math.atan(G.wheelbase*k/(1-sign*G.track*.5*k))) for sign in (1,-1))


def plan_is_current(start,finish,scan_stamp,epoch,current_epoch,max_age=.30):
    return epoch==current_epoch and finish>=start and 0.<=finish-scan_stamp<=max_age
