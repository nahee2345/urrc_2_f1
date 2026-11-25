"""Sensor-local, forward Ackermann rollouts for bends that are not y(x) graphs.

No map, pose, checkpoint, previous path, or elapsed-race-time input. All poses here
are predictions relative to the CURRENT primary laser. The F1 rear axle is .25m
behind that laser. Its complete .536 x .232m envelope is swept against every raw
return (including unclassified obstacles); leading corners must remain observed.
"""
from dataclasses import dataclass
import math
import numpy as np
from .geometry import Point2
from .centerline import CenterlineEstimate

@dataclass(frozen=True)
class FootprintTrajectory:
    centerline: CenterlineEstimate
    steering: float
    free_distance: float
    minimum_clearance: float
    poses: tuple[tuple[float,float,float], ...]


def plan_footprint_trajectory(scan, segments, observation, current_steering,
                             speed_mps, maximum_steering=.488692,
                             steering_rate=2., margin=.08, horizon=4., step=.08):
    # Require long CURRENT wall support; absence of walls is never accepted blindly.
    support=max((len(s) for s in segments if len(s)>=8 and
        sum(a.point.distance(b.point) for a,b in zip(s,s[1:]))>=1.8),default=0)
    if not support:return None
    ranges=np.asarray(scan.ranges,dtype=float)
    # LaserScan +Inf is a valid no-hit ray through range_max, not NaN or
    # missing reception. Preserve invalid/negative-infinite rays as unknown.
    ray_ranges=np.where(np.isposinf(ranges),scan.range_max,ranges)
    angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=scan.range_max)
    points=np.column_stack((ranges*np.cos(angles),ranges*np.sin(angles)))
    # Returns within our measured envelope are self returns, not free space evidence.
    own=(points[:,0]>=-.35)&(points[:,0]<=.206)&(np.abs(points[:,1])<=.126)
    obstacles=points[valid&~own&(ranges<horizon+.8)]
    if len(obstacles)<8:return None
    candidates=np.unique(np.r_[np.linspace(-maximum_steering,maximum_steering,17),
                               max(-maximum_steering,min(maximum_steering,current_steering))])
    count=int(horizon/step)
    rear=np.zeros((len(candidates),count,3));rear[:,:,0]=-.25
    x=np.full(len(candidates),-.25);y=np.zeros(len(candidates));yaw=np.zeros(len(candidates))
    # Predict finite steering slew at a conservative reference forward speed.
    transition=np.maximum(.20,np.abs(candidates-current_steering)/steering_rate*max(1.,speed_mps))
    for i in range(count):
        d=(i+.5)*step
        delta=current_steering+(candidates-current_steering)*np.minimum(1.,d/transition)
        turn=np.tan(delta)/.340*step
        x+=step*np.cos(yaw+.5*turn);y+=step*np.sin(yaw+.5*turn);yaw+=turn
        rear[:,i,:]=np.c_[x,y,yaw]
    c=np.cos(rear[:,:,2]);s=np.sin(rear[:,:,2])
    dx=obstacles[:,0][None,None,:]-rear[:,:,0,None]
    dy=obstacles[:,1][None,None,:]-rear[:,:,1,None]
    qx=c[:,:,None]*dx+s[:,:,None]*dy;qy=-s[:,:,None]*dx+c[:,:,None]*dy
    # Physical envelope relative to rear axle: [-.09,.446] x [-.116,.116].
    ex=np.maximum(np.maximum(-.09-qx,qx-.446),0.)
    ey=np.maximum(np.abs(qy)-.116,0.)
    clearances=np.min(np.hypot(ex,ey),axis=2)
    safe=clearances>=margin
    # Prevent plans through unobserved space behind an occluding wall. Both front
    # padded corners and the nose centre must lie inside the current ray free-space.
    for ox,oy in [(.446+margin,.116+margin),(.446+margin,-.116-margin),(.446+margin,0.)]:
        px=rear[:,:,0]+c*ox-s*oy;py=rear[:,:,1]+s*ox+c*oy
        bearing=np.arctan2(py,px)
        idx=np.rint((bearing-scan.angle_min)/scan.angle_increment).astype(int)
        idx=np.clip(idx,0,len(ranges)-1)
        observed=np.minimum.reduce([ray_ranges[np.clip(idx+j,0,len(ranges)-1)] for j in [-1,0,1]])
        safe &= np.isfinite(observed)&(np.hypot(px,py)<=observed-.02)
    lengths=np.sum(np.cumprod(safe,axis=1),axis=1)*step
    choices=[]
    for j,delta in enumerate(candidates):
        length=float(lengths[j]);n=int(round(length/step))
        if length<1.20:continue
        near_clear=float(np.min(clearances[j,:min(n,25)]))
        # Prefer observed traversable distance, then centre clearance and smooth
        # steering. This reacts before a front wall, not at a map-specific bend.
        score=length+.7*min(.5,near_clear)-.25*abs(delta-current_steering)-.08*abs(delta)
        choices.append((score,j,n,near_clear))
    if not choices:return None
    _,j,n,near_clear=max(choices)
    poses=rear[j,:n]
    path=tuple(Point2(float(p[0]+.25*math.cos(p[2])),float(p[1]+.25*math.sin(p[2]))) for p in poses[1::3])
    if len(path)<5:return None
    return FootprintTrajectory(CenterlineEstimate(path,'RAW_FOOTPRINT_ROLLOUT',observation.track_width_m,
        forward_span_m=max(p.x for p in path)-min(p.x for p in path),raw_point_count=support),
        float(candidates[j]),float(lengths[j]),near_clear,tuple(tuple(float(v) for v in p) for p in poses))
