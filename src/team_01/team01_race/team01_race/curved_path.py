"""Depth/RGB curb-bounded local pure-pursuit rollout.

This is a *receding-horizon prediction*, not a world-map trajectory. Each
candidate uses changing steering while following multiple observed road
points. A constant-steering arc cannot generally follow an S-shaped track.
The resulting first steering command still goes through the existing
Ackermann adapter and collision envelope.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class Candidate:
    steer_rad: float
    points: np.ndarray  # predicted (x, y) in base_link frame
    lengths: np.ndarray
    max_curvature: float
    min_edge_clearance: float
    first_collision_m: float
    score: float


def _road_arrays(road):
    if not road.valid or len(road.centers)<4:
        return None
    centers=np.asarray(road.centers,dtype=float)
    if road.bounds is not None:
        b=np.asarray(road.bounds,dtype=float)
    else:
        b=np.column_stack((centers[:,0],
                           centers[:,1]-float(road.half_width),
                           centers[:,1]+float(road.half_width)))
    if len(b)<4 or not np.all(np.diff(b[:,0])>0):
        return None
    if not np.all(np.isfinite(b)) or not np.all(b[:,2]>b[:,1]):
        return None
    return b


def adaptive_pursuit_candidate(road, obstacles, cfg, speed, desired_offset=0.,
                               relative_offset=0., horizon=5., reference=None):
    """Return only an edge- and obstacle-safe, varying-steer candidate.

    The local target is sampled repeatedly as the simulated Ackermann bicycle
    advances. Always include the vehicle footprint in edge tests. Unknown
    stretches beyond the observed road are never treated as drivable.
    """
    b=_road_arrays(road)
    if b is None or b[0,0]>1.25 or b[-1,0]<1.35:
        return None
    wb=float(cfg['wheel_base_m'])
    max_steer=math.radians(float(cfg['max_center_steer_deg']))
    margin=float(cfg['path_half_width_m'])+.07
    # The curve planner must check the current near-field road envelope before
    # accepting an otherwise-valid trajectory farther down the road. v7 could
    # continue after the near-side curb was already behind the footprint.
    near_x=min(max(.75,float(b[0,0])),float(b[-1,0]))
    near_r=float(np.interp(near_x,b[:,0],b[:,1]))
    near_l=float(np.interp(near_x,b[:,0],b[:,2]))
    if near_r+margin>=0 or near_l-margin<=0:
        return None
    last_x=float(b[-1,0])
    limit=max(.5,min(float(horizon),last_x-.20,14.0))
    obs=np.asarray(obstacles,dtype=float).reshape((-1,2))
    obs=obs[np.all(np.isfinite(obs),axis=1)]
    off=float(desired_offset)+float(relative_offset)
    ref=None if reference is None else np.asarray(reference,dtype=float)
    if ref is not None and (ref.ndim!=2 or ref.shape[1]!=2 or
                            len(ref)<4 or not np.all(np.isfinite(ref)) or
                            np.any(np.diff(ref[:,0])<=0)):
        return None
    x=y=yaw=s=0.
    positions=[];lengths=[]
    first_steer=None;maxcurv=0.;min_edge=100.;first_hit=float('inf')
    look=max(1.2,min(6.0,1.5+max(0.,float(speed))*.38))
    if abs(float(road.heading_deg))>8.:look=min(look,3.4)
    ds=.14
    n=int(limit/ds)
    for i in range(n):
        look_x=min(last_x-.08,max(float(b[0,0]), x+look))
        right=float(np.interp(look_x,b[:,0],b[:,1]))
        left=float(np.interp(look_x,b[:,0],b[:,2]))
        target_y=(float(np.interp(look_x,ref[:,0],ref[:,1]))+float(relative_offset)
                  if ref is not None else (left+right)*.5+off)
        # Clip target into observed road envelope with footprint clearance.
        target_y=max(right+margin,min(left-margin,target_y))
        dx=look_x-x;dy=target_y-y
        d2=max(.02,dx*dx+dy*dy)
        local_y=-math.sin(yaw)*dx+math.cos(yaw)*dy
        steer=float(np.clip(math.atan(2*wb*local_y/d2),-max_steer,max_steer))
        if first_steer is None:first_steer=steer
        k=math.tan(steer)/wb
        maxcurv=max(maxcurv,abs(k))
        # Midpoint integration reduces curvature error for tight corners.
        midyaw=yaw+.5*ds*k
        nx=x+ds*math.cos(midyaw)
        ny=y+ds*math.sin(midyaw)
        nyaw=yaw+ds*k
        if nx>last_x-.10 or nx>limit+.03:break
        if nx>=b[0,0]:
            rr=float(np.interp(nx,b[:,0],b[:,1]));ll=float(np.interp(nx,b[:,0],b[:,2]))
            # Turning cars occupy additional lateral space due to yaw.
            # 0.275m is half of the maximum regulation vehicle length.
            footprint=margin + .275*abs(math.sin(nyaw))
            edge=min(ny-rr-footprint,ll-ny-footprint)
            min_edge=min(min_edge,edge)
            if edge<-.005:return None
        if len(obs):
            # Inflated footprint; reject candidate near any actual obstacle.
            close=np.any((obs[:,0]-nx)**2+(obs[:,1]-ny)**2<(.25)**2)
            if close and first_hit==float('inf'):first_hit=s+ds
        x,y,yaw=nx,ny,nyaw;s+=ds
        positions.append((x,y));lengths.append(s)
    if not positions or s<.65 or first_steer is None:return None
    if first_hit<max(1.,s-.1):return None
    # Favor the desired path and generous clearance; discourage large wheel
    # corrections. Off-center trajectories are allowed when boundaries permit.
    score=(s+.10*min(2.,min_edge)-.70*abs(relative_offset)
           -.12*abs(first_steer)-.035*maxcurv)
    return Candidate(first_steer,np.asarray(positions),np.asarray(lengths),
                     maxcurv,min_edge,min(first_hit,s),score)


def preview_curvature(road):
    """Conservative observed centerline curvature for pre-corner speed cap."""
    if not road.valid or len(road.centers)<5:return 0.
    c=np.asarray(road.centers,dtype=float)
    xs=np.arange(max(.7,float(c[0,0])),min(float(c[-1,0]),5.5),.35)
    if len(xs)<4:return 0.
    ys=np.interp(xs,c[:,0],c[:,1])
    dy=np.gradient(ys,xs); ddy=np.gradient(dy,xs)
    k=np.abs(ddy)/(1+dy*dy)**1.5
    return float(np.percentile(k,80))
