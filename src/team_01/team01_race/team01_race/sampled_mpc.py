"""Experimental constrained local model-predictive control, SHADOW only.

Samples a small set of steering targets, simulates steering-rate-limited
Ackermann states, checks the live road envelope and actual obstacle cloud.
Not enabled as the production driving command until independent Gazebo proof.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class Proposal:
    steer_rad: float
    cost: float
    points: np.ndarray
    min_clearance_m: float


def search_shadow_mpc(road,reference,obstacles,speed,initial_steer,cfg,
                      horizon=3.0):
    if (road is None or not road.valid or reference is None or
            len(reference)<4 or len(road.centers)<4):
        return None
    ref=np.asarray(reference,float)
    if not np.all(np.isfinite(ref)) or np.any(np.diff(ref[:,0])<=0):
        return None
    if road.bounds is not None:
        b=np.asarray(road.bounds,float)
    else:
        c=np.asarray(road.centers,float)
        b=np.column_stack((c[:,0],c[:,1]-road.half_width,
                           c[:,1]+road.half_width))
    if b[0,0]>1.2 or b[-1,0]<1.4:return None
    obs=np.asarray(obstacles,float).reshape((-1,2))
    obs=obs[np.all(np.isfinite(obs),axis=1)]
    limit=min(float(horizon),float(b[-1,0])-.15,
              float(ref[-1,0])-.15)
    if limit<1.0:return None
    ds=.17
    v=max(.45,float(speed))
    dt=ds/v
    steps=int(limit/ds)
    if steps<5:return None
    maxsteer=math.radians(float(cfg['max_center_steer_deg']))
    rate=math.radians(float(cfg.get('max_steer_rate_deg_s',35.)))
    wb=float(cfg['wheel_base_m'])
    half=float(cfg['path_half_width_m'])+.07
    # Vary only the steering target, never change a verified reference
    # into an offset candidate outside current left/right bounds.
    from team01_race.stanley_control import stanley_steer
    baseline=stanley_steer(ref,0.,0.,0.,v,wb,maxsteer,cfg)
    offsets=np.radians((-7.,-3.5,0.,3.5,7.))
    best=None
    for bias in offsets:
        steer=float(initial_steer);x=y=yaw=0.
        coords=[];cost=0.;minimum=float('inf');first=None
        feasible=True
        for i in range(steps):
            wanted=stanley_steer(ref,x,y,yaw,v,wb,maxsteer,cfg)+bias
            wanted=float(np.clip(wanted,-maxsteer,maxsteer))
            nextsteer=float(np.clip(wanted,steer-rate*dt,
                                    steer+rate*dt))
            if first is None:first=nextsteer
            dsteer=nextsteer-steer
            steer=nextsteer
            curvature=math.tan(steer)/wb
            mid=yaw+.5*ds*curvature
            x+=ds*math.cos(mid)
            y+=ds*math.sin(mid)
            yaw+=ds*curvature
            if x>=b[-1,0]-.10:break
            if x>=b[0,0]:
                right=float(np.interp(x,b[:,0],b[:,1]))
                left=float(np.interp(x,b[:,0],b[:,2]))
                footprint=half+.275*abs(math.sin(yaw))
                clearance=min(y-right-footprint,left-y-footprint)
                minimum=min(minimum,clearance)
                if clearance<0.:
                    feasible=False;break
            if len(obs) and np.any(np.sum((obs-[x,y])**2,axis=1)<.28**2):
                feasible=False;break
            target=float(np.interp(x,ref[:,0],ref[:,1]))
            error=y-target
            cost+=3.*error*error+1.1*dsteer*dsteer+.05*steer*steer
            coords.append((x,y))
        if not feasible or len(coords)<5:continue
        cost+=.5*abs(float(bias))
        proposal=Proposal(float(first),cost,np.asarray(coords,float),
                          minimum)
        if best is None or proposal.cost<best.cost:
            best=proposal
    return best
