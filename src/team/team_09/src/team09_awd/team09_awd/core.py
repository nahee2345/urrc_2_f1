from .config import G, SETTINGS
"""Local sensor-only planner, v0.5. Model origin at centre; rear axle x=-G.wheelbase/2.
Rectangle includes front wheel steering envelope. Odom only compensates scan motion.
No ground-truth or stored racing line used in planning.
"""
from dataclasses import dataclass
import math
import numpy as np



def steering_rate(speed):
    """Equivalent centre-wheel rad/s; shared by actuator and trajectory predictions."""
    c=SETTINGS["control"]
    return min(c["steer_rate_max_radps"], c["steer_rate_base_radps"] + c["steer_rate_speed_gain"] / max(abs(speed), 1.)**2)

def reachable_curvature(desired, speed):
    # Full mechanical steering at low speed; lateral acceleration bound at speed.
    bound=min(G.max_curvature, SETTINGS["control"]["curvature_lateral_bound_mps2"]/max(speed*speed, 1.))
    return float(np.clip(desired, -bound, bound))


@dataclass
class Result:
    curvature: float = 0.
    free: float = 0.
    confidence: float = 0.
    clearance: float = 0.
    centre_error: float = 0.
    mode: str = 'BLOCKED'
    heading: float = 0.
    wall_quality: float = 0.


def clean_points(points):
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    return p[np.isfinite(p).all(axis=1)]


def external_points(points):
    p=clean_points(points)
    own=(np.abs(p[:,0])<G.half_length+.002)&(np.abs(p[:,1])<G.half_width)
    return p[~own]


def footprint_clearance(points):
    p = clean_points(points)
    if not len(p): return 0.
    d = np.maximum(np.abs(p) - [G.half_length, G.half_width], 0.)
    return float(np.linalg.norm(d, axis=1).min())


def compact(points):
    """Retain actual closest points, not reconstructed angular-bin centres."""
    p = clean_points(points)
    if not len(p): return p
    a = np.arctan2(p[:,1],p[:,0]);r = np.linalg.norm(p,axis=1)
    bins = np.clip(((a+np.pi)*720/np.pi).astype(int),0,1439)
    order = np.argsort(r)
    _,ix = np.unique(bins[order],return_index=True)
    return p[order[ix]]


def corridor_state(p):
    """Fit both local walls. Reject nonparallel bends and short obstacle clusters."""
    lines=[];errors=[]
    for sign in (1,-1):
        q=p[(p[:,0]>.20)&(p[:,0]<6.)&(sign*p[:,1]>.20)&(sign*p[:,1]<5.)]
        if len(q)<10 or np.ptp(q[:,0])<2.:return None
        for _ in range(3):
            fit=np.polyfit(q[:,0],q[:,1],1)
            residual=np.abs(q[:,1]-np.polyval(fit,q[:,0]))
            keep=residual<max(.07,float(np.median(residual))*2.5)
            q=q[keep]
            if len(q)<8 or np.ptp(q[:,0])<2.:return None
        lines.append(np.polyfit(q[:,0],q[:,1],1))
        errors.append(float(np.sqrt(np.mean((q[:,1]-np.polyval(lines[-1],q[:,0]))**2))))
    if abs(lines[0][0]-lines[1][0])>.15 or max(errors)>.12:return None
    mid=(lines[0]+lines[1])/2
    widths=[float(np.polyval(lines[0]-lines[1],x)) for x in (0.,5.)]
    if not all(.75<w<8. for w in widths):return None
    return float(mid[1]),math.atan(float(mid[0])),min(1.,1.-max(errors)/.15)


def centre_target(p):
    state=corridor_state(p)
    return None if state is None else float(np.clip(state[0]+1.5*math.tan(state[1]),-1.5,1.5))


def tracking_curvature(offset,heading,speed,yawrate=0.):
    # A central band is acceptable: remove cross-track correction inside +/-15cm.
    error=math.copysign(max(0.,abs(offset)-.15),offset)
    lookahead=max(4.,min(14.,4.+.8*speed))
    # Heading alignment dominates gentle lateral recentering.
    heading_lookahead=max(2.5,min(6.,2.5+.3*speed))
    desired=2.*math.sin(heading)/heading_lookahead+.65*error/(lookahead*lookahead)
    correction=float(np.clip(.12*(speed*desired-yawrate)/max(speed,3.),-.015,.015))
    # Cap centering lateral acceleration; obstacle/bend planner remains independent.
    cap=min(.35,1.5/max(speed*speed,4.))
    return float(np.clip(desired+correction,-cap,cap))


class WallTracker:
    """Filter a road line in odom coordinates, then predict it in current body frame.
    Does not treat odom as an absolute map or infer unseen collision clearance.
    """
    def __init__(self):self.line=None;self.stamp=None
    def update(self,state,pose,stamp):
        if state is None:
            self.line=None;self.stamp=None;return None
        offset,heading,quality=state;x,y,yaw=pose
        theta=yaw+heading
        normal=np.array([-math.sin(theta),math.cos(theta)])
        distance=offset*math.cos(heading)+float(normal@[x,y])
        if self.line is not None and self.stamp is not None and 0.<stamp-self.stamp<.2:
            old_theta,old_d=self.line
            dtheta=math.atan2(math.sin(theta-old_theta),math.cos(theta-old_theta))
            # Reject abrupt inconsistent wall fits; use fallback planner this frame.
            old_n=np.array([-math.sin(old_theta),math.cos(old_theta)])
            old_local=old_d-float(old_n@[x,y])
            if abs(dtheta)>.20 or abs(offset*math.cos(heading)-old_local)>.50:
                self.line=None;self.stamp=None;return None
            alpha=(stamp-self.stamp)/(.35+stamp-self.stamp)
            theta=old_theta+alpha*dtheta
            # Blend offsets at the CURRENT vehicle pose, avoiding rotation about odom origin.
            old_n=np.array([-math.sin(old_theta),math.cos(old_theta)])
            old_local=old_d-float(old_n@[x,y])
            local=(1-alpha)*old_local+alpha*offset*math.cos(heading)
            normal=np.array([-math.sin(theta),math.cos(theta)])
            distance=local+float(normal@[x,y])
        self.line=(theta,distance);self.stamp=stamp
        h=math.atan2(math.sin(theta-yaw),math.cos(theta-yaw))
        if abs(h)>.6:return None
        return (float((distance-normal@[x,y])/math.cos(h)),h,quality)


def rollout(k, previous, speed, horizon=32.):
    # Resolve short recovery manoeuvres densely; add sampling-error padding below.
    s=np.r_[0.,np.arange(.08,3.,.08),np.arange(3.,horizon+.01,.20)]
    ds=np.diff(s,prepend=0.)
    previous=float(np.clip(previous,-G.max_curvature,G.max_curvature));k=float(np.clip(k,-G.max_curvature,G.max_curvature))
    start=math.atan(G.wheelbase*previous);goal=math.atan(G.wheelbase*reachable_curvature(k,speed))
    delta=start+np.sign(goal-start)*np.minimum(abs(goal-start),steering_rate(speed)*s/max(speed,.4))
    mid_k=np.tan((delta+np.r_[start,delta[:-1]])/2)/G.wheelbase
    dyaw=mid_k*ds;yaw=np.cumsum(dyaw);mid=yaw-dyaw/2
    rear_x=-G.wheelbase/2+np.cumsum(ds*np.cos(mid));rear_y=np.cumsum(ds*np.sin(mid))
    return s,np.column_stack([rear_x+G.wheelbase/2*np.cos(yaw),rear_y+G.wheelbase/2*np.sin(yaw),yaw])


def evaluate(p,k,previous,speed,margin,horizon):
    s,poses=rollout(k,previous,speed,horizon)
    # Later poses cannot change the first yaw-limit hit; avoid their large point matrices.
    yaw_hit=np.flatnonzero(np.abs(poses[:,2])>1.3)
    if len(yaw_hit):
        end=int(yaw_hit[0])+1;s=s[:end];poses=poses[:end]
    rel=p[None,:,:]-poses[:,None,:2]
    c=np.cos(poses[:,2,None]);sn=np.sin(poses[:,2,None])
    x=rel[:,:,0]*c+rel[:,:,1]*sn;y=-rel[:,:,0]*sn+rel[:,:,1]*c
    # Minkowski rectangular expansion with near/far sampling padding.
    padding=np.where(s<3.,.045,.115)[:,None]
    blocked=((np.abs(x)<G.half_length+margin+padding)&(np.abs(y)<G.half_width+margin+padding)).any(axis=1)
    # Current pose checked using real extent+margin, without rollout discretization padding.
    blocked[0]=((np.abs(x[0])<G.half_length+margin)&(np.abs(y[0])<G.half_width+margin)).any()
    blocked|=np.abs(poses[:,2])>1.3
    hits=np.flatnonzero(blocked);free=float(s[hits[0]]) if len(hits) else horizon
    idx=max(0,min(len(s)-1,int(np.searchsorted(s,min(free*.6,2.)))-1))
    clear=float(np.sqrt(np.maximum(np.abs(x[idx])-G.half_length,0)**2+np.maximum(np.abs(y[idx])-G.half_width,0)**2).min())
    return free,clear,poses[idx],poses


def plan_local(points,previous=0.,speed=0.,camera_yaw=None,margin=.06,horizon=48.,yawrate=0.,wall_state=None,filtered_walls=False):
    p=compact(points)
    horizon=min(horizon,max(12.,speed*.30+speed*speed/8.+2.))
    if len(p)<30:return Result()
    clearance=footprint_clearance(p)
    state=wall_state if filtered_walls else corridor_state(p)
    target=(state[0]+1.5*math.tan(state[1])) if state else None
    preferred=tracking_curvature(*state[:2],speed,yawrate) if state else None
    conf=min(1.,len(p)/180.)
    # Current external points inside footprint stop; never equate odometry with no collision.
    if clearance<.025:return Result(clearance=clearance,confidence=conf)
    previous=float(np.clip(previous,-G.max_curvature,G.max_curvature))
    candidates=np.unique(np.r_[np.linspace(-G.max_curvature,G.max_curvature,49),np.linspace(-.10,.10,21),previous,0.,preferred if preferred is not None else 0.])
    # Accept the continuous corridor command only after the same swept-footprint test.
    if preferred is not None:
        free,_,_,_=evaluate(p,preferred,previous,speed,margin,horizon)
        required=3.
        if free>=required:
            # Keep a gentle centering path and reduce speed to its visible safe distance,
            # rather than switch abruptly to a longer, diagonally opposite path.
            return Result(preferred,free,conf,clearance,state[0],'RUN',state[1],state[2])
    best=None
    for padding,mode in [(margin,'RUN'),(.025,'RECOVER_CRAWL')]:
        # Small-margin recovery only at low speed and with measured positive clearance.
        if mode=='RECOVER_CRAWL' and (speed>.6 or clearance<.045):break
        for k in candidates:
            free,clear,end,poses=evaluate(p,float(k),previous,speed,padding,horizon)
            minimum=.45 if mode=='RECOVER_CRAWL' else .85
            if free<minimum:continue
            score=min(free,8.)+.6*min(clear,1.5)-.25*abs(k)-.30*abs(k-previous)
            if target is not None:score-=.35*abs(float(end[1])-target)
            if preferred is not None:score-=2.*abs(k-preferred)
            if camera_yaw is not None:score-=.15*abs(float(end[2])-camera_yaw)
            if best is None or score>best[0]:best=(score,float(k),free,mode)
        if best is not None:break
    if best is None:return Result(clearance=clearance,confidence=conf,centre_error=target or 0.)
    return Result(best[1],best[2],conf,clearance,state[0] if state else target or 0.,best[3],state[1] if state else 0.,state[2] if state else 0.)


def plan(points,previous=0.,camera_yaw=None,margin=.06):
    r=plan_local(points,previous,camera_yaw=camera_yaw,margin=margin)
    return r.curvature,r.free,r.confidence


def speed_limit(k,free,confidence,maximum,lateral=1.8,brake=4.,latency=.20,reserve=.45):
    distance=max(0.,free-reserve)
    braking=max(0.,-brake*latency+math.sqrt((brake*latency)**2+2*brake*distance))
    corner=math.sqrt(lateral/max(abs(k),.001))
    return max(0.,min(maximum,corner,braking)*min(1.,max(0.,confidence)))


def interpolate_pose(history,stamp,max_gap=.15):
    if not history:return None
    values=list(history)
    if stamp<values[0][0] or stamp>values[-1][0]:
        v=min(values,key=lambda x:abs(x[0]-stamp))
        return v[1:] if abs(v[0]-stamp)<=max_gap else None
    for a,b in zip(values,values[1:]):
        if a[0]<=stamp<=b[0]:
            if b[0]-a[0]>max_gap:return None
            f=(stamp-a[0])/max(b[0]-a[0],1e-9)
            dy=math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]))
            return (a[1]+f*(b[1]-a[1]),a[2]+f*(b[2]-a[2]),a[3]+f*dy)
    return None


def compensate(points,source_pose,current_pose):
    p=clean_points(points);x,y,a=source_pose;u,v,b=current_pose
    c,s=math.cos(a),math.sin(a)
    gx=c*p[:,0]-s*p[:,1]+x-u;gy=s*p[:,0]+c*p[:,1]+y-v
    c,s=math.cos(b),math.sin(b)
    return np.column_stack([c*gx+s*gy,-s*gx+c*gy])


class ResumeGate:
    """Three distinct safe sensor frames required after a blocked/stale state."""
    def __init__(self):self.count=0;self.last=None;self.running=False
    def update(self,safe,stamp):
        if not safe:self.count=0;self.running=False;self.last=stamp;return False
        if self.last!=stamp:self.count+=1;self.last=stamp
        if self.count>=3:self.running=True
        return self.running

# Compatibility exports for the retained pure planner regression suite.
def __getattr__(name):
    mapping = {'HALF_LENGTH': 'half_length', 'HALF_WIDTH': 'half_width', 'WHEELBASE': 'wheelbase', 'KINGPIN_WIDTH': 'track', 'MAX_WHEEL_ANGLE': 'max_wheel_angle', 'MAX_CURVATURE': 'max_curvature', 'MAX_CENTRE_ANGLE': 'max_centre_angle'}
    if name in mapping: return getattr(G, mapping[name])
    raise AttributeError(name)
