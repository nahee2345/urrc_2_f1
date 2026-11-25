from .config import G, SETTINGS
"""Local static-obstacle bypass. No vehicle classification or moving-rival prediction."""
import numpy as np
from .core import footprint_clearance,Result

class BypassState:
    def __init__(self):self.side=0;self.clear_since=None
    def clear(self,stamp):
        if stamp is None:return
        if self.clear_since is None:self.clear_since=stamp
        if stamp-self.clear_since>.6:self.side=0
    def commit(self,side):self.side=side;self.clear_since=None

def propose(points,fit,previous,speed,yawrate,margin,maximum,lateral,brake,latency,state,stamp):
    # Import lazily: racing owns the single trajectory generator and sweep model.
    from .racing import route_rollout,check_route,route_command,preview_speed,proximity_cap
    poly,horizon,width,quality=fit
    q=points[(points[:,0]>1.)&(points[:,0]<min(horizon-1,12.))]
    if len(q):q=q[abs(q[:,1]-np.polyval(poly,q[:,0]))<width/2-.40]
    if len(q)<4:
        state.clear(stamp);return None
    nearest=float(q[:,0].min());cluster=q[q[:,0]<nearest+.8]
    if len(cluster)<4:return None
    d=max(3.,nearest+.5);end=min(horizon,1.8*d,nearest+4.)
    if end<nearest+1.2:return None
    offsets=[x for x in (.45,-.45,.65,-.65,.85,-.85) if abs(x)+G.half_width+margin+.12<width/2]
    choices=[]
    for offset in offsets:
        side=1 if offset>0 else -1
        if state.side and side!=state.side and speed>.6:continue
        candidate=poly+np.array([0.,offset/d,0.])
        s,poses,curves=route_rollout(candidate,previous,speed,end,yawrate)
        # Remain inside the observed corridor, including lateral vehicle margin.
        clearance=width/2-abs(poses[:,1]-np.polyval(poly,poses[:,0]))
        free,index=check_route(points,s,poses,margin)
        edge=np.flatnonzero(clearance<G.half_width+margin+.10)
        if len(edge) and edge[0]<index:index=int(edge[0]);free=float(s[index])
        if free<nearest+1.2:continue
        choices.append((free-.15*abs(offset),side,candidate,s,poses,curves,free,index))
    if not choices:
        # Re-evaluate using the ordinary swept-footprint planner; never latch a wait.
        return None
    _,side,poly,s,poses,curves,free,index=max(choices,key=lambda x:x[0])
    state.commit(side);k=route_command(poly,0,0,0,speed,yawrate)
    r=Result(k,free,min(1.,len(points)/180.),footprint_clearance(points),float(poly[2]),'RUN',float(np.arctan(poly[1])),quality)
    r.route_kind='BYPASS_LEFT' if side>0 else 'BYPASS_RIGHT';r.vision_fused=False
    r.route_poses=poses[:index+1];r.route_dist=s[:index+1]
    r.preview_limit=preview_speed(s,curves,free,maximum,lateral,brake,latency)
    r.proximity_limit=proximity_cap(points,k,speed,r.clearance,latency,yawrate)
    r.road_curvature=float(2*poly[0]/(1+poly[1]**2)**1.5)
    return r
