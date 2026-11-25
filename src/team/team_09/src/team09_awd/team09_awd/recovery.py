from .config import G, SETTINGS
"""Contact escape for the simulator. No permanent contact-stop latch.
Initial contact may persist briefly, but new contacts or deeper penetration reject a path.
No feasible escape -> replan next frame, rather than driving blindly into a wall.
"""
import numpy as np
from .core import compact,rollout,Result,footprint_clearance

def escape_plan(points,previous,speed):
    p=compact(points)
    if speed>=.6 or len(p)<30:return None
    choices=[]
    rear_observed=np.sum((p[:,0]<-.3)&(p[:,1]>.2))>=10 and np.sum((p[:,0]<-.3)&(p[:,1]<-.2))>=10
    for direction,k in [(d,k) for d in ([1,-1] if rear_observed else [1]) for k in np.linspace(-G.max_curvature,G.max_curvature,25)]:
        s,poses=rollout(float(k),previous,.4,horizon=1.)
        if direction<0:
            # Integrate negative travel with the same front steering geometry.
            import math
            from .core import steering_rate
            rear=np.array([-G.wheelbase/2,0.]);yaw=0.;delta=math.atan(G.wheelbase*previous);rows=[]
            for i,dist in enumerate(s):
                if i:
                    ds=float(s[i]-s[i-1]);goal=math.atan(G.wheelbase*k)
                    delta+=float(np.clip(goal-delta,-steering_rate(.4)*ds/.4,steering_rate(.4)*ds/.4))
                    turn=-math.tan(delta)/G.wheelbase*ds
                    rear-=ds*np.array([math.cos(yaw+turn/2),math.sin(yaw+turn/2)]);yaw+=turn
                centre=rear+G.wheelbase/2*np.array([math.cos(yaw),math.sin(yaw)])
                rows.append([*centre,yaw])
            poses=np.array(rows)
        mask=s<=.72;s=s[mask];poses=poses[mask]
        rel=p[None,:,:]-poses[:,None,:2]
        c=np.cos(poses[:,2,None]);sn=np.sin(poses[:,2,None])
        x=rel[:,:,0]*c+rel[:,:,1]*sn;y=-rel[:,:,0]*sn+rel[:,:,1]*c
        gap=np.maximum(abs(x)-G.half_length,abs(y)-G.half_width)
        initial=gap[0]
        # Allow only existing near contacts; never permit increasing penetration.
        if np.any(gap < np.minimum(initial,0.)[None,:]-1e-5):continue
        near=initial<.06
        if not np.any(near) or np.min(gap[-1])<.035:continue
        if np.any(gap[-1,near]<initial[near]+.025):continue
        choices.append((float(np.min(gap[-1]))-.02*abs(k)-(.1 if direction<0 else 0),k,s,poses,direction))
    if not choices:return None
    _,k,s,poses,direction=max(choices,key=lambda a:a[0])
    r=Result(float(k),float(s[-1]),min(1.,len(p)/180.),footprint_clearance(p),0.,'RECOVER_CRAWL' if direction>0 else 'RECOVER_REVERSE')
    r.route_kind='CONTACT_ESCAPE' if direction>0 else 'CONTACT_REVERSE';r.route_dist=s;r.route_poses=poses
    r.preview_limit=.25;r.proximity_limit=.25;r.road_curvature=float(k)
    return r
