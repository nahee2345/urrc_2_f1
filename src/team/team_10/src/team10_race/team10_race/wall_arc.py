"""Current-scan wall-normal path when a tight bend cannot be expressed as y(x)."""
import math
import numpy as np
from .geometry import Point2, median
from .centerline import CenterlineEstimate


def build_wall_arc_centerline(segments, observation, step=.25, minimum_length=1.20,
                              minimum_clearance=.35):
    """Offset a contiguous measured boundary; never extrapolate unseen geometry.

    This is only a fallback supported by a long current-scan boundary. Every output point is
    checked against all current LiDAR returns. Existing perception, collision,
    command freshness and start gates still apply downstream.
    """
    obstacles=[item.point for seg in segments for item in seg]
    obstacle_xy=np.array([(p.x,p.y) for p in obstacles])
    choices=[]
    for side in ('left','right'):
        sign=1 if side=='left' else -1
        for seg in segments:
            seeds=[p.point for p in seg if -.10<=p.point.x<=1.2 and sign*p.point.y>.18]
            if len(seeds)<3:continue
            wall=[p.point for p in seg]
            if side=='left':wall.reverse()
            used={id(seg)}
            # Self-occlusion can remove several beams even when the two measured
            # boundary ends are close. Join only aligned, spatially adjacent ends.
            while len(wall)>=7:
                a,b=wall[-7],wall[-1]
                heading=math.atan2(b.y-a.y,b.x-a.x)
                joins=[]
                for other in segments:
                    if id(other) in used or len(other)<7:continue
                    q=[p.point for p in other]
                    if side=='left':q.reverse()
                    gap=b.distance(q[0]);h=math.atan2(q[6].y-q[0].y,q[6].x-q[0].x)
                    turn=abs(math.atan2(math.sin(h-heading),math.cos(h-heading)))
                    if gap<=.30 and turn<=.50:joins.append((gap,other,q))
                if not joins:break
                _,other,q=min(joins,key=lambda item:item[0]);wall.extend(q);used.add(id(other))
            # Reject compact objects and normal-noise loops as wall support.
            if len(wall)<8 or wall[0].distance(wall[-1])<1.5:continue
            if sum(a.distance(b) for a,b in zip(wall,wall[1:]))<1.8:continue
            offset=[]
            for i,p in enumerate(wall):
                a=wall[max(0,i-6)];b=wall[min(len(wall)-1,i+6)]
                dx,dy=b.x-a.x,b.y-a.y;norm=math.hypot(dx,dy)
                if norm<.02:continue
                half=.5*observation.track_width_m
                # Right normal for left wall; left normal for right wall.
                offset.append(Point2(p.x+sign*half*dy/norm,p.y-sign*half*dx/norm))
            if not offset:continue
            offset=[Point2(median(q.x for q in offset[max(0,i-3):i+4]),
                           median(q.y for q in offset[max(0,i-3):i+4])) for i in range(len(offset))]
            starts=[i for i,p in enumerate(offset) if .15<p.x<.9 and p.norm<1.0]
            if not starts:continue
            start=min(starts,key=lambda i:offset[i].norm)
            # Same all-return Euclidean clearance test, batched in NumPy.
            # The old nested Python generator dominated tight-bend callback
            # time and could starve the independent permission lease.
            candidates=offset[start:]
            delta=np.array([(p.x,p.y) for p in candidates])[:,None,:]-obstacle_xy[None,:,:]
            nearest_squared=np.min(np.sum(delta*delta,axis=2),axis=1)
            path=[]
            for p,distance_squared in zip(candidates,nearest_squared):
                if p.x<=.05:break
                if path and p.distance(path[-1])>.30:break
                if distance_squared<minimum_clearance*minimum_clearance:break
                path.append(p)
            if len(path)<5:continue
            length=sum(a.distance(b) for a,b in zip(path,path[1:]))
            if length<minimum_length:continue
            # Uniform arc-length sampling preserves the measured bend, including
            # portions whose forward x extent is small at an almost 90 degree turn.
            result=[path[0]];next_at=max(.1,step);walked=0.
            for a,b in zip(path,path[1:]):
                ds=a.distance(b)
                while ds>1e-9 and walked+ds>=next_at:
                    f=(next_at-walked)/ds
                    result.append(Point2(a.x+f*(b.x-a.x),a.y+f*(b.y-a.y)))
                    next_at+=max(.1,step)
                walked+=ds
            if len(result)<5:continue
            heading=math.atan2(result[1].y-result[0].y,result[1].x-result[0].x)
            if abs(heading)>1.3:continue
            score=result[0].norm+.25*abs(heading)-.05*min(length,4.)
            choices.append((score,result,side,len(wall)))
    if not choices:return CenterlineEstimate((),'NONE',observation.track_width_m)
    _,path,side,support=min(choices,key=lambda v:v[0])
    return CenterlineEstimate(tuple(path),'RAW_'+side.upper()+'_ARC_FALLBACK',observation.track_width_m,
                              forward_span_m=max(p.x for p in path)-min(p.x for p in path), raw_point_count=support)
