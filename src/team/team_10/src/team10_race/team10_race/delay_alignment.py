"""Bounded command dead reckoning; contains no truth pose or map inputs.

A short planar model is an estimate, not odometry. The caller must certify the
remaining path with additional prediction uncertainty against the ORIGINAL rays.
"""
import bisect
import math
from .geometry import Point2


def rotate(x, y, yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return c*x-s*y,s*x+c*y


def command_motion(history, start, end, lag=.05):
    """Laser displacement in the sensor frame; +left geometry, +right commands."""
    if not 0 < end-start <= .30 or not history:
        return None
    times=[r[0] for r in history]
    if times[0]>start-lag or times[-1]<start-lag:
        return None
    cuts=[start]+[t+lag for t in times if start<t+lag<end]+[end]
    x,y,yaw=-.25,0.,0.
    for a,b in zip(cuts,cuts[1:]):
        _,v,steer=history[bisect.bisect_right(times,(a+b)/2-lag)-1]
        distance=v*(b-a);k=-math.tan(steer)/.340;theta=distance*k
        dx,dy=(distance,0.) if abs(k)<1e-9 else (math.sin(theta)/k,(1-math.cos(theta))/k)
        dx,dy=rotate(dx,dy,yaw);x+=dx;y+=dy;yaw+=theta
    dx,dy=rotate(.25,0.,yaw)
    return x+dx,y+dy,yaw


def align_path(path, motion, certified_distance):
    """Consume old observed path; never extend its certified length."""
    x,y,yaw=motion;travel=0.;previous=Point2(0.,0.);result=[]
    for point in path:
        leg=previous.distance(point)
        if travel+leg>certified_distance:
            ratio=max(0.,(certified_distance-travel)/leg) if leg else 0.
            point=Point2(previous.x+ratio*(point.x-previous.x),previous.y+ratio*(point.y-previous.y))
            last=True
        else:last=False
        qx,qy=rotate(point.x-x,point.y-y,-yaw)
        if qx>.05:result.append(Point2(qx,qy))
        if last:break
        travel+=leg;previous=point
    return tuple(result)


def path_target(path, distance):
    previous=Point2(0.,0.);travel=0.
    for point in path:
        length=previous.distance(point)
        if travel+length>=distance and length>1e-9:
            r=(distance-travel)/length
            return Point2(previous.x+r*(point.x-previous.x),previous.y+r*(point.y-previous.y))
        travel+=length;previous=point
    return None  # Never shorten requested lookahead by silently taking the end.
