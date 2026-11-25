"""Current-scan, complete-body clearance along an ordinary forward path.

The reference path is expressed at the primary laser, 0.25 m ahead of the rear
axle. This verifies geometry, not closed-loop tracking or road membership.
"""
from dataclasses import dataclass
import math
import numpy as np
from .geometry import Point2


@dataclass(frozen=True)
class PathSafety:
    distance: float
    clearance: float
    length: float
    reason: str


def near_path_agreement(reference, candidate, tolerance=.08):
    """A smooth wall fit may replace bin stair steps only near the same path."""
    a=np.array([(p.x,p.y) for p in reference if p.x>.05])
    b=np.array([(p.x,p.y) for p in candidate if p.x>.05])
    if len(a)<5 or len(b)<5:return False
    if np.any(np.diff(a[:,0])<=0) or np.any(np.diff(b[:,0])<=0):return False
    begin=max(a[0,0],b[0,0]);end=min(3.,a[-1,0],b[-1,0])
    if end-begin<1.5:return False
    x=np.linspace(begin,end,16)
    return bool(np.max(np.abs(np.interp(x,a[:,0],a[:,1])-np.interp(x,b[:,0],b[:,1])))<=tolerance)


def extend_straight(path, maximum_length, maximum_curvature=.02):
    """Propose a tangent continuation; verify_path MUST certify every added pose.

    This is not evidence that an unseen lane continues. Only current ray free
    space and the full body sweep may give this proposal a usable speed horizon.
    A currently observed curve is never replaced by a straight continuation.
    """
    pts=[p for p in path if p.x>.05 and math.isfinite(p.x) and math.isfinite(p.y)]
    if len(pts)<5:return tuple(pts)
    # Far wall bins have centimetre stair steps: a three-point derivative at
    # 0.25m spacing can turn that into fictitious sharp curvature. A small-
    # residual spatial fit is admissible only for a gently bending graph.
    original=tuple(pts)
    x=np.array([p.x for p in pts]);y=np.array([p.y for p in pts])
    if np.any(np.diff(x)<=0) or x[-1]-x[0]<2.:return original
    fit=np.polynomial.polynomial.polyfit(x,y,2)
    predicted=np.polynomial.polynomial.polyval(x,fit)
    residual=float(np.max(abs(predicted-y)))
    slopes=fit[1]+2*fit[2]*x
    curvature=float(np.max(abs(2*fit[2])/(1+slopes*slopes)**1.5))
    if residual>.04 or curvature>maximum_curvature or abs(fit[0])>.04:return original
    pts=[Point2(float(a),float(b-fit[0])) for a,b in zip(x,predicted)]
    if not near_path_agreement(original,pts):return original
    last=pts[-1];back=pts[-2]
    for point in reversed(pts[:-1]):
        back=point
        if last.distance(back)>=1.5:break
    heading=math.atan2(last.y-back.y,last.x-back.x)
    previous=Point2(0.,0.);length=0.
    for point in pts:length+=point.distance(previous);previous=point
    d=.25
    while length+d<=maximum_length:
        pts.append(Point2(last.x+d*math.cos(heading),last.y+d*math.sin(heading)))
        d+=.25
    return tuple(pts)


def _gentle_tail(path, maximum_length, curvature, near_distance=3.):
    # Keep the entire controller lookahead unchanged. Only the
    # stopping-horizon tail may bend, with radius >=50m and continuous heading.
    pts=[]
    for point in path:
        if point.x<=.05:continue
        pts.append(point)
        if point.x>=near_distance:break
    if len(pts)<5:return ()
    previous=Point2(0.,0.);length=0.
    for point in pts:length+=point.distance(previous);previous=point
    a,b=pts[-2:];heading=math.atan2(b.y-a.y,b.x-a.x)
    for distance in np.arange(.16,maximum_length-length+1e-6,.16):
        if abs(curvature)<1e-9:
            x=b.x+distance*math.cos(heading);y=b.y+distance*math.sin(heading)
        else:
            x=b.x+(math.sin(heading+curvature*distance)-math.sin(heading))/curvature
            y=b.y-(math.cos(heading+curvature*distance)-math.cos(heading))/curvature
        if x<=pts[-1].x:break
        pts.append(Point2(float(x),float(y)))
    return tuple(pts)


def extend_visible_path(scan, path, maximum_length, required_length=None, diagnostics=None,
                        near_distance=3.):
    """Select a current-scan-certified gentle continuation, never unseen space.

    Existing fit/near-path eligibility remains authoritative. A rigid tangent
    can leave a visible gently bending corridor. Compare small-curvature tails
    only when that existing extension is eligible and not fully observable.
    The same complete-body/ray check certifies every proposal; a shorter or
    only marginally longer alternative cannot replace the original path.
    """
    prepared=_prepare_scan(scan)
    tangent=extend_straight(path,maximum_length)
    safety=verify_path(scan,tangent,_prepared=prepared)
    best=tangent;selected=None
    needed=min(maximum_length-.16,required_length if required_length is not None else maximum_length)
    if diagnostics is not None:diagnostics.update(tangent_distance=safety.distance,body_checks=1)
    if len(tangent)<=len(path) or safety.distance>=needed:
        return best,safety,selected
    proposals=[]
    for curvature in (0.,-.005,.005,-.01,.01,-.015,.015,-.02,.02):
        candidate=_gentle_tail(tangent,maximum_length,curvature,near_distance)
        if not candidate:continue
        sweep=_path_sweep(candidate,.08)
        if sweep is None:continue
        _,ds,rear,c,s=sweep
        visible=_ray_visibility(scan,prepared[0],rear,c,s,.08)
        failed=np.flatnonzero(~visible);n=int(failed[0]) if len(failed) else len(ds)
        bound=float(ds[n-1]) if n else 0.
        proposals.append((bound,curvature,candidate))
    # Ray visibility is an upper bound on body-safe distance. Rank using this
    # cheap bound, then fully certify only candidates that could still win.
    # Every selected path receives the unchanged complete-body collision check.
    for bound,curvature,candidate in sorted(proposals,key=lambda p:(-p[0],abs(p[1]))):
        if bound<=safety.distance+.48:break
        checked=verify_path(scan,candidate,_prepared=prepared)
        if diagnostics is not None:diagnostics['body_checks']+=1
        if checked.distance>safety.distance+.48:
            best,safety,selected=candidate,checked,curvature
        if safety.distance>=needed:break
    return best,safety,selected


def _path_sweep(path, step):
    points=np.array([(p.x,p.y) for p in path if math.isfinite(p.x) and math.isfinite(p.y) and p.x>.05])
    if len(points)<2:return None
    points=np.vstack(([0.,0.],points))
    legs=np.diff(points,axis=0);lengths=np.linalg.norm(legs,axis=1)
    keep=np.r_[True,lengths>1e-5];points=points[keep]
    legs=np.diff(points,axis=0);lengths=np.linalg.norm(legs,axis=1)
    cumulative=np.r_[0.,np.cumsum(lengths)]
    length=float(cumulative[-1])
    ds=np.arange(step,length+1e-6,step)
    if not len(ds):return length,ds,np.empty((0,2)),np.array([]),np.array([])
    ids=np.minimum(np.searchsorted(cumulative,ds,side='right')-1,len(legs)-1)
    ratios=(ds-cumulative[ids])/lengths[ids]
    xy=points[ids]+legs[ids]*ratios[:,None]
    yaw=np.arctan2(legs[ids,1],legs[ids,0]);c=np.cos(yaw);s=np.sin(yaw)
    rear=xy-.25*np.c_[c,s]
    return length,ds,rear,c,s


def _prepare_scan(scan):
    ranges=np.asarray(scan.ranges,dtype=float)
    angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=scan.range_max)
    finite=ranges[valid];bearings=angles[valid]
    obs=np.c_[finite*np.cos(bearings),finite*np.sin(bearings)]
    own=(obs[:,0]>=-.35)&(obs[:,0]<=.206)&(abs(obs[:,1])<=.126)
    obs=obs[~own]
    return np.where(np.isposinf(ranges),scan.range_max,ranges),obs


def _ray_visibility(scan, observed_ranges, rear, c, s, margin):
    visible=np.ones(len(rear),dtype=bool)
    for ox,oy in [(.446+margin,.116+margin),(.446+margin,-.116-margin),(.446+margin,0.)]:
        px=rear[:,0]+c*ox-s*oy;py=rear[:,1]+s*ox+c*oy
        idx=np.rint((np.arctan2(py,px)-scan.angle_min)/scan.angle_increment).astype(int)
        inside=(idx>=1)&(idx<len(observed_ranges)-1);idx=np.clip(idx,1,len(observed_ranges)-2)
        bound=np.minimum.reduce([observed_ranges[idx+j] for j in (-1,0,1)])
        visible &= inside&np.isfinite(bound)&(np.hypot(px,py)<=bound-.02)
    return visible


def verify_path(scan, path, margin=.08, step=.08, *, _prepared=None):
    sweep=_path_sweep(path,step)
    if sweep is None:return PathSafety(0.,0.,0.,'NO_PATH')
    length,ds,rear,c,s=sweep
    if not len(ds):return PathSafety(0.,0.,length,'SHORT_PATH')
    observed_ranges,obs=_prepare_scan(scan) if _prepared is None else _prepared
    if len(obs):
        dx=obs[:,0][None,:]-rear[:,0,None];dy=obs[:,1][None,:]-rear[:,1,None]
        qx=c[:,None]*dx+s[:,None]*dy;qy=-s[:,None]*dx+c[:,None]*dy
        ex=np.maximum(np.maximum(-.09-qx,qx-.446),0.)
        ey=np.maximum(np.abs(qy)-.116,0.)
        clearance=np.min(np.hypot(ex,ey),axis=1)
    else:clearance=np.full(len(ds),math.inf)
    collision=clearance<margin
    visible=_ray_visibility(scan,observed_ranges,rear,c,s,margin)
    failed=np.flatnonzero(collision|~visible)
    n=int(failed[0]) if len(failed) else len(ds)
    reason='COLLISION' if n<len(ds) and collision[n] else ('UNOBSERVED' if n<len(ds) else 'PATH_END')
    return PathSafety(float(ds[n-1]) if n else 0.,float(np.min(clearance[:n])) if n else 0.,length,reason)
