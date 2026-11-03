"""Validated local road corridors from image and lidar returns."""
from dataclasses import dataclass
import math
import numpy as np

@dataclass
class RoadCorridor:
    centers: np.ndarray
    half_width: float
    valid: bool
    heading_deg: float
    bounds: np.ndarray = None   # optional local (x, right, left) from RGB+Depth


def estimate_road_corridor(points, *, min_width=.65, max_width=5.5):
    """Extract left/right guardrail clearance from local 2-D obstacles.

    No global track/map coordinates are used. This is a geometric corridor,
    not semantic road-surface segmentation: if walls are invisible it is
    explicitly invalid. Vehicles inside the roadway are excluded from the
    rail bands by multi-bin continuity checks.
    """
    pts=np.asarray(points,dtype=np.float64).reshape(-1,2)
    pts=pts[np.all(np.isfinite(pts),axis=1)]
    centers=[]
    for x in np.arange(.75,15.3,.55):
        p=pts[np.abs(pts[:,0]-x)<.40]
        l=p[(p[:,1]>.32)&(p[:,1]<4.0),1]
        r=p[(p[:,1]<-.32)&(p[:,1]>-4.0),1]
        if len(l)<2 or len(r)<2:
            continue
        # The inside faces of the two walls are near y=0.
        left=float(np.percentile(l,15));right=float(np.percentile(r,85))
        width=left-right
        if min_width<=width<=max_width:
            centers.append((x,.5*(left+right),width))
    if len(centers)<3:
        return RoadCorridor(np.empty((0,2)),0.,False,0.)
    a=np.asarray(centers)
    # Require continuity. A nearby opponent is not a road boundary.
    widths=a[:,2]
    median=float(np.median(widths))
    keep=np.abs(widths-median)<max(.35,median*.35)
    a=a[keep]
    if len(a)<3 or (a[-1,0]-a[0,0])<1.2:
        return RoadCorridor(np.empty((0,2)),0.,False,0.)
    centers=a[:,:2]
    slopes=np.diff(centers[:,1])/np.maximum(.1,np.diff(centers[:,0]))
    heading=math.degrees(math.atan(float(np.median(slopes))))
    return RoadCorridor(centers,float(np.median(a[:,2])/2),True,heading)


def fuse_verified_corridors(vision, lidar, *, max_center_error_m=.30,
                            max_width_error_m=.65):
    """Extend short aligned RGB/depth asphalt with independently sensed rails.

    NEVER read a preloaded track, odometry pose, or map. Corridor extension is
    permitted only where observed LiDAR and visual centres/widths agree across
    a useful overlapping interval; fused boundary is deliberately shrunken
    6 cm beyond vision to account for uncertain rail distance.
    """
    if vision is None or lidar is None or not vision.valid or not lidar.valid:
        return None
    if vision.bounds is None or len(vision.bounds)<4 or len(lidar.centers)<4:
        return None
    v=np.asarray(vision.bounds,dtype=float)
    lc=np.asarray(lidar.centers,dtype=float)
    lo=max(float(v[0,0]),float(lc[0,0]))
    hi=min(float(v[-1,0]),float(lc[-1,0]))
    if hi-lo<.75 or float(lc[-1,0])-float(v[-1,0])<.75:
        return None
    probes=np.linspace(lo+.10,hi-.10,5)
    vr=np.interp(probes,v[:,0],v[:,1]);vl=np.interp(probes,v[:,0],v[:,2])
    vc=.5*(vr+vl)
    ll=np.interp(probes,lc[:,0],lc[:,1])
    if (np.max(np.abs(vc-ll))>max_center_error_m or
            np.max(np.abs(vl-vr-2*float(lidar.half_width)))>max_width_error_m):
        return None
    extra=lc[lc[:,0]>v[-1,0]+.08]
    extra=extra[extra[:,0]<=min(15.0,v[-1,0]+12.)]
    if len(extra)<2:return None
    half=max(.30,float(lidar.half_width)-.06)
    additions=np.column_stack((extra[:,0],extra[:,1]-half,extra[:,1]+half))
    fused=np.vstack((v,additions))
    if np.any(np.diff(fused[:,0])<=0):return None
    centers=np.column_stack((fused[:,0],(fused[:,1]+fused[:,2])*.5))
    slope=np.diff(centers[:,1])/np.maximum(.05,np.diff(centers[:,0]))
    heading=math.degrees(math.atan(float(np.median(slope))))
    return RoadCorridor(centers,float(np.median((fused[:,2]-fused[:,1])/2)),
                        True,heading,fused)


