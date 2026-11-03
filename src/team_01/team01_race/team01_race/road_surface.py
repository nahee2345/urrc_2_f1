"""Aligned road RGB + depth-ground projection: no map or global pose access."""
from dataclasses import dataclass
import numpy as np


@dataclass
class AsphaltRoad:
    bounds: np.ndarray           # per row x, right_y, left_y in base frame
    coverage: float
    valid: bool


def dark_asphalt_bounds(ground_xy, ground_uv, rgb):
    """Use black/neutral pixels ONLY where depth confirms physical ground.

    Empty or ambiguous camera images never grant drivable-space authority.
    Consecutive distance slices bound a conservative visible asphalt corridor.
    """
    empty=AsphaltRoad(np.empty((0,3),dtype=float),0.,False)
    xy=np.asarray(ground_xy,dtype=float).reshape(-1,2)
    uv=np.asarray(ground_uv,dtype=int).reshape(-1,2)  # row, col
    frame=np.asarray(rgb)
    if frame.ndim!=3 or frame.shape[2]<3 or len(xy)<80 or len(uv)!=len(xy):
        return empty
    h,w=frame.shape[:2]
    if np.any(uv[:,0]<0) or np.any(uv[:,0]>=h) or np.any(uv[:,1]<0) or np.any(uv[:,1]>=w):
        return empty
    c=frame[uv[:,0],uv[:,1],:3].astype(np.float32)
    bright=c.mean(axis=1)
    spread=c.max(axis=1)-c.min(axis=1)
    # URRC asphalt texture is neutral blue-grey (42,44,47); outside
    # grass/dirt is dark olive (73,79,68), NOT road despite low luminance.
    blue_grey=(c[:,1]-c[:,2]<7.)
    black=(bright<105.)&(spread<40.)&blue_grey
    coverage=float(np.mean(black))
    # A black image with no depth-fit ground has already been rejected above.
    if np.count_nonzero(black)<65 or coverage<.24:
        return empty
    black_points=xy[black]
    candidates=[]
    for dist in np.arange(.65,7.,.32):
        seg=black_points[np.abs(black_points[:,0]-dist)<.27]
        if len(seg)<7:continue
        right=float(np.percentile(seg[:,1],8))
        left=float(np.percentile(seg[:,1],92))
        if .52<=left-right<=4.6:
            candidates.append((dist,right,left))
    if len(candidates)<4:return empty
    a=np.asarray(candidates,dtype=float)
    # Enforce contiguous measurements: do not bridge over invisible roadway.
    starts=[0]
    for i in range(1,len(a)):
        if a[i,0]-a[i-1,0]>.58:starts.append(i)
    starts.append(len(a))
    segments=[a[starts[k]:starts[k+1]] for k in range(len(starts)-1)]
    a=max(segments,key=lambda s:(len(s),s[-1,0]-s[0,0]))
    if len(a)<4 or (a[-1,0]-a[0,0])<1.:return empty
    return AsphaltRoad(a,coverage,True)


def transverse_start_marking(ground_xy, ground_uv, rgb, road_bounds):
    """Detect a broad, bright transverse road stripe under depth-confirmed ground.

    Do not use other vehicles, bright sky or texture-only pixels as a lap marker.
    Requires darker asphalt immediately before/after the candidate stripe.
    """
    xy=np.asarray(ground_xy,dtype=float).reshape(-1,2)
    uv=np.asarray(ground_uv,dtype=int).reshape(-1,2)
    im=np.asarray(rgb)
    if len(xy)<90 or len(uv)!=len(xy) or im.ndim!=3 or im.shape[2]<3 or road_bounds is None:
        return False
    h,w=im.shape[:2]
    if np.any(uv<0) or np.any(uv[:,0]>=h) or np.any(uv[:,1]>=w):return False
    bounds=np.asarray(road_bounds,dtype=float).reshape(-1,3)
    if len(bounds)<4:return False
    val=im[uv[:,0],uv[:,1],:3].astype(float)
    brightness=val.mean(axis=1)
    white=(brightness>157)&((val.max(axis=1)-val.min(axis=1))<55)
    dark=(brightness<115)&((val.max(axis=1)-val.min(axis=1))<45)
    x,y=xy[:,0],xy[:,1]
    valid=(x>=.45)&(x<=3.)&(x>=bounds[0,0])&(x<=bounds[-1,0])
    left=np.interp(x,bounds[:,0],bounds[:,2]);right=np.interp(x,bounds[:,0],bounds[:,1])
    width=np.maximum(.01,left-right)
    valid &= (y>right+.12)&(y<left-.12)
    for dist in np.arange(.55,2.65,.09):
        near=valid&(np.abs(x-dist)<.095)
        whites=near&white
        if np.count_nonzero(near)<9 or np.count_nonzero(whites)<6:continue
        span=np.ptp(y[whites])
        typical=float(np.median(width[whites]))
        if span<max(.38,typical*.51):continue
        before=valid&(x>dist-.42)&(x<dist-.20)
        after=valid&(x>dist+.20)&(x<dist+.42)
        if (np.count_nonzero(before)>=6 and np.count_nonzero(after)>=6 and
            np.mean(dark[before])>.35 and np.mean(dark[after])>.35):
            return True
    return False

def aligned_depth_rgb(depth_at, rgb_at, speed_mps, max_age_s=1.1):
    """ROS software cameras arrive asynchronously; align within a speed-aware limit."""
    if depth_at <= 0 or rgb_at <= 0:
        return False
    if not np.isfinite(speed_mps) or not np.isfinite(max_age_s):
        return False
    v=max(0.,float(speed_mps))
    allowed=min(float(max_age_s),max(.20,1.1/(1.+v*.28)))
    return abs(float(depth_at)-float(rgb_at))<=allowed
