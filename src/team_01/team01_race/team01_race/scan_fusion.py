"""Build one 360-degree base_link scan from front/rear measured LiDAR."""
import math
import numpy as np


def fused_ranges(front,rear,n=721):
    """Return lidar range bins, unknown beams remain inf; no invented free space."""
    if n<9:raise ValueError('scan requires >=9 beams')
    ranges=np.full(n,np.inf,dtype=float)
    for scan,xoff,yaw in ((front,.187,0.),(rear,-.208,math.pi)):
        if scan is None:continue
        vals=np.asarray(scan['ranges'],dtype=float)
        angles=scan['angle_min']+np.arange(len(vals))*scan['angle_increment']+yaw
        ok=np.isfinite(vals)&(vals>=scan['range_min'])&(vals<=scan['range_max'])
        if not np.any(ok):continue
        xs=xoff+vals[ok]*np.cos(angles[ok])
        ys=vals[ok]*np.sin(angles[ok])
        rad=np.hypot(xs,ys)
        bearing=np.arctan2(ys,xs)
        bins=np.rint((bearing+math.pi)*(n-1)/(2.*math.pi)).astype(int)
        valid=(rad>=.1)&(rad<=16.)&(bins>=0)&(bins<n)
        np.minimum.at(ranges,bins[valid],rad[valid])
    return ranges
