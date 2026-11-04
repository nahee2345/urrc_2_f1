"""Verified near/mid/far road ROI metrics in base_link metres.

Only measured boundary intervals are reported; peripheral wide-RGB pixels
without registered depth are for observation, not clearance authorization.
"""
import math
import numpy as np

BANDS={'near':(.5,3.),'middle':(3.,6.),'far':(6.,10.)}


def road_roi_metrics(road):
    result={}
    for name,(start,end) in BANDS.items():
        entry={'valid':False,'x_start_m':start,'x_end_m':end}
        if road is None or not road.valid or len(road.centers)<4:
            result[name]=entry
            continue
        c=np.asarray(road.centers,float)
        lo=max(start,float(c[0,0]));hi=min(end,float(c[-1,0]))
        if hi-lo<.65:
            result[name]=entry
            continue
        xs=np.linspace(lo,hi,12)
        y=np.interp(xs,c[:,0],c[:,1])
        if road.bounds is not None:
            b=np.asarray(road.bounds,float)
            width=np.interp(xs,b[:,0],b[:,2])-np.interp(xs,b[:,0],b[:,1])
        else:
            width=np.full_like(xs,2.*float(road.half_width))
        dy=np.gradient(y,xs)
        curvature=np.gradient(dy,xs)/(1.+dy*dy)**1.5
        entry.update(valid=True,
            observed_length_m=round(float(hi-lo),2),
            median_width_m=round(float(np.median(width)),2),
            heading_deg=round(math.degrees(math.atan(float(np.median(dy)))),2),
            curvature_m_inv=round(float(np.median(curvature)),4),
            peak_abs_curvature_m_inv=round(float(np.percentile(abs(curvature),80)),4))
        result[name]=entry
    return result
