"""Geometric curvature supported by finite chords, independent of PP."""
import numpy as np


def supported_curvature(path):
    points=np.asarray(path,dtype=float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<3 or not np.isfinite(points).all():
        raise ValueError('invalid_curvature_path')
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    if arc[-1]>=.6-1e-8:
        middle=np.arange(.3,arc[-1]-.3+1e-8,.1)
        positions=(middle-.3,middle,middle+.3)
    else:
        positions=(np.array([0.]),np.array([arc[-1]/2]),np.array([arc[-1]]))
    a,b,c=(np.column_stack([np.interp(s,arc,points[:,axis]) for axis in (0,1)]) for s in positions)
    u,v=b-a,c-b
    denominator=np.linalg.norm(u,axis=1)*np.linalg.norm(v,axis=1)*np.linalg.norm(u+v,axis=1)
    if not np.isfinite(denominator).all() or np.any(denominator<=1e-12):
        raise ValueError('invalid_curvature_support')
    return 2*(u[:,0]*v[:,1]-u[:,1]*v[:,0])/denominator
