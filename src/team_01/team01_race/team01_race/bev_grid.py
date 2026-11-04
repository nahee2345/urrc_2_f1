"""Observed local bird's-eye occupancy with unknown/unverified space = -1."""
import numpy as np


def local_bev(road, obstacles, resolution=.2, forward=10., lateral=3.):
    width=int(round(forward/resolution))
    height=int(round(2*lateral/resolution))
    grid=np.full((height,width),-1,dtype=np.int8)
    if road is not None and road.valid and len(road.centers)>=4:
        x=(np.arange(width)+.5)*resolution
        y=(np.arange(height)+.5)*resolution-lateral
        if road.bounds is not None:
            b=np.asarray(road.bounds,float)
        else:
            c=np.asarray(road.centers,float)
            b=np.column_stack((c[:,0],c[:,1]-road.half_width,
                               c[:,1]+road.half_width))
        known=(x>=b[0,0])&(x<=b[-1,0])
        if np.any(known):
            right=np.interp(x[known],b[:,0],b[:,1])+.23
            left=np.interp(x[known],b[:,0],b[:,2])-.23
            free=(y[:,None]>right[None,:])&(y[:,None]<left[None,:])
            grid[:,known]=np.where(free,0,-1)
    obs=np.asarray(obstacles,float).reshape((-1,2))
    obs=obs[np.all(np.isfinite(obs),axis=1)]
    for px,py in obs:
        ix=int(px/resolution);iy=int((py+lateral)/resolution)
        if 0<=ix<width and 0<=iy<height:
            grid[max(0,iy-1):min(height,iy+2),
                 max(0,ix-1):min(width,ix+2)]=100
    return grid
