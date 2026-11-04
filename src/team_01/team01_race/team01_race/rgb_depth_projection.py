"""Project Depth pixel coordinates into a different-FOV, coaxial RGB camera.

The Gazebo TEAM01 road cameras have identical pose, so the pinhole formula is
valid. Calibrated extrinsic transforms are mandatory for different poses.
The result is a pixel sample; it never grants drivable-space authority.
"""
import math
import numpy as np


def depth_uv_in_rgb(uv, depth_shape, rgb_shape, depth_hfov_deg, rgb_hfov_deg):
    a=np.asarray(uv,dtype=float).reshape((-1,2))
    dh,dw=depth_shape[:2];rh,rw=rgb_shape[:2]
    if min(dh,dw,rh,rw)<8:
        raise ValueError('invalid camera geometry')
    if not (10.<depth_hfov_deg<170. and 10.<rgb_hfov_deg<170.):
        raise ValueError('invalid horizontal field of view')
    fd=dw/(2.*math.tan(math.radians(depth_hfov_deg)/2.))
    fr=rw/(2.*math.tan(math.radians(rgb_hfov_deg)/2.))
    ratio=fr/fd
    v=np.rint((a[:,0]-(dh-1)/2.)*ratio+(rh-1)/2.).astype(int)
    u=np.rint((a[:,1]-(dw-1)/2.)*ratio+(rw-1)/2.).astype(int)
    mapped=np.column_stack((v,u))
    valid=(v>=0)&(v<rh)&(u>=0)&(u<rw)
    return mapped,valid


def red_white_wall_points(obstacle_xy,obstacle_uv,obstacle_height,rgb,
                          depth_shape,depth_hfov_deg=92.,rgb_hfov_deg=105.):
    """Require a *raised Depth obstacle* co-located with a red/white RGB wall.

    This is independent evidence for a border; it is not an automatic route
    boundary because a red/white vehicle or advertisement may look similar.
    """
    xy=np.asarray(obstacle_xy,dtype=float).reshape((-1,2))
    uv=np.asarray(obstacle_uv,dtype=int).reshape((-1,2))
    heights=np.asarray(obstacle_height,dtype=float).reshape(-1)
    frame=np.asarray(rgb)
    empty=np.empty((0,2),dtype=float)
    if (frame.ndim!=3 or frame.shape[2]<3 or len(xy)!=len(uv)
            or len(heights)!=len(xy) or not len(xy)):
        return empty
    mapped,valid=depth_uv_in_rgb(uv,depth_shape,frame.shape,
                                 depth_hfov_deg,rgb_hfov_deg)
    valid &= (heights>.085)&(heights<.65)
    if not np.any(valid):return empty
    c=frame[mapped[valid,0],mapped[valid,1],:3].astype(float)
    red=(c[:,0]>115)&(c[:,0]>c[:,1]*1.45)&(c[:,0]>c[:,2]*1.45)
    white=(np.min(c,axis=1)>145)&(np.ptp(c,axis=1)<60)
    return xy[valid][red|white]
