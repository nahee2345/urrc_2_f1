"""Align a wider RGB camera onto the co-located depth camera optical rays."""
import math
import numpy as np


def camera_focal(width,hfov):
    if width<=0 or not (0.<hfov<math.pi):
        raise ValueError('invalid RGB-D camera geometry')
    return float(width)/(2.*math.tan(float(hfov)/2.))


def wide_rgb_to_depth(rgb,depth_width,depth_height,
                      rgb_fov=1.832596,depth_fov=1.6057):
    """Bilinearly sample RGB on depth-image optical rays; no geometry guessing."""
    import cv2
    if rgb.ndim!=3 or rgb.shape[2]!=3 or depth_width<1 or depth_height<1:
        raise ValueError('RGB-D alignment shape error')
    rh,rw=rgb.shape[:2]
    fr=camera_focal(rw,rgb_fov)
    fd=camera_focal(depth_width,depth_fov)
    u,v=np.meshgrid(np.arange(depth_width,dtype=np.float32),
                    np.arange(depth_height,dtype=np.float32))
    cxr=(rw-1.)/2.;cyr=(rh-1.)/2.
    cxd=(depth_width-1.)/2.;cyd=(depth_height-1.)/2.
    scale=fr/fd
    mx=(u-cxd)*scale+cxr
    my=(v-cyd)*scale+cyr
    return cv2.remap(rgb,mx,my,cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT,borderValue=(0,0,0))


def paired_stamp(rgb_stamp,depth_stamp,max_skew=.06):
    return abs(float(rgb_stamp)-float(depth_stamp))<=float(max_skew)
