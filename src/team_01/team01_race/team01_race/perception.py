"""Sensor-only geometry. No track coordinates, poses, or opponent topics used."""
from dataclasses import dataclass, field
import math
import numpy as np


@dataclass
class DepthScene:
    obstacles: np.ndarray  # x forward, y left, metres in base frame
    slope_deg: float
    ground_count: int
    visible_count: int
    valid: bool
    ground_xy: np.ndarray = field(default_factory=lambda: np.empty((0,2)))
    ground_uv: np.ndarray = field(default_factory=lambda: np.empty((0,2),dtype=int))
    obstacle_uv: np.ndarray = field(default_factory=lambda: np.empty((0,2),dtype=int))
    obstacle_height: np.ndarray = field(default_factory=lambda: np.empty(0,dtype=float))


def decode_depth(msg):
    """Respect ROS Image stride and endian; reject unknown depth encodings."""
    if msg.encoding.upper() == '32FC1':
        dtype = '>f4' if msg.is_bigendian else '<f4'
        scale = 1.0
        size = 4
    elif msg.encoding.upper() == '16UC1':
        dtype = '>u2' if msg.is_bigendian else '<u2'
        scale = 0.001
        size = 2
    else:
        raise ValueError('Unsupported depth encoding: ' + msg.encoding)
    if msg.step < msg.width * size or len(msg.data) < msg.height * msg.step:
        raise ValueError('Truncated depth image')
    data = np.ndarray((msg.height, msg.width), dtype=np.dtype(dtype),
                      buffer=bytes(msg.data), strides=(msg.step, size))
    return data.astype(np.float32) * scale


def depth_geometry(depth, horizontal_fov_deg=92., pitch_deg=11., mount_x=.214,
                   mount_z=.112, step=5, min_road_points=45):
    """Fit drivable road plane, then remove it from the obstacle cloud.

    Positive slopes are rises relative to the body-mounted camera; a continuous
    grade is deliberately *not* an obstacle. This estimates geometric free
    space, not semantic asphalt/grass; unknown or poor fits fail closed.
    """
    a = np.asarray(depth, dtype=np.float32)
    if a.ndim != 2 or min(a.shape) < 8:
        return DepthScene(np.empty((0, 2)), 0., 0, 0, False)
    h, w = a.shape
    rows, cols = np.mgrid[step//2:h:step, step//2:w:step]
    d = a[step//2:h:step, step//2:w:step].ravel()
    u = cols.ravel(); v = rows.ravel()
    fx = w / (2 * np.tan(np.deg2rad(horizontal_fov_deg)/2))
    fy = fx
    with np.errstate(invalid="ignore", over="ignore"):
        ylocal = -(u - (w-1)/2) * d / fx
        zlocal = -(v - (h-1)/2) * d / fy
        p = np.deg2rad(pitch_deg)
        x = mount_x + np.cos(p)*d + np.sin(p)*zlocal
        y = ylocal
        z = mount_z - np.sin(p)*d + np.cos(p)*zlocal
    valid = (np.isfinite(d) & (d >= .12) & (d < 13.8) & (x > .25)
             & (x < 12.5) & (np.abs(y) < 4.2) & np.isfinite(z)
             & (z > -.9) & (z < .75))
    x, y, z = x[valid], y[valid], z[valid]
    sampled_u, sampled_v = u[valid], v[valid]
    count = len(x)
    if count < min_road_points:
        return DepthScene(np.empty((0,2)), 0., 0, count, False)
    # Use broad neighbourhood; enough points across the road are required.
    candidates = np.where((x < 9.) & (np.abs(y) < 2.) & (z > -.65) & (z < .16))[0]
    if len(candidates) < min_road_points:
        return DepthScene(np.empty((0,2)), 0., 0, count, False)
    rng = np.random.default_rng(101)  # deterministic for repeatable tests
    xyz = np.stack((x[candidates], y[candidates], z[candidates]), axis=1)
    best = np.empty(0, dtype=np.int64)
    best_coef = None
    # RANSAC prevents one vehicle or wall occupying the entire road fit.
    for _ in range(60):
        ids = rng.choice(len(xyz), 3, replace=False)
        matrix = np.column_stack((xyz[ids,0], xyz[ids,1], np.ones(3)))
        if abs(np.linalg.det(matrix)) < .004:
            continue
        try: coef = np.linalg.solve(matrix, xyz[ids,2])
        except np.linalg.LinAlgError: continue
        if max(abs(coef[0]), abs(coef[1])) > .43:
            continue
        resid = np.abs(xyz[:,2] - (coef[0]*xyz[:,0]+coef[1]*xyz[:,1]+coef[2]))
        inside = np.flatnonzero(resid < .048)
        if len(inside) > len(best): best, best_coef = inside, coef
    if best_coef is None or len(best) < min_road_points:
        return DepthScene(np.empty((0,2)), 0., len(best), count, False)
    inp = xyz[best]
    coef = np.linalg.lstsq(np.column_stack((inp[:,0],inp[:,1],np.ones(len(inp)))),
                           inp[:,2], rcond=None)[0]
    resid = z - (coef[0]*x + coef[1]*y + coef[2])
    # Obstacles above road or drop-offs below it. Road grades are subtracted.
    obs = ((resid > .068) & (resid < .60)) | ((resid < -.12) & (resid > -.65))
    # Remove completely far points; retain enough features to see barriers.
    points = np.column_stack((x[obs], y[obs]))
    ground=np.abs(resid)<.065
    return DepthScene(points, float(np.rad2deg(np.arctan(coef[0]))),
                      len(best), count, True,
                      np.column_stack((x[ground],y[ground])),
                      np.column_stack((sampled_v[ground],sampled_u[ground])).astype(int),
                      np.column_stack((sampled_v[obs],sampled_u[obs])).astype(int),
                      np.asarray(resid[obs],dtype=float))


def laser_to_xy(ranges, angle_min, angle_increment, low=.14, high=15.8,
                sensor_x=.187, heading=0.):
    ranges = np.asarray(ranges, dtype=np.float32)
    idx = np.arange(len(ranges))
    ang = angle_min + angle_increment * idx + heading
    good = np.isfinite(ranges) & (ranges > low) & (ranges < high)
    r, a = ranges[good], ang[good]
    return np.column_stack((sensor_x + r*np.cos(a), r*np.sin(a)))


def red_signal_features(image, min_area=3):
    """Count luminous red circles in a horizontal bank and return their energy.

    The energy is deliberately kept alongside the count: a static dark-red
    gantry can still look red in a rendered image, so 5 blobs is NOT proof
    that all five lamps are currently illuminated.
    """
    import cv2
    if image is None or image.ndim != 3 or image.shape[2] < 3:
        return 0, 0.0
    frame=image[:int(image.shape[0]*.82),:,:3]
    rgb=frame.astype(np.int16)
    red=rgb[:,:,0]
    dominance=red-np.maximum(rgb[:,:,1],rgb[:,:,2])
    mask=((red>145) & (dominance>75) & (rgb[:,:,1]<160)).astype(np.uint8)
    n,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    blobs=[]
    for i in range(1,n):
        x,y,w,h,area=[int(v) for v in stats[i]]
        if min_area<=area<=10000 and .5<=w/max(h,1)<=2.5:
            energy=float(np.sum(dominance[labels==i]))
            blobs.append((x+w/2.,y+h/2.,w,h,energy))
    if not blobs:
        return 0,0.0
    # Prefer the largest horizontal bank; ties choose the brighter bank.
    banks=[]
    for bx,by,bw,bh,_ in blobs:
        bank=[b for b in blobs if abs(b[1]-by)<=max(20.,bh*3.,b[3]*3.)]
        banks.append((len(bank),sum(b[4] for b in bank)))
    return max(banks)


def red_blob_count(image, min_area=3):
    return red_signal_features(image,min_area)[0]


def decode_rgb(msg):
    if msg.encoding.lower() in ('rgb8','bgr8'): chans=3
    elif msg.encoding.lower() in ('rgba8','bgra8'): chans=4
    else:raise ValueError('Unsupported RGB encoding: '+msg.encoding)
    if msg.step < msg.width*chans or len(msg.data) < msg.height*msg.step:
        raise ValueError('Truncated rgb image')
    frame=np.ndarray((msg.height,msg.width,chans), dtype=np.uint8,
                     buffer=bytes(msg.data),strides=(msg.step,chans,1))[:,:,:3]
    if msg.encoding.lower().startswith('bgr'):
        return frame[:,:,::-1].copy()
    return frame.copy()
