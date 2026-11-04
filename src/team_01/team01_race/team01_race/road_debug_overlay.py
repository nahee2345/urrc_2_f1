"""Visualize sensed border and reference in the real wide RGB camera frame.

This is diagnostic rendering only; it is NEVER used by the planner.
Uses local flat ground z~0; on banked/sloped terrain it is approximate.
"""
import math
import numpy as np


def ground_to_rgb(points,shape,*,hfov_deg=105.,pitch_deg=11.,
                  camera_x=.214,camera_z=.112):
    xy=np.asarray(points,float).reshape((-1,2))
    h,w=shape[:2]
    f=w/(2*math.tan(math.radians(hfov_deg)/2))
    pitch=math.radians(pitch_deg)
    xc=xy[:,0]-camera_x
    depth=math.cos(pitch)*xc+math.sin(pitch)*camera_z
    vertical=math.sin(pitch)*xc-math.cos(pitch)*camera_z
    valid=(depth>.15)&np.isfinite(depth)
    safe=np.maximum(depth,.15)
    u=np.rint((w-1)/2-f*xy[:,1]/safe).astype(int)
    v=np.rint((h-1)/2-f*vertical/safe).astype(int)
    valid &= (u>=0)&(u<w)&(v>=0)&(v<h)
    return np.column_stack((u,v)),valid


def overlay_camera(frame,node):
    import cv2
    out=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
    def curve(points,color,thickness=2):
        pix,mask=ground_to_rgb(points,out.shape,
            hfov_deg=float(node.p['road_rgb_hfov_deg']),
            pitch_deg=float(node.p['sensor_pitch_down_deg']),
            camera_z=float(node.p['sensor_mount_height_m']))
        p=pix[mask]
        if len(p)>1:
            cv2.polylines(out,[p.astype(np.int32).reshape((-1,1,2))],
                          False,color,thickness,cv2.LINE_AA)
    road=node.planner.road
    if road.valid and len(road.centers)>=4:
        if road.bounds is not None:
            curve(road.bounds[:,[0,1]],(40,210,250),2)  # yellow
            curve(road.bounds[:,[0,2]],(40,210,250),2)
        curve(road.centers,(0,220,90),2)  # green center
    reference=node.planner.reference_geometry
    if reference is not None and len(reference)>=4:
        curve(reference,(220,70,230),2)  # magenta target
    if len(node.colored_wall_xy):
        pix,valid=ground_to_rgb(node.colored_wall_xy,out.shape,
            hfov_deg=float(node.p['road_rgb_hfov_deg']))
        for u,v in pix[valid][::max(1,len(pix)//60)]:
            cv2.circle(out,(int(u),int(v)),2,(20,70,250),-1)
    cv2.putText(out,'Yellow: observed edge | Green: center | Magenta: plan',
                (8,19),cv2.FONT_HERSHEY_SIMPLEX,.38,(255,255,255),1,
                cv2.LINE_AA)
    return out
