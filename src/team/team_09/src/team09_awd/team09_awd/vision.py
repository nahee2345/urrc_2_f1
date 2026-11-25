"""Calibrated planar-road RGB preview; a simulator-specific colour prior, not ML."""
import math
import cv2
import numpy as np

from .config import G, SETTINGS
from .kinematics import rotation


def pixels_to_ground(uv,k):
    uv=np.asarray(uv,float);fx,fy,cx,cy=k
    pose=SETTINGS["sensors"]["camera_pose"]
    rays=np.c_[np.ones(len(uv)),-(uv[:,0]-cx)/fx,-(uv[:,1]-cy)/fy] @ rotation(pose[3:]).T
    t=np.where(rays[:,2]<-.005,(G.ground_z-pose[2])/np.minimum(rays[:,2],-.005),np.nan)
    return (np.asarray(pose[:3])+t[:,None]*rays)[:,:2]


def road_preview(bgr,k):
    h,w=bgr.shape[:2]
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([0,0,15],np.uint8),np.array([179,110,175],np.uint8))
    # Fill small white grid markings, without bridging large red/white walls.
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    candidates=[]
    for i in range(1,count):
        x,y,ww,hh,area=stats[i]
        if area>w*h*.02 and y+hh>h*.70 and x<w*.65 and x+ww>w*.35:candidates.append(i)
    if not candidates:return None,bgr.copy()
    best=max(candidates,key=lambda i:stats[i,cv2.CC_STAT_AREA]);road=labels==best
    rows=[];debug=bgr.copy();last=w/2
    for v in range(int(h*.95),int(h*.40),-3):
        row=road[v];edge=np.diff(np.r_[False,row,False].astype(int))
        runs=list(zip(np.flatnonzero(edge==1),np.flatnonzero(edge==-1)))
        runs=[(l,r) for l,r in runs if r-l>w*.025]
        if not runs:continue
        l,r=min(runs,key=lambda pair:abs((pair[0]+pair[1])/2-last))
        # Both sides must be visible to estimate the centre; reject clipped near-road rows.
        if l<2 or r>w-2:continue
        u=(l+r)/2;last=u
        ground=pixels_to_ground([[u,v],[l,v],[r,v]],k)
        if not np.isfinite(ground).all():continue
        x,y=ground[0];width=abs(ground[1,1]-ground[2,1])
        if .6<x<25 and .75<width<5.:
            rows.append([x,y]);cv2.circle(debug,(int(u),v),2,(0,255,0),-1)
    if len(rows)<6:return None,debug
    points=np.array(rows);points=points[np.argsort(points[:,0])]
    if np.ptp(points[:,0])<3.:return None,debug
    poly=np.polyfit(points[:,0],points[:,1],2)
    error=np.abs(np.polyval(poly,points[:,0])-points[:,1]);points=points[error<.20]
    if len(points)<6 or np.ptp(points[:,0])<3.:return None,debug
    return points,debug

# Historical regression test compatibility; values follow active calibration.
def __getattr__(name):
    pose=SETTINGS['sensors']['camera_pose']
    if name=='CAMERA_PITCH':return pose[4]
    if name=='CAMERA_HEIGHT':return pose[2]-G.ground_z
    if name=='CAMERA_X':return pose[0]
    raise AttributeError(name)
