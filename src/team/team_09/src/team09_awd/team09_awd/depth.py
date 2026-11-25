"""Calibrated RGB-D road corridor. No planar projection used for depth pixels."""
import math
import cv2
import numpy as np
from .config import G, SETTINGS
from .kinematics import transform_points


def depth_meters(image,encoding):
    if encoding=='32FC1':d=np.asarray(image,dtype=float)
    elif encoding=='16UC1':d=np.asarray(image,dtype=float)*.001
    else:raise ValueError('Unsupported depth encoding: '+encoding)
    if d.ndim!=2:raise ValueError('Depth must be HxW')
    return np.where(np.isfinite(d)&(d>.25)&(d<30.),d,np.nan)


def project(uv,z,k):
    uv=np.asarray(uv,float);z=np.asarray(z,float);fx,fy,cx,cy=k
    # Optical z=forward, x=right, y=down -> Gazebo sensor x=forward,y=left,z=up.
    local=np.c_[z, -(uv[:,0]-cx)*z/fx, -(uv[:,1]-cy)*z/fy]
    return transform_points(local, SETTINGS["sensors"]["camera_pose"])



def fit_depth_route(route):
    if route is None or len(route)<7:return None
    q=np.asarray(route);q=q[np.isfinite(q).all(axis=1)&(q[:,0]>.6)&(q[:,0]<25)]
    if len(q)<7 or np.ptp(q[:,0])<3:return None
    q=q[np.argsort(q[:,0])];poly=np.polyfit(q[:,0],q[:,1],2)
    residual=np.sqrt(np.mean((np.polyval(poly,q[:,0])-q[:,1])**2))
    if residual>.12 or abs(poly[1])>.6:return None
    return poly,float(q[:,0].max()),float(np.min(q[:,2])),float(max(0,1-residual/.2))


def rgbd_preview(bgr,depth,k):
    if bgr.shape[:2]!=depth.shape:raise ValueError('Unaligned RGB/depth dimensions')
    h,w=depth.shape
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    gray=cv2.inRange(hsv,np.array([0,0,15],np.uint8),np.array([179,110,175],np.uint8))>0
    vv,uu=np.mgrid[0:h:4,0:w:4];uv=np.c_[uu.ravel(),vv.ravel()]
    z=depth[vv,uu].ravel();valid=np.isfinite(z);pts=project(uv[valid],z[valid],k)
    road_mask=gray[vv,uu].ravel()[valid]
    seed=pts[road_mask&(pts[:,0]>.5)&(pts[:,0]<6)&(abs(pts[:,1])<2)&(pts[:,2]<.20)]
    plane=None
    if len(seed)>30 and np.ptp(seed[:,0])>2 and np.ptp(seed[:,1])>.6:
        for _ in range(3):
            A=np.c_[seed[:,:2],np.ones(len(seed))];coef=np.linalg.lstsq(A,seed[:,2],rcond=None)[0]
            seed=seed[abs(A@coef-seed[:,2])<.035]
            if len(seed)<30:break
        if len(seed)>=30 and abs(coef[0])<.3 and abs(coef[1])<.2 and abs(coef[2]-G.ground_z)<.08:plane=coef
    above=pts[:,2]-G.ground_z if plane is None else pts[:,2]-(np.c_[pts[:,:2],np.ones(len(pts))]@plane)
    obstacles=pts[(above>.06)&(above<.8)&(pts[:,0]>.25)&(pts[:,0]<25),:2]
    # No valid ground => no road claim. Unknown scene cannot grant acceleration.
    if plane is None:return None,obstacles,0.
    rows=[];last=w/2
    for v in range(int(h*.92),int(h*.38),-4):
        line=gray[v]&np.isfinite(depth[v]);edge=np.diff(np.r_[False,line,False].astype(int))
        runs=[(l,r) for l,r in zip(np.flatnonzero(edge==1),np.flatnonzero(edge==-1)) if r-l>12 and l>1 and r<w-2]
        if not runs:continue
        l,r=min(runs,key=lambda pair:abs((pair[0]+pair[1])/2-last));last=(l+r)/2
        us=np.array([l+3,(l+r)//2,r-4]);zz=depth[v,us]
        xyz=project(np.c_[us,np.full(3,v)],zz,k)
        height=xyz[:,2]-np.c_[xyz[:,:2],np.ones(3)]@plane
        if not np.isfinite(xyz).all() or max(abs(height))>.07:continue
        width=abs(xyz[0,1]-xyz[2,1]);x=xyz[1,0]
        if .6<x<25 and .85<width<6 and np.ptp(xyz[:,0])<.8:rows.append([x,(xyz[0,1]+xyz[2,1])/2,width])
    route=np.array(rows) if len(rows)>=7 else None
    return route,obstacles,float(plane[0])
