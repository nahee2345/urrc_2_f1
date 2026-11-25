from .config import G, SETTINGS
"""Sensor-local curved corridor and preview velocity planner (original implementation).
No map, CSV centreline or simulator pose service is accessed by this module.
"""
import math
import numpy as np
from .core import Result,compact,compensate,footprint_clearance,plan_local,speed_limit,steering_rate,reachable_curvature


def fit_route(points):
    """Trace two boundaries through forward slabs, then fit a smooth quadratic.
    Ambiguous widths, obstacle-like discontinuities and sharp bends reject this model.
    The reactive swept-footprint planner remains the fallback.
    """
    p=compact(points);rows=[];centre=0.;last_width=None
    for x in np.arange(.5,30.1,.5):
        q=p[np.abs(p[:,0]-x)<(.40+.04*x)]
        if len(q)<(4 if x<5. else 2):
            if rows and x-rows[-1][0]>max(2.,.2*x):break
            continue
        left=q[(q[:,1]>centre+.30)&(q[:,1]<centre+4.)]
        right=q[(q[:,1]<centre-.30)&(q[:,1]>centre-4.)]
        minimum=2 if x<5. else 1
        if len(left)<minimum or len(right)<minimum:
            if rows and x-rows[-1][0]>max(2.,.2*x):break
            continue
        l=float(np.quantile(left[:,1],.25));rr=float(np.quantile(right[:,1],.75))
        width=l-rr;middle=(l+rr)/2
        if not .85<width<6. or (last_width is not None and abs(width-last_width)>.35):break
        if rows and abs(middle-centre)>.45:break
        rows.append((x,middle,width));centre=middle;last_width=width
    if len(rows)<7:return None
    a=np.array(rows)
    if a[-1,0]-a[0,0]<3.:return None
    weights=1./(1.+a[:,0]*.04)
    poly=np.polyfit(a[:,0],a[:,1],2,w=weights)
    residual=float(np.sqrt(np.mean((np.polyval(poly,a[:,0])-a[:,1])**2)))
    if residual>.10 or np.ptp(a[:,2])>.65 or abs(poly[1])>.55:return None
    return poly,float(a[-1,0]),float(np.min(a[:,2])),max(0.,1.-residual/.15)


class CurveTracker:
    def __init__(self):self.path=None;self.pose=None;self.stamp=None
    def update(self,fit,pose,stamp):
        if fit is None:self.path=None;return None
        poly,horizon,width,quality=fit
        if self.path is not None and self.stamp is not None and 0.<stamp-self.stamp<.2:
            old=compensate(self.path,self.pose,pose)
            old=old[(old[:,0]>.2)&(old[:,0]<horizon)]
            if len(old)>5 and np.ptp(old[:,0])>3.:
                old_poly=np.polyfit(old[:,0],old[:,1],2)
                probes=np.linspace(.5,min(horizon,float(old[:,0].max())),12)
                if max(abs(np.polyval(old_poly,probes)-np.polyval(poly,probes)))<.40:
                    alpha=(stamp-self.stamp)/(.12+stamp-self.stamp)
                    poly=(1-alpha)*old_poly+alpha*poly
        xs=np.linspace(0,horizon,60);self.path=np.c_[xs,np.polyval(poly,xs)]
        self.pose=pose;self.stamp=stamp
        return poly,horizon,width,quality


def combine_vision(poly,vision,horizon,width):
    """Vision can refine a shape, but cannot extend LiDAR's safe horizon."""
    if vision is None or len(vision)<6:return poly,False
    v=np.asarray(vision);v=v[(v[:,0]>.6)&(v[:,0]<min(horizon,25.))]
    if len(v)<6 or np.ptp(v[:,0])<3.:return poly,False
    vp=np.polyfit(v[:,0],v[:,1],2)
    probes=np.linspace(max(.7,float(v[:,0].min())),min(horizon,float(v[:,0].max())),12)
    error=np.abs(np.polyval(vp,probes)-np.polyval(poly,probes))
    if max(error)>min(.30,width*.15) or abs(vp[0]-poly[0])>.025:return poly,False
    mixed=poly.copy();mixed[:2]=.8*poly[:2]+.2*vp[:2]
    return mixed,True


def route_command(poly,x,y,yaw,speed,rate=None):
    slope=float(2*poly[0]*x+poly[1]);heading=math.atan(slope)-yaw
    heading=math.atan2(math.sin(heading),math.cos(heading))
    offset=(float(np.polyval(poly,x))-y)/math.sqrt(1+slope*slope)
    e=math.copysign(max(0.,abs(offset)-(.22 if speed>6. else .15)),offset)
    ff=float(2*poly[0]/(1+slope*slope)**1.5)
    lh=max(3.,min(8.,3.+.35*speed));lc=max(4.,min(14.,4.+.8*speed))
    # A short preview of the tangent starts a bend earlier without chasing its apex.
    preview=min(2.,max(0.,.22*speed))
    upcoming=math.atan(float(2*poly[0]*(x+preview)+poly[1]))-yaw
    heading+=.35*math.atan2(math.sin(upcoming-heading),math.cos(upcoming-heading))
    k=ff+2.*math.sin(heading)/lh+.45*e/(lc*lc)
    if rate is not None:k+=float(np.clip(.08*(speed*k-rate)/max(speed,3.),-.012,.012))
    return float(np.clip(k,-G.max_curvature,G.max_curvature))


def route_rollout(poly,previous,speed,horizon,yawrate=None):
    # Same rear-axle frame and 0.9rad/s steering slew as core and ROS actuator command.
    ds=.12;s=np.arange(0.,horizon+.001,ds)
    rear=np.array([-G.wheelbase/2,0.]);yaw=0.;delta=math.atan(G.wheelbase*previous)
    poses=[];curves=[]
    for _ in s:
        centre=rear+G.wheelbase/2*np.array([math.cos(yaw),math.sin(yaw)])
        poses.append([*centre,yaw]);curves.append(math.tan(delta)/G.wheelbase)
        goal=math.atan(G.wheelbase*reachable_curvature(route_command(poly,*centre,yaw,speed,yawrate),speed))
        delta+=float(np.clip(goal-delta,-steering_rate(speed)*ds/max(speed,.4),steering_rate(speed)*ds/max(speed,.4)))
        k=math.tan(delta)/G.wheelbase;dyaw=k*ds
        rear+=ds*np.array([math.cos(yaw+dyaw/2),math.sin(yaw+dyaw/2)])
        yaw+=dyaw;yawrate=speed*k
    return s,np.array(poses),np.array(curves)


def check_route(points,s,poses,margin=.06):
    # Dense rectangle sweep. Current overlap is tested with actual margin.
    p=compact(points);rel=p[None,:,:]-poses[:,None,:2]
    c=np.cos(poses[:,2,None]);sn=np.sin(poses[:,2,None])
    x=rel[:,:,0]*c+rel[:,:,1]*sn;y=-rel[:,:,0]*sn+rel[:,:,1]*c
    pad=np.full((len(s),1),margin+.075);pad[0]=margin
    blocked=((np.abs(x)<G.half_length+pad)&(np.abs(y)<G.half_width+pad)).any(axis=1)
    blocked|=np.abs(poses[:,2])>1.25
    hit=np.flatnonzero(blocked);index=int(hit[0]) if len(hit) else len(s)-1
    return float(s[index]),index


def preview_speed(s,curves,free,maximum,lateral=2.5,brake=4.,latency=.20):
    valid=s<=max(0.,free-.45)
    if sum(valid)<2:return 0.
    dist=s[valid];k=np.abs(curves[valid]);v=np.minimum(maximum,np.sqrt(lateral/np.maximum(k,.001)))
    v[-1]=0. # stop within observed safe horizon, even if the road seems to continue
    for i in range(len(v)-2,-1,-1):
        # Retain some grip for cornering instead of asking full longitudinal+ lateral grip.
        alat=min(lateral,float(v[i+1]*v[i+1]*k[i+1]))
        effective=max(.8,math.sqrt(max(0.,brake*brake-alat*alat)))
        v[i]=min(v[i],math.sqrt(v[i+1]**2+2*effective*(dist[i+1]-dist[i])))
    cap=speed_limit(0.,free,1.,maximum,lateral,brake,latency)
    return min(float(v[0]),cap)


def proximity_cap(points,k,speed,clearance,latency=.2,yawrate=None):
    """Distance alone is not closing risk. Check closest side's normal approach speed.
    Parallel travel at .30m no longer gets an unconditional 2.5m/s cap.
    """
    if clearance<SETTINGS["control"]["near_clearance_m"]:return SETTINGS["control"]["near_speed_mps"]
    if clearance<SETTINGS["control"]["close_clearance_m"]:return SETTINGS["control"]["close_speed_mps"]
    p=compact(points);side=p[(np.abs(p[:,0])<.55)&(np.abs(p[:,1])<1.5)]
    if not len(side):return float('inf')
    signs=np.sign(side[:,1]);yaw=speed*k if yawrate is None else yawrate
    approach=np.maximum(0.,signs*yaw*side[:,0])
    gaps=np.maximum(0.,np.abs(side[:,1])-G.half_width-.10)
    closing=approach*latency+.5*abs(speed*yaw)*latency*latency
    if np.any(closing>gaps):return 2.5
    return float('inf')


def plan_racing(points,previous=0.,speed=0.,camera_yaw=None,margin=.06,yawrate=0.,
                wall_state=None,vision=None,maximum=12.,lateral=2.5,brake=4.,latency=.2,tracker=None,pose=None,stamp=None,bypass_state=None):
    p=compact(points);fit=fit_route(p)
    if tracker is not None:fit=tracker.update(fit,pose,stamp)
    if fit is not None and footprint_clearance(p)>=.025:
        poly,horizon,width,quality=fit
        if bypass_state is not None:
            from .bypass import propose
            bypass=propose(p,fit,previous,speed,yawrate,margin,maximum,lateral,brake,latency,bypass_state,stamp)
            if bypass is not None:return bypass
        poly,vision_used=combine_vision(poly,vision,horizon,width)
        # Only use the portion supported by measured boundaries; no unseen extrapolation.
        s,poses,curves=route_rollout(poly,previous,speed,horizon,yawrate)
        free,index=check_route(p,s,poses,margin)
        if free>=3.:
            k=route_command(poly,0.,0.,0.,speed,yawrate)
            result=Result(k,free,min(1.,len(p)/180.),footprint_clearance(p),float(poly[2]),'RUN',math.atan(float(poly[1])),quality)
            result.route_poses=poses[:index+1];result.route_dist=s[:index+1]
            result.preview_limit=preview_speed(s,curves,free,maximum,lateral,brake,latency)
            result.route_kind='CURVED_LOCAL';result.vision_fused=vision_used
            result.road_curvature=float(2*poly[0]/(1+poly[1]**2)**1.5)
            result.proximity_limit=proximity_cap(p,k,speed,result.clearance,latency,yawrate)
            return result
    result=plan_local(p,previous,speed,camera_yaw,margin,yawrate=yawrate,wall_state=wall_state,filtered_walls=True)
    result.preview_limit=float('inf');result.route_kind='REACTIVE';result.vision_fused=False
    result.road_curvature=result.curvature
    result.proximity_limit=proximity_cap(p,result.curvature,speed,result.clearance,latency,yawrate)
    return result
