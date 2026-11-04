"""Smooth acquisition of a newly detected road center or preferred lane.

All points are in the current vehicle base_link frame. This is not a global
map and does not grant permission to drive outside the sensed corridor.
"""
import numpy as np


def smooth5(u):
    z=np.clip(u,0.0,1.0)
    return z*z*z*(10.0+z*(-15.0+6.0*z))


def tangent_basis(u):
    z=np.clip(u,0.0,1.0)
    return z-6.0*z**3+8.0*z**4-3.0*z**5


class GradualReference:
    """Quintic Hermite join from current pose/yaw to a visible road path."""

    def __init__(self,cfg):
        self.cfg=cfg
        self.last_capture_m=0.0
        self.last_initial_error_m=0.0

    def build(self,road,offset,speed,map_target=None):
        if not road.valid or len(road.centers)<4:
            return None
        centers=np.asarray(road.centers,dtype=float)
        if not np.all(np.isfinite(centers)) or np.any(np.diff(centers[:,0])<=0):
            return None
        x,y=centers[:,0],centers[:,1]+float(offset)
        if len(y)>=5:
            yy=np.convolve(np.pad(y,(2,2),mode='edge'),
                           np.ones(5)/5.,mode='valid')
        else:
            yy=y.copy()
        n=min(5,len(x))
        slope=float(np.median(np.diff(yy[:n])/
                              np.maximum(.06,np.diff(x[:n]))))
        slope=float(np.clip(slope,-.9,.9))
        at_zero=float(yy[0]-slope*x[0])
        horizon=min(14.0,float(x[-1]))
        if horizon<1.5:
            return None
        xs=np.linspace(0.,horizon,max(45,int(horizon/.12)+1))
        base=np.interp(xs,x,yy)
        front=xs<x[0]
        base[front]=at_zero+slope*xs[front]
        # A rapid new corner often causes the heading controller to
        # overshoot the *inside* wall before recovering. Bias the target
        # <=10cm toward the outside of that measured turn, still inside
        # the current measured corridor; collision validation is unchanged.
        if len(x)>=5:
            first=np.gradient(yy,x)
            second=np.gradient(first,x)
            curve=second/(1.+first*first)**1.5
            magnitude=float(self.cfg.get('corner_center_bias_gain',.45))
            bound=float(self.cfg.get('corner_center_bias_max_m',.10))
            correction=-np.clip(curve*magnitude,-bound,bound)
            base+=np.interp(xs,x,correction,left=correction[0],
                            right=correction[-1])
        if map_target is not None:
            mx,my=(float(v) for v in map_target)
            if np.isfinite(mx) and np.isfinite(my) and x[0]<=mx<=x[-1]:
                correction=my-float(np.interp(mx,x,yy))
                # A map may only trim the live route, never replace its bounds.
                if abs(correction)<.25:
                    base+=correction*np.exp(-.5*((xs-mx)/2.5)**2)
        offset_at_ego=float(base[0])
        heading_at_ego=float((base[1]-base[0])/max(.01,xs[1]-xs[0]))
        capture=float(np.clip(
            3.+.5*max(0.,float(speed))+2.*abs(offset_at_ego),
            3.,float(self.cfg.get('path_capture_max_length_m',8.5))))
        if abs(float(road.heading_deg))>12.:
            capture*=.78
        elif abs(float(road.heading_deg))>6.:
            capture*=.88
        self.last_capture_m=capture
        self.last_initial_error_m=offset_at_ego
        u=xs/capture
        result=(base-offset_at_ego*(1.-smooth5(u))
                -heading_at_ego*capture*tangent_basis(u))
        result[0]=0.
        return np.column_stack((xs,result))
