"""Decouple trajectory generation (10 Hz) from steering control (20 Hz).

A stored base_link path must be reprojected using ego motion between updates;
reusing unchanged local XYZ is not valid once the vehicle has moved.
"""
import math
import numpy as np


class ReferenceScheduler:
    def __init__(self,builder,cfg):
        self.builder=builder
        self.period=max(.075,float(cfg.get('reference_update_period_s',.1)))
        self.wheelbase=max(.1,float(cfg['wheel_base_m']))
        self.path=None
        self.elapsed=0.
        self.count=0
        self.offset=None
        self.source='EMPTY'

    def reset(self):
        self.path=None
        self.elapsed=0.
        self.offset=None
        self.source='EMPTY'

    def update(self,road,offset,speed,yaw_rate,steer,dt,map_target=None):
        dt=max(.005,min(.25,float(dt)))
        if not road.valid:
            self.reset()
            return None
        if self.path is not None:
            angle=float(yaw_rate) if yaw_rate is not None and math.isfinite(float(yaw_rate)) else (
                max(0.,float(speed))*math.tan(float(steer))/self.wheelbase)
            dyaw=max(-.16,min(.16,angle*dt))
            dx=max(0.,float(speed))*dt
            c,s=math.cos(dyaw),math.sin(dyaw)
            old=self.path
            xx=c*(old[:,0]-dx)+s*old[:,1]
            yy=-s*(old[:,0]-dx)+c*old[:,1]
            valid=(xx>.035)&np.isfinite(xx)&np.isfinite(yy)
            xx,yy=xx[valid],yy[valid]
            if len(xx)<5 or np.any(np.diff(xx)<=0):
                self.path=None
            else:
                self.path=np.vstack((np.array([[0.,0.]]),np.column_stack((xx,yy))))
        self.elapsed+=dt
        force=(self.path is None or self.offset is None or
               abs(float(offset)-self.offset)>.20)
        if force or self.elapsed>=self.period:
            created=self.builder.build(road,offset,speed,map_target)
            self.elapsed=0.
            self.offset=float(offset)
            if created is None:
                self.path=None
                self.source='NO_REFERENCE'
                return None
            if self.path is not None:
                # Blend the newly sensed road reference with the pose-compensated
                # previous path. Never use the cache to declare a road drivable.
                x=created[:,0]
                old_y=np.interp(x,self.path[:,0],self.path[:,1])
                created[:,1]=.5*created[:,1]+.5*old_y
                created[0,1]=0.
            self.path=created
            self.count+=1
            self.source='REFRESH'
        else:
            self.source='PREDICT'
        return self.path
