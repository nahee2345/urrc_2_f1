"""Current-sensor-only drivable corridor with ego-motion temporal filtering."""
import math
import numpy as np
from team01_race.planning_road import RoadCorridor, fuse_verified_corridors


class RoadBelief:
    def __init__(self):
        self.previous=None
        self.source='none'
        self.innovation_m=None
        self.disagreement_m=None
        self.smoothed=False
        self.rejected=False
        self.reason='NONE'

    @staticmethod
    def _transform(prior,dx,yaw):
        c,s=math.cos(yaw),math.sin(yaw)
        p=np.asarray(prior.centers,float)
        x=c*(p[:,0]-dx)+s*p[:,1]
        y=-s*(p[:,0]-dx)+c*p[:,1]
        use=np.isfinite(x)&np.isfinite(y)&(x>=0)
        if use.sum()<4:return None
        x=x[use];y=y[use]
        if np.any(np.diff(x)<=0):return None
        return np.column_stack((x,y))

    @staticmethod
    def _difference(a,b):
        if a is None or b is None:return float('inf')
        lo=max(float(a[0,0]),float(b[0,0]),.75)
        hi=min(float(a[-1,0]),float(b[-1,0]),5.)
        if hi-lo<1.:return float('inf')
        xs=np.linspace(lo+.10,hi-.10,7)
        d=np.interp(xs,a[:,0],a[:,1])-np.interp(xs,b[:,0],b[:,1])
        return float(np.median(abs(d)))

    def update(self,visual,lidar,*,speed=0.,yaw_rate=None,dt=.05):
        self.innovation_m=None
        self.disagreement_m=None
        self.smoothed=False
        self.rejected=False
        self.reason='NONE'
        v=visual if visual is not None and visual.valid else None
        l=lidar if lidar is not None and lidar.valid else None
        merged=fuse_verified_corridors(v,l)
        dt=max(.005,min(.20,float(dt)))
        dx=max(0.,float(speed))*dt
        dyaw=(float(yaw_rate)*dt if yaw_rate is not None and
               math.isfinite(float(yaw_rate)) else 0.)
        old=self._transform(self.previous,dx,dyaw) if self.previous is not None else None
        if v is not None and l is not None:
            difference=self._difference(v.centers,l.centers)
            if math.isfinite(difference):self.disagreement_m=difference
        if merged is not None:
            chosen=merged;name='rgb_depth+lidar'
        elif v is not None and l is not None:
            vd=self._difference(v.centers,old)
            ld=self._difference(l.centers,old)
            # If both currently measured corridors contradict the previous
            # pose-compensated road AND each other, neither one is trusted.
            # A real corner reported by both sensors is accepted normally.
            conflict=(self.disagreement_m is not None and
                      self.disagreement_m>.35)
            if old is not None and conflict and vd>.35 and ld>.35:
                self.rejected=True
                self.reason='SENSOR_CONFLICT_NO_CONSENSUS'
                self.source='unverified_sensor_conflict'
                self.previous=None
                return RoadCorridor(np.empty((0,2)),0.,False,0.)
            if ld<.24 and ld+.14<vd:
                chosen=l;name='lidar_temporal_consensus'
            else:
                chosen=v;name='rgb_depth'
        elif v is not None:
            chosen=v;name='rgb_depth'
        elif l is not None:
            chosen=l;name='lidar'
        else:
            self.previous=None
            self.source='none'
            return RoadCorridor(np.empty((0,2)),0.,False,0.)
        centers=np.array(chosen.centers,dtype=float,copy=True)
        innovation=self._difference(centers,old)
        if math.isfinite(innovation):self.innovation_m=innovation
        if old is not None and innovation<.23:
            xs=centers[:,0]
            overlap=(xs>=old[0,0])&(xs<=old[-1,0])
            if np.any(overlap):
                estimate=np.interp(xs[overlap],old[:,0],old[:,1])
                centers[overlap,1]=.72*centers[overlap,1]+.28*estimate
                if chosen.bounds is not None:
                    b=np.asarray(chosen.bounds,dtype=float)
                    minimum=np.interp(xs[overlap],b[:,0],b[:,1])+.18
                    maximum=np.interp(xs[overlap],b[:,0],b[:,2])-.18
                    centers[overlap,1]=np.clip(centers[overlap,1],
                                               minimum,maximum)
                self.smoothed=True
        slopes=np.diff(centers[:,1])/np.maximum(.05,np.diff(centers[:,0]))
        angle=math.degrees(math.atan(float(np.median(slopes))))
        result=RoadCorridor(centers,chosen.half_width,True,angle,
                            chosen.bounds)
        # Historical center is only a filter. Current physical bounds are
        # always used by the collision planner.
        self.previous=result
        self.source=name
        return result
