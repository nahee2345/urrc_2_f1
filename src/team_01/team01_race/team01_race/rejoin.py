"""Persistent passing and smooth rejoin to the preselected base lane."""
import math
import numpy as np


def smooth(u):
    u=max(0.,min(1.,float(u)))
    return u*u*u*(10.+u*(-15.+6.*u))


class RejoinPlanner:
    def __init__(self,cfg):
        self.cfg=cfg
        self.phase='CRUISE'
        self.side=0
        self.offset=0.
        self.start_offset=0.
        self.pass_offset=0.
        self.elapsed=0.
        self.clear_time=0.
        self.remaining_m=0.
        self.base_offset=0.

    def _side_safe(self,road,points,side,lead,offset):
        if not road.valid or lead is None:return False
        if road.bounds is not None:
            xs=road.bounds[:,0]; b=road.bounds
            x=min(float(xs[-1]),max(float(xs[0]),lead))
            middle=.5*(np.interp(x,xs,b[:,1])+np.interp(x,xs,b[:,2]))
            edge=np.interp(x,xs,b[:,2 if side>0 else 1])
            free=side*(edge-middle)
        else:free=road.half_width
        margin=self.cfg['path_half_width_m']+.18
        if free<abs(offset)+margin:return False
        cloud=np.asarray(points,dtype=float).reshape((-1,2))
        candidate=cloud[(cloud[:,0]>.7)&(cloud[:,0]<min(lead+2.5,11.))]
        return not np.any(np.abs(candidate[:,1]-side*abs(offset))<margin)

    def update(self,road,points,lead,lead_speed,rear_gap,dt,speed,
               enabled,base_offset=0.):
        dt=max(.01,min(.25,float(dt)))
        self.base_offset=float(base_offset)
        if not enabled or not road.valid:
            # Localization dropout while alongside another car must NOT snap
            # laterally back into that car. Hold the last safe offset; the
            # independent speed supervisor will enforce a lower speed.
            if self.phase!='CRUISE':
                self.phase='HOLD'
                return self.base_offset+self.offset,self.side
            self.offset=0.
            return self.base_offset,0
        if self.phase=='HOLD':
            # Reacquired sensors: regain a safe passing offset before merging.
            self.phase='PASS'
            self.clear_time=0.
        front_limit=max(3.5,min(10.,speed*.7+2.))
        if self.phase=='CRUISE':
            self.offset=0.
            if (lead is not None and 1.5<lead<front_limit and
                    rear_gap!=-1. and (lead_speed is None or lead_speed<speed-.5)):
                for side in (1,-1):
                    lateral=min(float(self.cfg.get('overtake_offset_m',.48)),
                                road.half_width-self.cfg['path_half_width_m']-.20)
                    if lateral>.35 and self._side_safe(road,points,side,lead,lateral):
                        self.side=side;self.phase='SHIFT';self.elapsed=0.
                        self.start_offset=0.;self.pass_offset=side*lateral
                        self.remaining_m=max(3.,speed*1.5)
                        break
        elif self.phase=='SHIFT':
            self.elapsed+=dt
            duration=max(1.0,min(2.5,self.remaining_m/max(.5,speed)))
            self.offset=self.start_offset+(self.pass_offset-self.start_offset)*smooth(self.elapsed/duration)
            if self.elapsed>=duration:
                self.offset=self.pass_offset;self.phase='PASS';self.clear_time=0.
        elif self.phase=='PASS':
            self.offset=self.pass_offset
            if lead is None and rear_gap!=-1. and (rear_gap is None or rear_gap>3.):
                self.clear_time+=dt
            else:self.clear_time=0.
            if self.clear_time>=.8:
                self.phase='MERGE';self.elapsed=0.;self.start_offset=self.offset
        elif self.phase=='MERGE':
            if lead is not None and lead<3.0:
                self.phase='PASS';self.offset=self.pass_offset
            elif rear_gap==-1. or (rear_gap is not None and rear_gap<2.5):
                # Do not cut in front of a vehicle whose rear gap is small.
                self.phase='PASS';self.offset=self.pass_offset;self.clear_time=0.
            else:
                self.elapsed+=dt
                duration=max(1.0,min(2.5,1.5/max(.5,speed/5.)))
                self.offset=self.start_offset*(1.-smooth(self.elapsed/duration))
                if self.elapsed>=duration:
                    self.phase='CRUISE';self.side=0;self.offset=0.
        return self.base_offset+self.offset,(self.side if self.phase!='CRUISE' else 0)
