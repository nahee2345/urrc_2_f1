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
        """Collision-check the moving lateral path, not a static side stripe."""
        if (road is None or not road.valid or road.bounds is None or
                side not in (-1,1)):return False
        target=side*abs(float(offset))
        ahead=float(lead) if lead is not None else 5.
        if not math.isfinite(ahead) or ahead<1.5:return False
        b=np.asarray(road.bounds,dtype=float)
        if b.ndim!=2 or len(b)<4 or b.shape[1]!=3:return False
        if b[0,0]>.75 or b[-1,0]<ahead+1.:return False
        margin=float(self.cfg['path_half_width_m'])+.18
        shift_end=max(1.0,ahead-1.)
        xs=np.linspace(.75,min(ahead+2.5,10.),40)
        lateral=target*np.array([smooth(x/shift_end) for x in xs])
        right=np.interp(xs,b[:,0],b[:,1])
        left=np.interp(xs,b[:,0],b[:,2])
        if np.any(lateral-margin<right) or np.any(lateral+margin>left):
            return False
        pts=np.asarray(points,dtype=float).reshape((-1,2))
        if not len(pts):return True
        pts=pts[np.all(np.isfinite(pts),axis=1)]
        pts=pts[(pts[:,0]>=xs[0])&(pts[:,0]<=xs[-1])]
        if not len(pts):return True
        planned=np.interp(pts[:,0],xs,lateral)
        return not np.any(abs(pts[:,1]-planned)<margin)

    def _merge_safe(self,road,points):
        """Reject merge if live points occupy the return-to-lane region."""
        if road.bounds is None or not road.valid:return False
        cloud=np.asarray(points,dtype=float).reshape((-1,2))
        cloud=cloud[np.all(np.isfinite(cloud),axis=1)]
        cloud=cloud[(cloud[:,0]>.75)&(cloud[:,0]<4.)]
        margin=float(self.cfg['path_half_width_m'])+.18
        return not np.any(np.abs(cloud[:,1]-self.base_offset)<margin)

    def update(self,road,points,lead,lead_speed,rear_gap,dt,speed,
               enabled,base_offset=0.):
        dt=max(0.,min(.25,float(dt)))
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
            # Rejoin only when the chosen side is still physically observed clear.
            if not self._side_safe(road,points,self.side,lead,
                                   abs(self.pass_offset)):
                return self.base_offset+self.offset,self.side
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
            if not self._side_safe(road,points,self.side,lead,
                                   abs(self.pass_offset)):
                self.phase='HOLD'
                return self.base_offset+self.offset,self.side
            self.elapsed+=dt
            duration=max(1.0,min(2.5,self.remaining_m/max(.5,speed)))
            self.offset=self.start_offset+(self.pass_offset-self.start_offset)*smooth(self.elapsed/duration)
            if self.elapsed>=duration:
                self.offset=self.pass_offset;self.phase='PASS';self.clear_time=0.
        elif self.phase=='PASS':
            if not self._side_safe(road,points,self.side,lead,
                                   abs(self.pass_offset)):
                self.phase='HOLD'
                return self.base_offset+self.offset,self.side
            self.offset=self.pass_offset
            if lead is None and rear_gap!=-1. and (rear_gap is None or rear_gap>3.):
                self.clear_time+=dt
            else:self.clear_time=0.
            if self.clear_time>=.8:
                self.phase='MERGE';self.elapsed=0.;self.start_offset=self.offset
        elif self.phase=='MERGE':
            if not self._merge_safe(road,points):
                self.phase='HOLD'
                return self.base_offset+self.offset,self.side
            if not self._side_safe(road,points,self.side,lead,
                                   abs(self.pass_offset)):
                self.phase='HOLD'
                return self.base_offset+self.offset,self.side
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
