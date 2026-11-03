"""Independent, offline trajectory verifier; NEVER imported by vehicle control.

Only an external QA process may read the original track centerline; the racing
controller uses live RGB/depth/lidar and never receives these reference points.
"""
import csv
import math
from pathlib import Path
import numpy as np


def load_reference(csv_path):
    with Path(csv_path).open(newline='', encoding='utf-8') as stream:
        rows=list(csv.DictReader(stream))
    if len(rows)<25:
        raise ValueError('reference track requires at least 25 points')
    points=np.asarray([(float(r['x_m']),float(r['y_m'])) for r in rows],dtype=float)
    widths=np.asarray([float(r['road_width_m']) for r in rows],dtype=float)
    if not np.isfinite(points).all() or not np.isfinite(widths).all():
        raise ValueError('reference contains non-finite values')
    if np.linalg.norm(points[0]-points[-1])<1e-6:
        points=points[:-1];widths=widths[:-1]
    if np.any(widths<.30):
        raise ValueError('invalid reference road width')
    return points,widths


class LapVerifier:
    """Monotonic, full-course progress with offroad and discontinuity guards."""
    def __init__(self, points, widths, edge_margin=.145):
        pts=np.asarray(points,dtype=float)
        ws=np.asarray(widths,dtype=float)
        if len(pts)<25 or pts.shape[1]!=2 or len(ws)!=len(pts):
            raise ValueError('bad reference shape')
        self.points=pts
        self.widths=ws
        self.seg=np.roll(pts,-1,axis=0)-pts
        self.seg_sq=np.sum(self.seg*self.seg,axis=1)
        if np.any(self.seg_sq<1e-10):raise ValueError('duplicate reference segments')
        self.seg_len=np.sqrt(self.seg_sq)
        self.start_len=np.insert(np.cumsum(self.seg_len),0,0.)[:-1]
        self.length=float(np.sum(self.seg_len))
        if self.length<30.:raise ValueError('reference unreasonably short')
        self.edge_margin=float(edge_margin)
        self.started=False
        self.first_xy=None
        self.first_s=None
        self.prev_s=None
        self.prev_xy=None
        self.unwrapped_s=0.
        self.travel_m=0.
        self.n=0
        self.offroad_frames=0
        self.max_offroad_m=0.
        self.backtrack_m=0.
        self.max_step_m=0.
        self.status='WAITING_FOR_ODOM'
        self.coverage=set()
        self.last_lateral_m=None
        self.last_width_m=None
        self.last_s=None

    def project(self,xy):
        p=np.asarray(xy,dtype=float)
        delta=p-self.points
        t=np.clip(np.sum(delta*self.seg,axis=1)/self.seg_sq,0.,1.)
        nearest=self.points+t[:,None]*self.seg
        d_sq=np.sum((nearest-p)**2,axis=1)
        idx=int(np.argmin(d_sq))
        signed=float(np.cross(self.seg[idx],p-nearest[idx])/self.seg_len[idx])
        s=float((self.start_len[idx]+t[idx]*self.seg_len[idx])%self.length)
        w=float(self.widths[idx]*(1-t[idx])+self.widths[(idx+1)%len(self.widths)]*t[idx])
        return s, signed, w, math.sqrt(float(d_sq[idx]))

    def observe(self,x,y):
        x=float(x);y=float(y)
        if not math.isfinite(x) or not math.isfinite(y):
            self.status='INVALID_ODOM';return self.status
        xy=np.array([x,y])
        s,lateral,width,off=self.project(xy)
        self.last_s=s;self.last_lateral_m=lateral;self.last_width_m=width
        self.n+=1
        if not self.started:
            self.started=True;self.first_xy=xy;self.first_s=s
            self.prev_s=s;self.prev_xy=xy;self.status='TRACKING'
            if off>max(.8,width/2):self.status='START_OFF_TRACK'
            return self.status
        delta=float(np.linalg.norm(xy-self.prev_xy))
        self.max_step_m=max(self.max_step_m,delta)
        if delta>3.0:
            self.status='POSE_JUMP';return self.status
        self.travel_m+=delta
        self.prev_xy=xy
        ds=(s-self.prev_s+self.length/2)%self.length-self.length/2
        self.prev_s=s
        if ds < -.20:self.backtrack_m+=-ds
        self.unwrapped_s+=ds
        # Width is measured from track reference (for independent QA only).
        outside=max(0.,off-(width/2-self.edge_margin))
        if outside>.025:
            self.offroad_frames+=1
            self.max_offroad_m=max(self.max_offroad_m,outside)
        # Discrete arc-length coverage; a small repeated loop cannot fake a lap.
        if off<=max(.3,width/2):
            self.coverage.add(int((s/self.length)*120)%120)
        if self.offroad_frames>=8:
            self.status='OFF_TRACK';return self.status
        if self.backtrack_m>max(5.,self.length*.06):
            self.status='WRONG_DIRECTION_OR_REVERSAL';return self.status
        if (self.unwrapped_s>=self.length-1.8 and
                self.travel_m>=self.length*.80 and
                len(self.coverage)>=100 and
                np.linalg.norm(xy-self.first_xy)<=3.0):
            self.status='GEOMETRIC_LAP_PASS'
            return self.status
        self.status='TRACKING'
        return self.status

    def report(self):
        return {
            'status':self.status,'reference_length_m':round(self.length,3),
            'progress_m':round(self.unwrapped_s,3),
            'distance_actual_odometry_m':round(self.travel_m,3),
            'coverage_bins_of_120':len(self.coverage),
            'max_offroad_m':round(self.max_offroad_m,3),
            'offroad_frames':self.offroad_frames,
            'backtrack_m':round(self.backtrack_m,3),
            'max_pose_step_m':round(self.max_step_m,3),
            'last_lateral_m':self.last_lateral_m,
            'last_width_m':self.last_width_m,
            'samples':self.n,
            'note':'Independent odometry/reference geometric check only; not a collision-contact or official lap timer certificate.',
            'controller_track_map_used':False,
        }
