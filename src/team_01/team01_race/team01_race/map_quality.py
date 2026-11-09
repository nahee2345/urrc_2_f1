"""Only verified local SLAM and measured road edges can authorize map guidance."""
import math
import numpy as np


def map_target_agrees(road,target,half_width):
    if not road.valid or target is None or road.bounds is None:return False
    x,y=map(float,target)
    b=np.asarray(road.bounds,dtype=float)
    if not all(map(math.isfinite,(x,y))) or len(b)<4:return False
    if x<b[0,0] or x>b[-1,0]:return False
    right=float(np.interp(x,b[:,0],b[:,1]))
    left=float(np.interp(x,b[:,0],b[:,2]))
    return right+half_width+.04<=y<=left-half_width-.04


class MapQuality:
    def __init__(self):
        self.odom_at=None
        self.info_at=None
        self.good=0
        self.last_xy=None
        self.closed=False
        self.reason='NO_SLAM_ODOM'

    def odom(self,stamp,cov,x,y):
        stamp=float(stamp)
        c=np.asarray(cov,dtype=float)
        good=(math.isfinite(stamp) and stamp>0 and len(c)==36 and
              np.all(np.isfinite(c)) and all(math.isfinite(float(v)) for v in (x,y)) and
              0.<c[0]<=.09 and 0.<c[7]<=.09 and 0.<c[35]<=.12 and
              (self.odom_at is None or stamp>self.odom_at))
        if good and self.odom_at is not None and self.last_xy is not None:
            elapsed=stamp-self.odom_at
            displacement=math.hypot(float(x)-self.last_xy[0],
                                    float(y)-self.last_xy[1])
            if displacement>.5+22.*elapsed:
                # A pose discontinuity invalidates earlier loop evidence.
                good=False
                self.closed=False
                self.info_at=None
        self.good=self.good+1 if good else 0
        self.odom_at=stamp if math.isfinite(stamp) and stamp>0 else None
        self.last_xy=(float(x),float(y)) if good else None

    def reset(self):
        self.__init__()

    def info(self,stamp,loop_id,proximity_id=0):
        stamp=float(stamp)
        if stamp>0 and math.isfinite(stamp):
            self.info_at=stamp
            # A nearby keyframe is not proof of successful global closure.
            # Only RTAB-Map graph loop closure unlocks learned-map guidance.
            if int(loop_id)>0:self.closed=True

    def ready(self,now,track_complete,pose_valid):
        if not track_complete or not pose_valid:
            self.reason='TRACK_OR_TF_UNVERIFIED'
        elif not self.closed:
            self.reason='NO_RTAB_LOOP_CLOSURE'
        elif (self.odom_at is None or self.good<3 or
              not 0.<=now-self.odom_at<=.75):
            self.reason='SLAM_ODOM_STALE_OR_UNCERTAIN'
        elif self.info_at is None or not 0.<=now-self.info_at<=2.5:
            self.reason='RTAB_INFO_STALE'
        else:
            self.reason='LOCALIZED_AND_LOOP_CLOSED'
        return self.reason=='LOCALIZED_AND_LOOP_CLOSED'

    def diag(self):
        return {'reason':self.reason,'loop_confirmed':self.closed,
                'valid_visual_odom_frames':self.good}
