"""Track envelope learned from SLAM map-frame poses; never from Gazebo world truth."""
import json
import math
import os
from pathlib import Path
import numpy as np


class TrackMemory:
    def __init__(self,cfg):
        self.cfg=cfg
        self.samples=[]
        self.complete=False
        self.start_pose=None
        self.last_pose=None
        self.total_m=0.
        self.reason='NO_SLAM_POSE'

    @staticmethod
    def angle_diff(a,b):
        return math.atan2(math.sin(a-b),math.cos(a-b))

    def observe(self,pose,road):
        if self.complete or pose is None or not road.valid or len(road.centers)<3:
            return False
        if not all(math.isfinite(float(v)) for v in pose):return False
        x,y,yaw=(float(v) for v in pose)
        if self.start_pose is None:self.start_pose=(x,y,yaw)
        if self.last_pose is not None:
            moved=math.hypot(x-self.last_pose[0],y-self.last_pose[1])
            if moved>4.:
                self.reason='SLAM_POSE_JUMP';return False
            if moved<.6:return False
            self.total_m+=moved
        self.last_pose=(x,y,yaw)
        # Sample road CENTER, not the ego path. The first lap need not follow
        # a perfect line, and transient opponents are excluded from the map.
        c=np.asarray(road.centers,dtype=float)
        ahead=min(2.,float(c[-1,0]))
        cy=float(np.interp(ahead,c[:,0],c[:,1]))
        wx=x+math.cos(yaw)*ahead-math.sin(yaw)*cy
        wy=y+math.sin(yaw)*ahead+math.cos(yaw)*cy
        w=2.*float(road.half_width)
        if w<.65 or w>5.5:return False
        self.samples.append((wx,wy,w))
        self.reason='COLLECTING'
        return True

    def try_close(self,pose):
        if self.complete:return True
        if pose is None or self.start_pose is None or len(self.samples)<55:
            self.reason='TOO_FEW_SAMPLES';return False
        x,y,yaw=pose
        sx,sy,syaw=self.start_pose
        if (self.total_m<65. or math.hypot(x-sx,y-sy)>1.8 or
                abs(self.angle_diff(yaw,syaw))>math.radians(32.)):
            self.reason='MAP_LOOP_NOT_GEOMETRICALLY_CLOSED';return False
        cloud=np.asarray(self.samples,dtype=float)
        gaps=np.linalg.norm(np.diff(cloud[:,:2],axis=0),axis=1)
        if len(gaps)==0 or np.percentile(gaps,95)>3.5:
            self.reason='MAP_HAS_GAPS';return False
        # Centerline smoothing: light periodic 3-point filter only. Tight
        # corners are kept, and live curb checks still override the map.
        self.samples=[tuple(p) for p in (.15*np.roll(cloud,1,axis=0)+
                                           .7*cloud+.15*np.roll(cloud,-1,axis=0))]
        self.complete=True
        self.reason='SLAM_MAP_PROVISIONAL_CLOSED'
        return True

    def target(self,pose,lookahead):
        if not self.complete or pose is None or len(self.samples)<55:return None
        x,y,yaw=pose
        cloud=np.asarray(self.samples,dtype=float)
        delta=cloud[:,:2]-np.asarray([x,y])
        idx=int(np.argmin(np.sum(delta*delta,axis=1)))
        if float(np.linalg.norm(delta[idx]))>2.0:
            self.reason='LOCALIZATION_MISMATCH';return None
        total=0.
        j=idx
        for _ in range(len(cloud)):
            k=(j+1)%len(cloud)
            total+=float(np.linalg.norm(cloud[k,:2]-cloud[j,:2]))
            j=k
            if total>=lookahead:break
        dx,dy=cloud[j,0]-x,cloud[j,1]-y
        local_x=math.cos(yaw)*dx+math.sin(yaw)*dy
        local_y=-math.sin(yaw)*dx+math.cos(yaw)*dy
        if local_x<.5:return None
        return float(local_x),float(local_y)

    def save(self):
        if not self.complete:return None
        directory=Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state')))/'urrc_team01'/'learning'
        directory.mkdir(parents=True,exist_ok=True)
        dest=directory/'slam_lap1_map.json'
        tmp=dest.with_suffix('.tmp')
        tmp.write_text(json.dumps({'status':self.reason,'samples_xyz_width':self.samples,
            'distance_m':self.total_m,'provenance':'SLAM map->base_link TF + live road boundaries',
            'note':'Candidate loop closure; verify RTAB-Map graph and official lap timing.'},indent=2),encoding='utf-8')
        tmp.replace(dest)
        return dest
