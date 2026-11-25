import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.path_manager import lidar_corridor,select_geometry

def corridor(conf=.8):
 x=np.arange(.1,5.01,.1);p=np.column_stack((x,np.zeros(len(x))))
 return dict(path=p,left=p+[0,1.2],right=p-[0,1.2],confidence=conf,observed=(True,True),light='LIGHTS_OUT',reason='test')

def test_priority_fusion_and_disagreement():
 c,l=corridor(1.),corridor();assert select_geometry(c,l)[1]=='CAMERA_PATH'
 c['confidence']=.68;assert select_geometry(c,l)[1]=='FUSED_PATH'
 c['path'][:,1]+=.5;assert select_geometry(c,l)[1]=='LIDAR_PATH'
 assert select_geometry(None,l)[1]=='LIDAR_PATH'
 assert select_geometry(None,None)==(None,'NO_PATH')

def test_measured_lidar_walls_and_compact_obstacle_not_used_as_wall():
 x=np.arange(.2,5.,.025);left=np.column_stack((x,np.full(len(x),1.2)));right=np.column_stack((x[::-1],np.full(len(x),-1.2)))
 points=np.vstack((left,right));idx=np.r_[np.arange(len(x)),np.arange(len(x))+len(x)+10]
 d=lidar_corridor(points,idx)
 assert d is not None and np.max(abs(d['path'][:,1]))<1e-9
 assert lidar_corridor(np.array([[1.,.5],[1.05,.5],[1.1,.5]]),np.arange(3)) is None
 assert lidar_corridor(left,np.arange(len(x))) is None


def test_delayed_fresh_camera_is_held_then_expired_camera_falls_back_immediately():
 from team_01_race.path_manager import SourceSelector
 c,l=corridor(1.),corridor();s=SourceSelector()
 for t in (1.,1.05,1.10):s.select(c,l,dict(camera=t,lidar=t))
 assert s.source=='CAMERA_PATH'
 assert s.select(c,l,dict(camera=1.10,lidar=1.30))[1]=='CAMERA_PATH'
 assert s.select(c,l,dict(camera=1.10,lidar=1.40))[1]=='LIDAR_PATH'
 # Promotion needs hold time, agreement and three DISTINCT Camera frames.
 assert s.select(c,l,dict(camera=1.41,lidar=1.41))[1]=='LIDAR_PATH'
 for t in (1.71,1.76):assert s.select(c,l,dict(camera=t,lidar=t))[1]=='LIDAR_PATH'
 for _ in range(3):assert s.select(c,l,dict(camera=1.76,lidar=1.76))[1]=='LIDAR_PATH'
 assert s.select(c,l,dict(camera=1.81,lidar=1.81))[1]=='LIDAR_PATH'
 assert s.select(c,l,dict(camera=1.91,lidar=1.91))[1]=='CAMERA_PATH'


def test_invalid_current_source_does_not_delay_valid_safety_fallback():
 from team_01_race.path_manager import SourceSelector
 c,l=corridor(1.),corridor();s=SourceSelector()
 for t in (1.,1.04,1.08):s.select(c,None,dict(camera=t,lidar=None))
 assert s.source=='CAMERA_PATH'
 assert s.select(None,l,dict(camera=None,lidar=1.09))[1]=='LIDAR_PATH'
 assert s.select(None,None,dict(camera=None,lidar=None),now=1.10)[0] is None
 # No existing authority at startup requires confirmation again.
 assert s.select(None,l,dict(camera=None,lidar=1.16))[0] is None


def test_single_weak_frame_does_not_change_camera_identity():
 from team_01_race.path_manager import SourceSelector
 c,l=corridor(1.),corridor();s=SourceSelector()
 for t in (1.,1.05,1.1):s.select(c,l,dict(camera=t,lidar=t))
 weak=corridor(.68)
 assert s.select(weak,l,dict(camera=1.15,lidar=1.15))[1]=='CAMERA_PATH'
 assert s.reason=='transient_camera_degradation'
 assert s.select(c,l,dict(camera=1.2,lidar=1.2))[1]=='CAMERA_PATH'


def test_camera_recovery_disagreement_does_not_promote():
 from team_01_race.path_manager import SourceSelector
 c,l=corridor(1.),corridor();s=SourceSelector()
 for t in (1.,1.05,1.1):s.select(None,l,dict(camera=None,lidar=t))
 c['path'][:,1]+=.5;c['left'][:,1]+=.5;c['right'][:,1]+=.5
 for t in (1.5,1.55,1.6,1.65):assert s.select(c,l,dict(camera=t,lidar=t))[1]=='LIDAR_PATH'
 assert s.reason=='camera_recovery_disagreement'


def test_curvature_agreement_handles_different_arc_lengths():
 from team_01_race.path_manager import geometry_difference
 c,l=corridor(1.),corridor(.9)
 c['path'][:,1]=.008*c['path'][:,0]**2
 diff,_=geometry_difference(c,l)
 assert diff is not None and np.isfinite(diff['curvature'])


def test_burst_recovery_frames_do_not_immediately_promote_camera():
 from team_01_race.path_manager import SourceSelector
 c,l=corridor(1.),corridor();s=SourceSelector()
 for t in (1.,1.05,1.1):s.select(None,l,dict(camera=None,lidar=t),now=t)
 # Three distinct Camera stamps delivered in one callback burst.
 for stamp in (1.45,1.50,1.55):assert s.select(c,l,dict(camera=stamp,lidar=1.55),now=1.56)[1]=='LIDAR_PATH'
 assert s.select(c,l,dict(camera=1.6,lidar=1.6),now=1.65)[1]=='LIDAR_PATH'
 assert s.select(c,l,dict(camera=1.8,lidar=1.8),now=1.80)[1]=='CAMERA_PATH'
