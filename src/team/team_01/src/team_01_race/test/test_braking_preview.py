import math,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.braking_preview import BrakingPreview,BrakingPreviewConfig
from team_01_race.navigation import valid_path,pure_pursuit

def packet(stamp=1.,curve=True):
 x=np.arange(.1,8.01,.1);p=np.column_stack((x,.175*np.maximum(0,x-6.)**2 if curve else np.zeros(len(x))))
 return dict(stamp=stamp,frame_id='team_01/base_link',confidence=1.,path=p.tolist())

def test_far_curve_brakes_before_steering_path_sees_a_bend():
 b=BrakingPreview();d=packet();p=np.asarray(d['path']);near=p[p[:,0]<=5.]
 assert not valid_path(p) and valid_path(near)
 assert abs(pure_pursuit(near,7.,.34)[0])<1e-9
 assert b.update(d,1.,1.) and b.context(1.) is None
 assert b.update(packet(1.04),1.04,1.04)
 assert b.context(1.05)['speed_limit']==2.5

def test_replay_invalid_frame_and_expiration_cannot_extend_warning():
 b=BrakingPreview();b.update(packet(),1.,1.);b.update(packet(1.04),1.04,1.04)
 assert not b.update(packet(1.04),1.04,1.2)
 bad=packet(1.1);bad['path']=[];assert not b.update(bad,1.1,1.1)
 assert b.context(1.15) and b.context(1.3) is None

def test_straight_requires_two_new_frames_to_release_warning():
 b=BrakingPreview();b.update(packet(),1.,1.);b.update(packet(1.04),1.04,1.04)
 b.update(packet(1.08,False),1.08,1.08);assert b.context(1.08)
 b.update(packet(1.12,False),1.12,1.12);assert b.context(1.12) is None

def test_bad_source_does_not_grant_warning():
 for changed in ({'frame_id':'map'},{'confidence':.39},{'stamp':math.nan},{'stamp':2.}):
  b=BrakingPreview();d=packet();d.update(changed)
  assert not b.update(d,1.,1.) and b.context(1.) is None


def test_reliable_inferred_single_wall_can_lower_speed_without_steering_authority():
 b=BrakingPreview();d=packet();d['confidence']=.68
 assert b.update(d,1.,1.) and b.context(1.) is None
 d=packet(1.04);d['confidence']=.68;b.update(d,1.04,1.04)
 assert b.context(1.04)['speed_limit']==2.5


def test_twelve_metre_preview_sees_bend_beyond_old_eight_metre_window():
 x=np.arange(.1,12.01,.1)
 p=np.column_stack((x,.175*np.maximum(0,x-9.5)**2))
 b=BrakingPreview(BrakingPreviewConfig(horizon_m=12.))
 for stamp in (1.,1.04):
  assert b.update(dict(stamp=stamp,frame_id='team_01/base_link',confidence=.85,path=p.tolist()),stamp,stamp)
 assert b.context(1.04)['speed_limit']==2.5
 assert b.context(1.04)['horizon_m']==12.
 assert abs(pure_pursuit(p[p[:,0]<=5.],7.,.34)[0])<1e-9
