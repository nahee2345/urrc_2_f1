import sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.speed_feedback import SpeedFeedback

def test_speed_norm_and_freshness():
 f=SpeedFeedback();assert f.update(1.,10.,.02,(3.,4.,0.))
 assert f.speed==5. and f.fresh(10.1) and not f.fresh(10.3)

def test_bad_stale_future_and_replay_never_renew_velocity():
 f=SpeedFeedback();assert f.update(1.,10.,0.,(4.,0.,0.))
 for stamp,age,v in [(1.,0.,(0.,0.,0.)),(.9,0.,(0.,0.,0.)),(2.,.3,(0.,0.,0.)),(2.,-.1,(0.,0.,0.)),(2.,0.,(math.nan,0.,0.))]:
  assert not f.update(stamp,10.2,age,v)
 assert f.speed==4. and not f.fresh(10.3)
