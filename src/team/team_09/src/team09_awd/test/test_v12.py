import math
import numpy as np
from team09_awd.core import MAX_CURVATURE,rollout,steering_rate,reachable_curvature
from team09_awd.control import steer_step
from team09_awd.racing import plan_racing,route_command
from team09_awd.bypass import BypassState
from team09_awd.recovery import escape_plan

def test_high_speed_step_is_small():
 k=steer_step(0,MAX_CURVATURE,.05,12)
 assert abs(math.atan(.33*k))<=steering_rate(12)*.05+1e-9
 assert k<=3/144
 assert abs(k)<abs(steer_step(0,MAX_CURVATURE,.05,0))

def test_low_speed_retains_full_angle():
 k=0
 for _ in range(40):k=steer_step(k,MAX_CURVATURE,.05,.4)
 assert abs(k-MAX_CURVATURE)<1e-8

def test_rollout_uses_same_actuation():
 s,p=rollout(MAX_CURVATURE,0,12,2)
 assert abs(p[-1,2])<=reachable_curvature(MAX_CURVATURE,12)*s[-1]+1e-8

def test_centre_band_and_smooth_correction():
 assert route_command(np.array([0.,0.,.2]),0,0,0,12,0)==0
 assert abs(route_command(np.array([0.,0.,.5]),0,0,0,12,0))<.002

def test_bypass_fit_loss_does_not_latch_stop():
 x=np.linspace(-2,25,700);p=np.vstack([np.c_[x,x*0+1.2],np.c_[x,x*0-1.2]])
 state=BypassState();state.commit(1)
 r=plan_racing(p,0,0,bypass_state=state,stamp=1)
 assert r.route_kind!='BYPASS_WAIT'
 assert r.mode=='RUN'

def test_escape_parallel_side_contact():
 x=np.linspace(-2,4,300);p=np.vstack([np.c_[x,x*0+.145],np.c_[x,x*0-2.]])
 r=escape_plan(p,0,0)
 assert r is not None and r.curvature<0 and r.mode=='RECOVER_CRAWL'

def test_escape_cannot_drive_through_front_wall():
 y=np.linspace(-3,3,400);p=np.c_[y*0+.245,y]
 assert escape_plan(p,0,0) is None

def test_no_escape_at_speed_or_without_observation():
 assert escape_plan(np.zeros((0,2)),0,0) is None
 assert escape_plan(np.zeros((40,2)),0,2) is None


def test_front_contact_can_reverse_with_observed_rear_corridor():
 x=np.linspace(-4,0,300);y=np.linspace(-1.2,1.2,200)
 p=np.vstack([np.c_[x,x*0+1.2],np.c_[x,x*0-1.2],np.c_[y*0+.245,y]])
 r=escape_plan(p,0,0)
 assert r is not None and r.mode=='RECOVER_REVERSE'
 assert r.route_poses[-1,0]<0
