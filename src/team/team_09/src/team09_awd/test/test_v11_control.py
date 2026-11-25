import unittest,math
import numpy as np
from team09_awd.control import steer_step,govern_speed,CommandWatchdog
from team09_awd.core import Result,MAX_CURVATURE,compact
from team09_awd.racing import combine_vision,plan_racing,CurveTracker
from team09_awd.bypass import BypassState,propose
class ControlTests(unittest.TestCase):
 def test_steering_rate_and_reversal(self):
  k=steer_step(0,MAX_CURVATURE,.05)
  self.assertAlmostEqual(math.atan(.33*k),.045)
  self.assertLess(steer_step(.01,-.1,.05),0) # no three-frame direction delay
 def test_full_steering_available(self):
  k=0
  for _ in range(30):k=steer_step(k,MAX_CURVATURE,.05)
  self.assertAlmostEqual(k,MAX_CURVATURE)
 def test_high_speed_corner_brakes_without_clipping_target_steer(self):
  r=Result(.3,20,1,1,0,'RUN');r.route_kind='CURVED_LOCAL'
  v,why=govern_speed(r,10,.3,10,3,.05,12,2.5,4,1.5,.2,'RUN')
  self.assertLess(v,4)
 def test_no_camera_only_speed_cap(self):
  r=Result(0,48,1,1,0,'RUN');r.route_kind='CURVED_LOCAL';r.proximity_limit=99
  v,_=govern_speed(r,8,0,8,0,.05,12,2.5,4,1.5,.2,'RUN')
  self.assertGreater(v,8)
 def test_bad_depth_shape_rejected(self):
  x=np.linspace(1,15,30);p=np.zeros(3)
  q,used=combine_vision(p,np.c_[x,x*0+1],20,2.4)
  self.assertFalse(used);np.testing.assert_array_equal(q,p)
 def test_depth_influence_bounded(self):
  x=np.linspace(1,15,30);p=np.zeros(3)
  q,used=combine_vision(p,np.c_[x,.01*x],20,2.4)
  self.assertTrue(used);self.assertAlmostEqual(q[1],.002);self.assertEqual(q[2],0)
 def test_watchdog_latches_start_not_camera_heartbeat(self):
  g=CommandWatchdog();g.enable(True);g.signal(True);g.signal(False);g.command(3,.1,10)
  self.assertEqual(g.output(10.1),(3,.1,'PASS'))
  self.assertEqual(g.output(10.5)[2],'COMMAND_TIMEOUT')
 def test_disable_resets_start(self):
  g=CommandWatchdog();g.enable(True);g.signal(True);g.enable(False);g.enable(True);g.command(3,0,10)
  self.assertEqual(g.output(10.1)[2],'WAIT_START_LIGHTS')
 def test_nan_command_stops(self):
  g=CommandWatchdog();g.enable(True);g.signal(True);g.command(float('nan'),0,1)
  self.assertEqual(g.output(1)[0],0)
 def scene(self,block=False):
  x=np.linspace(-2,25,700);walls=np.vstack([np.c_[x,x*0+1.2],np.c_[x,x*0-1.2]])
  obs=np.c_[np.full(100,6.),np.linspace(-1.2 if block else -.18,1.2 if block else .18,100)]
  return compact(np.vstack([walls,obs]))
 def test_static_obstacle_bypass(self):
  p=self.scene();state=BypassState()
  r=propose(p,(np.zeros(3),20,2.4,1),0,3,0,.06,12,2.5,4,.2,state,0)
  self.assertIsNotNone(r);self.assertGreater(r.free,7.2);self.assertIn(r.route_kind,['BYPASS_LEFT','BYPASS_RIGHT'])
 def test_blocked_corridor_has_no_bypass(self):
  r=propose(self.scene(True),(np.zeros(3),20,2.4,1),0,3,0,.06,12,2.5,4,.2,BypassState(),0)
  self.assertIsNone(r)
 def test_bypass_commit_and_release(self):
  s=BypassState();s.commit(1);s.clear(0);s.clear(.3);self.assertEqual(s.side,1);s.clear(.7);self.assertEqual(s.side,0)
