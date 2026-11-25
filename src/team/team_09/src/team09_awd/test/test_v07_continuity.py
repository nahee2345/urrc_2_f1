import unittest,math
import numpy as np
from team09_awd.continuity import ScanBuffer,command_step,wheel_angles
from team09_awd.core import MAX_CURVATURE,ResumeGate,plan_local,evaluate,compact

class Continuity(unittest.TestCase):
    def test_new_scan_ahead_of_odom_does_not_discard_synced_scan(self):
        b=ScanBuffer();b.add(1.,[],1.);b.add(1.04,[],1.04)
        h=[(.98,0,0,0),(1.02,.04,0,0)]
        self.assertEqual(b.select(h,1.05)[0],1.)
        h.append((1.06,.08,0,0));self.assertEqual(b.select(h,1.07)[0],1.04)
    def test_no_extrapolation_and_no_stale_reuse(self):
        b=ScanBuffer();b.add(1.,[],1.)
        self.assertIsNone(b.select([(.9,0,0,0)],1.01))
        self.assertIsNone(b.select([(.9,0,0,0),(1.1,0,0,0)],1.25))
    def test_time_reset_discards_old_scans(self):
        b=ScanBuffer();b.add(100.,[],1.);b.add(1.,[],2.);self.assertEqual(len(b.frames),1)
    def test_reused_frame_does_not_fake_three_safe_frames(self):
        g=ResumeGate()
        for _ in range(8):self.assertFalse(g.update(True,1.))
        self.assertFalse(g.update(True,1.1));self.assertTrue(g.update(True,1.2))
        self.assertTrue(g.update(True,1.2));self.assertFalse(g.update(False,1.3))
    def test_nominal_deceleration_bounded(self):
        self.assertAlmostEqual(command_step(8,5,.05,1.5,4,12),7.8)
    def test_safety_cap_immediate(self):
        self.assertEqual(command_step(8,5,.05,1.5,4,3),3)
    def test_individual_wheels_respect_30_degrees(self):
        for k in np.linspace(-MAX_CURVATURE,MAX_CURVATURE,100):
            self.assertLessEqual(max(abs(v) for v in wheel_angles(k)),30.000001)
        self.assertAlmostEqual(max(wheel_angles(MAX_CURVATURE)),30.)
    def test_near_wall_escape_is_swept_and_turns_away(self):
        x=np.linspace(-2,20,1500)
        p=np.vstack([np.c_[x,np.full_like(x,y)] for y in (-.24,2.16)])
        r=plan_local(p,speed=.3)
        self.assertNotEqual(r.mode,'BLOCKED');self.assertGreater(r.curvature,0.)
        padding=.025 if r.mode=='RECOVER_CRAWL' else .06
        self.assertGreaterEqual(evaluate(compact(p),r.curvature,0.,.3,padding,48)[0],.45 if r.mode=='RECOVER_CRAWL' else .85)
