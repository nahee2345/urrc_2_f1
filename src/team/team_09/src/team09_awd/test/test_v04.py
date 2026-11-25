import unittest
import math
import numpy as np
from team09_awd.core import WallTracker,tracking_curvature,plan_local

class GentleControl(unittest.TestCase):
    def test_central_band_no_lateral_correction(self):
        for e in [-.15,-.05,0,.05,.15]:self.assertAlmostEqual(tracking_curvature(e,0,10,0),0)
    def test_cross_track_only_is_gentle(self):
        # 0.3m displacement at 10m/s: under .001 /m, lateral acceleration <.1m/s2.
        self.assertLess(abs(tracking_curvature(.3,0,10,0)),.001)
    def test_heading_still_corrected_inside_band(self):
        self.assertLess(tracking_curvature(.1,-.1,8,0),0)
    def test_lateral_acceleration_bound(self):
        for v in [4,8,10,18]:
            self.assertLessEqual(v*v*abs(tracking_curvature(1.5,.4,v,0)),1.500001)
    def test_odom_predicts_line_during_translation_and_rotation(self):
        t=WallTracker();t.update((.5,0,1),(0,0,0),1.)
        q=t.update((.4/math.cos(.1),-.1,1),(1,.1,.1),1.05)
        self.assertAlmostEqual(q[0],.4/math.cos(.1));self.assertAlmostEqual(q[1],-.1)
    def test_filter_translation_invariant_far_from_origin(self):
        states=[]
        for xy in [(0,0),(1000,-2000)]:
            t=WallTracker();t.update((.3,0,1),(*xy,0),1.)
            states.append(t.update((.32,.01,1),(*xy,0),1.05))
        np.testing.assert_allclose(states[0],states[1],atol=1e-9)
    def test_wall_jump_rejected(self):
        t=WallTracker();t.update((0,0,1),(0,0,0),1.)
        self.assertIsNone(t.update((1.,.3,1),(0,0,0),1.05))
    def test_missing_walls_not_replaced_by_stale_line(self):
        t=WallTracker();t.update((0,0,1),(0,0,0),1.)
        self.assertIsNone(t.update(None,(1,0,0),1.05));self.assertIsNone(t.line)
    def test_noisy_observation_filtered(self):
        t=WallTracker();t.update((0,0,1),(0,0,0),1.)
        q=t.update((.1,.05,1),(0,0,0),1.05)
        self.assertLess(abs(q[0]),.02);self.assertLess(abs(q[1]),.01)
