"""Yaw rate has to oppose overshoot without bypassing steering limits."""
import math
import unittest
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.yaw_control import YawFeedback

CFG={'wheel_base_m':.34,'max_center_steer_deg':29.8,
     'yaw_feedback_gain':.065,'yaw_feedback_max_correction_deg':4.}


class V16(unittest.TestCase):
    def test_yaw_overshoot_reduces_steer(self):
        c=YawFeedback(CFG)
        want=math.radians(8.)
        rate=4.*math.tan(want)/.34
        self.assertAlmostEqual(c.correct(want,4.,rate),want,places=7)
        self.assertLess(c.correct(want,4.,rate+.8),want)
        self.assertGreater(c.correct(want,4.,rate-.8),want)

    def test_stale_or_stationary_falls_back(self):
        c=YawFeedback(CFG)
        self.assertEqual(c.correct(.1,4.,None),.1)
        self.assertEqual(c.correct(.1,0.,10.),.1)

    def test_feedback_never_exceeds_bound(self):
        c=YawFeedback(CFG)
        cmd=c.correct(0.,5.,-90.)
        self.assertLessEqual(math.degrees(cmd),4.001)
        self.assertLessEqual(abs(c.correct(1.,5.,-20.)),
                             math.radians(29.8)+1e-6)


if __name__=='__main__': unittest.main()
