"""Feedback stays fresh under slow headless Gazebo only while sim clock advances."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.sensor_freshness import feedback_fresh


class Freshness(unittest.TestCase):
    def test_slow_headless_physics_feedback_remains_valid(self):
        self.assertTrue(feedback_fresh(15.10,15.04,101.2,100.0))

    def test_true_old_sim_sample_is_rejected(self):
        self.assertFalse(feedback_fresh(15.50,15.04,100.2,100.0))

    def test_wall_disconnect_is_always_rejected(self):
        self.assertFalse(feedback_fresh(15.10,15.04,103.0,100.0))

    def test_missing_clock_uses_conservative_wall_guard(self):
        self.assertTrue(feedback_fresh(0.,0.,100.4,100.0))
        self.assertFalse(feedback_fresh(0.,0.,100.9,100.0))

    def test_sim_clock_reset_disables_feedback(self):
        self.assertFalse(feedback_fresh(14.,15.,101.,100.))


if __name__=='__main__':unittest.main()
