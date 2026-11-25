import unittest
from team09_awd.continuity import plan_is_current
class Deadline(unittest.TestCase):
    def test_compute_time_included(self):
        self.assertTrue(plan_is_current(1.,1.05,.99,2,2))
        self.assertFalse(plan_is_current(1.,1.31,.99,2,2))
    def test_reset_invalidates_inflight_plan(self):
        self.assertFalse(plan_is_current(1.,1.05,.99,2,3))
    def test_time_rewind_and_future_scan_rejected(self):
        self.assertFalse(plan_is_current(2.,1.,.99,2,2))
        self.assertFalse(plan_is_current(1.,1.01,1.02,2,2))
