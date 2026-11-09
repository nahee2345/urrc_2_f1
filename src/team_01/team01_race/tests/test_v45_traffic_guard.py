"""Conservative multi-vehicle proximity helpers; QA execution deferred."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.traffic_guard import required_headway, should_abort_pass


class TrafficGuardCases(unittest.TestCase):
    def test_closing_traffic_requires_more_gap(self):
        self.assertGreater(required_headway(4.,1.),required_headway(1.,4.))

    def test_rapidly_closing_lead_aborts(self):
        self.assertTrue(should_abort_pass(1.,4.,1.))

    def test_unknown_target_stays_conservative(self):
        self.assertTrue(should_abort_pass(.5,3.,None))


if __name__=='__main__':unittest.main()
