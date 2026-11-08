"""Record failed curb projection without changing safety geometry."""
import pathlib
import sys
import unittest
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.planning_geometry import PlannerGeometry
from team01_race.planning_road import RoadCorridor
from team01_race.defaults import DEFAULTS


class EdgeEvidence(unittest.TestCase):
    def make(self):
        p=PlannerGeometry(DEFAULTS)
        xs=np.arange(.6,5.,.2)
        centers=np.column_stack((xs,np.zeros_like(xs)))
        bounds=np.column_stack((xs,np.full_like(xs,-.45),np.full_like(xs,.45)))
        p.road=RoadCorridor(centers,.45,True,0.,bounds)
        return p

    def test_boundary_violation_evidence(self):
        p=self.make()
        safe,_=p._within_road(.25,3.)
        self.assertFalse(safe)
        self.assertEqual(p.last_arc_rejection['reason'],'curb_envelope')
        self.assertGreater(p.last_arc_rejection['shortfall_m'],0.)

    def test_straight_no_false_violation(self):
        p=self.make()
        safe,_=p._within_road(0.,3.)
        self.assertTrue(safe)
        self.assertIsNone(p.last_arc_rejection)


if __name__=='__main__':unittest.main()
