"""Newly seen corners invalidate stale pose-projected straight paths."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.reference_path import GradualReference
from team01_race.reference_scheduler import ReferenceScheduler
from team01_race.planning_road import RoadCorridor

CFG={'wheel_base_m':.34,'reference_update_period_s':.10}
def road(curve):
    x=np.arange(.5,7.,.13); y=curve*x*x
    return RoadCorridor(np.column_stack((x,y)),1.0,True,0.,
                        np.column_stack((x,y-1.,y+1.)))

class V22(unittest.TestCase):
    def test_unexpected_new_corner_forces_refresh(self):
        s=ReferenceScheduler(GradualReference(CFG),CFG)
        s.update(road(0.),0.,1.,0.,0.,.05)
        self.assertEqual(s.count,1)
        s.update(road(.10),0.,1.,0.,0.,.05)
        self.assertEqual(s.count,2)
        self.assertEqual(s.source,'REFRESH')
        self.assertIsNotNone(s.path)

    def test_invalid_road_cannot_hold_old_reference(self):
        s=ReferenceScheduler(GradualReference(CFG),CFG)
        s.update(road(0.),0.,1.,0.,0.,.05)
        bad=RoadCorridor(np.empty((0,2)),0.,False,0.)
        self.assertIsNone(s.update(bad,0.,1.,0.,0.,.05))

if __name__=='__main__':unittest.main()
