"""Path reuse must be pose-compensated and not trusted after road loss."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.reference_scheduler import ReferenceScheduler
from team01_race.reference_path import GradualReference
from team01_race.planning_road import RoadCorridor

CFG={'wheel_base_m':.34,'reference_update_period_s':.10}
def lane(offset=.35):
    xs=np.arange(.5,12.,.25)
    y=np.full_like(xs,offset)
    return RoadCorridor(np.column_stack((xs,y)),.8,True,0.,
                        np.column_stack((xs,y-.8,y+.8)))

class V17(unittest.TestCase):
    def test_reference_updates_10hz_with_20hz_control(self):
        s=ReferenceScheduler(GradualReference(CFG),CFG)
        a=s.update(lane(),0.,2.,0.,0.,.05)
        self.assertIsNotNone(a)
        self.assertEqual(s.count,1)
        b=s.update(lane(),0.,2.,0.,0.,.05)
        self.assertEqual(s.count,1)
        self.assertEqual(s.source,'PREDICT')
        self.assertIsNotNone(b)
        s.update(lane(),0.,2.,0.,0.,.05)
        self.assertEqual(s.count,2)

    def test_yaw_and_forward_motion_shift_local_path(self):
        s=ReferenceScheduler(GradualReference(CFG),CFG)
        s.update(lane(),0.,3.,0.,0.,.05)
        old=s.path.copy()
        s.update(lane(),0.,3.,.3,0.,.05)
        self.assertNotEqual(s.path[12,1],old[12,1])

    def test_invalid_road_removes_reference(self):
        s=ReferenceScheduler(GradualReference(CFG),CFG)
        s.update(lane(),0.,2.,0.,0.,.05)
        bad=RoadCorridor(np.empty((0,2)),0.,False,0.)
        self.assertIsNone(s.update(bad,0.,2.,0.,0.,.05))
        self.assertEqual(s.source,'EMPTY')

if __name__=='__main__':unittest.main()
