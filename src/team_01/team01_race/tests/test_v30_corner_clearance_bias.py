"""Corner reference targets the safe central band, never the inner wall."""
import unittest
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.reference_path import GradualReference
from team01_race.planning_road import RoadCorridor

CFG={'corner_center_bias_gain':.45,'corner_center_bias_max_m':.10}
def road(curve):
    x=np.arange(.6,8.,.12)
    y=curve*x*x
    return RoadCorridor(np.column_stack((x,y)),.62,True,22.,
                        np.column_stack((x,y-.62,y+.62)))

class V30(unittest.TestCase):
    def test_smooth_corner_reference_keeps_measured_border_margin(self):
        road_obs=road(.12)
        ref=GradualReference(CFG).build(road_obs,0.,2.)
        self.assertIsNotNone(ref)
        self.assertAlmostEqual(ref[0,1],0.,places=6)
        x=np.array([4.,5.,6.])
        target=np.interp(x,ref[:,0],ref[:,1])
        center=np.interp(x,road_obs.centers[:,0],road_obs.centers[:,1])
        self.assertTrue(np.all(target<=center+.01))
        self.assertTrue(np.all(target>=center-.11))

    def test_straight_reference_still_at_measured_center(self):
        r=road(0.)
        ref=GradualReference(CFG).build(r,0.,2.)
        self.assertAlmostEqual(np.interp(5.,ref[:,0],ref[:,1]),0.,delta=.001)

if __name__=='__main__':unittest.main()
