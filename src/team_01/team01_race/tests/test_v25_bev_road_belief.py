"""Road history filters noise but never authorizes missing current sensors."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.road_belief import RoadBelief
from team01_race.planning_road import RoadCorridor


def road(center=0.,width=.9):
    x=np.arange(.6,8.,.2)
    centers=np.column_stack((x,np.full_like(x,center)))
    bounds=np.column_stack((x,np.full_like(x,center-width),
                                np.full_like(x,center+width)))
    return RoadCorridor(centers,width,True,0.,bounds)


class V25(unittest.TestCase):
    def test_current_sensors_agree(self):
        b=RoadBelief()
        result=b.update(road(.03),road(.04))
        self.assertTrue(result.valid)
        self.assertEqual(b.source,'rgb_depth+lidar')

    def test_single_lidar_glitch_not_followed(self):
        b=RoadBelief()
        b.update(road(0.),road(0.))
        result=b.update(road(.02),road(.47))
        self.assertEqual(b.source,'rgb_depth')
        self.assertLess(float(np.median(abs(result.centers[:,1]))),.06)

    def test_real_bend_not_overfiltered(self):
        b=RoadBelief()
        b.update(road(0.),road(0.))
        result=b.update(road(.55),road(.55))
        self.assertGreater(float(np.median(result.centers[:,1])),.50)

    def test_previous_path_not_permission_to_drive(self):
        b=RoadBelief()
        b.update(road(0.),road(0.))
        result=b.update(None,None)
        self.assertFalse(result.valid)
        self.assertEqual(b.source,'none')


if __name__=='__main__':unittest.main()
