"""Metric corridor replaces BEV raster with measured bounds only."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.corridor_node import measured_corridor
from team01_race.road_debug_overlay import ground_to_rgb
from team01_race.planning_road import RoadCorridor


def road():
    x=np.arange(.6,8.,.2)
    c=np.column_stack((x,np.zeros(len(x))))
    b=np.column_stack((x,-.9*np.ones(len(x)),.9*np.ones(len(x))))
    return RoadCorridor(c,.9,True,0.,b)


class V26(unittest.TestCase):
    def test_measured_obstacle_and_visible_stations(self):
        out=measured_corridor(road(),np.array([[2.,0.],[4.,2.]]))
        self.assertTrue(out['valid'])
        self.assertEqual(out['representation'],'metric_corridor')
        self.assertAlmostEqual(out['nearest_obstacle_m'],2.)
        self.assertNotIn(10.,[s['forward_m'] for s in out['stations']])

    def test_unknown_road_stays_unknown(self):
        out=measured_corridor(None,np.empty((0,2)))
        self.assertFalse(out['valid'])
        self.assertEqual(out['stations'],[])

    def test_projected_center_in_camera(self):
        pixels,valid=ground_to_rgb(np.array([[2.,0.],[4.,0.]]),(240,424,3))
        self.assertTrue(np.all(valid))
        self.assertTrue(np.all(abs(pixels[:,0]-212)<2))


if __name__=='__main__':unittest.main()
