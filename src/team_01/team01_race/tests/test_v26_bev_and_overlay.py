"""BEV rendering must preserve unknown space and show an eligible center."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.bev_grid import local_bev
from team01_race.road_debug_overlay import ground_to_rgb
from team01_race.planning_road import RoadCorridor


def road():
    x=np.arange(.6,8.,.2)
    c=np.column_stack((x,np.zeros(len(x))))
    b=np.column_stack((x,-.9*np.ones(len(x)),.9*np.ones(len(x))))
    return RoadCorridor(c,.9,True,0.,b)


class V26(unittest.TestCase):
    def test_birdseye_only_live_corridor_is_free(self):
        grid=local_bev(road(),np.array([[2.,0.],[4.,2.]]))
        self.assertEqual(grid.shape,(30,50))
        self.assertEqual(grid[15,10],100)
        self.assertEqual(grid[15,17],0)
        self.assertEqual(grid[0,20],-1)
        self.assertEqual(grid[25,20],100)

    def test_unknown_road_never_declared_free(self):
        grid=local_bev(None,np.empty((0,2)))
        self.assertTrue(np.all(grid==-1))

    def test_projected_center_ahead_in_frame(self):
        pixels,valid=ground_to_rgb(np.array([[2.,0.],[4.,0.]]),
                                   (240,424,3))
        self.assertTrue(np.all(valid))
        self.assertTrue(np.all(abs(pixels[:,0]-212)<2))


if __name__=='__main__':unittest.main()
