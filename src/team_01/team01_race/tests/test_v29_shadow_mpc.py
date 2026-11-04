"""Experimental MPC never drives actuators or bypasses live road boundaries."""
import math
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.sampled_mpc import search_shadow_mpc
from team01_race.planning_road import RoadCorridor

CFG={'wheel_base_m':.34,'max_center_steer_deg':29.8,
     'max_steer_rate_deg_s':35.,'path_half_width_m':.18}
def road(width=.9):
    x=np.arange(.5,6.,.15)
    y=np.zeros(len(x))
    return RoadCorridor(np.column_stack((x,y)),width,True,0.,
                        np.column_stack((x,y-width,y+width)))

def reference(y=0.):
    x=np.arange(0.,6.,.12)
    return np.column_stack((x,np.full_like(x,y)))


class V29(unittest.TestCase):
    def test_shadow_generates_road_safe_smooth_straight(self):
        p=search_shadow_mpc(road(),reference(),
                            np.empty((0,2)),2.,0.,CFG)
        self.assertIsNotNone(p)
        self.assertLess(abs(math.degrees(p.steer_rad)),3.0)
        self.assertGreater(p.min_clearance_m,0.)
        self.assertLess(float(np.max(abs(p.points[:,1]))),.30)

    def test_invalid_road_or_missing_path_never_generates_motion(self):
        self.assertIsNone(search_shadow_mpc(None,reference(),
            np.empty((0,2)),3.,0.,CFG))
        self.assertIsNone(search_shadow_mpc(road(),None,
            np.empty((0,2)),3.,0.,CFG))

    def test_wall_obstacle_blocks_only_unsafe_candidates(self):
        p=search_shadow_mpc(road(),reference(),
                            np.array([[1.,0.],[1.2,0.]]),2.,0.,CFG)
        self.assertIsNone(p)


if __name__=='__main__':unittest.main()
