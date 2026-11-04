"""A safety-checked offset must affect actual Stanley steering prediction."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.curved_path import adaptive_pursuit_candidate
from team01_race.planning_road import RoadCorridor

CFG={'wheel_base_m':.34,'max_center_steer_deg':29.8,
     'path_half_width_m':.18,'stanley_heading_gain':.75,
     'stanley_cross_gain':1.,'stanley_speed_softening_mps':1.5}


def straight(width=1.):
    x=np.arange(.6,7.,.16)
    centers=np.column_stack((x,np.zeros_like(x)))
    bounds=np.column_stack((x,np.full_like(x,-width),
                            np.full_like(x,width)))
    return RoadCorridor(centers,width,True,0.,bounds)


class SafeOffset(unittest.TestCase):
    def test_offset_changes_stanley_not_just_target_display(self):
        road=straight()
        x=np.linspace(0.,6.,61)
        reference=np.column_stack((x,np.zeros_like(x)))
        args=(road,np.empty((0,2)),CFG,2.)
        centre=adaptive_pursuit_candidate(*args,reference=reference,horizon=3.)
        right=adaptive_pursuit_candidate(*args,relative_offset=-.12,
                                         reference=reference,horizon=3.)
        left=adaptive_pursuit_candidate(*args,relative_offset=.12,
                                        reference=reference,horizon=3.)
        self.assertIsNotNone(centre)
        self.assertIsNotNone(right)
        self.assertIsNotNone(left)
        self.assertLess(right.steer_rad,centre.steer_rad)
        self.assertGreater(left.steer_rad,centre.steer_rad)
        self.assertGreaterEqual(right.min_edge_clearance,-.005)
        self.assertGreaterEqual(left.min_edge_clearance,-.005)

    def test_offset_does_not_allow_unsafe_narrow_road(self):
        road=straight(width=.20)
        x=np.linspace(0.,6.,61)
        ref=np.column_stack((x,np.zeros_like(x)))
        cand=adaptive_pursuit_candidate(
            road,np.empty((0,2)),CFG,2.,reference=ref,
            relative_offset=.16,horizon=3.)
        self.assertIsNone(cand)


if __name__=='__main__':unittest.main()
