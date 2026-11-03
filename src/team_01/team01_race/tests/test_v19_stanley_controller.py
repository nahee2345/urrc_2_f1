"""Stanley path tracking and collision-aware rollout invariants."""
import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.stanley_control import stanley_steer
from team01_race.curved_path import adaptive_pursuit_candidate
from team01_race.reference_path import GradualReference
from team01_race.planning_road import RoadCorridor

CFG={'wheel_base_m':.34,'max_center_steer_deg':29.8,
     'path_half_width_m':.18,'stanley_heading_gain':.85,
     'stanley_cross_gain':1.0,'stanley_speed_softening_mps':1.5}
def path(y=.4):
    x=np.linspace(0.,10.,101)
    return np.column_stack((x,np.full_like(x,y)))


class V19(unittest.TestCase):
    def test_lateral_error_corrects_in_expected_direction(self):
        pos=stanley_steer(path(.4),0.,0.,0.,3.,.34,.52,CFG)
        self.assertGreater(pos,0.)
        neg=stanley_steer(path(-.4),0.,0.,0.,3.,.34,.52,CFG)
        self.assertLess(neg,0.)

    def test_overshot_heading_commands_countersteer(self):
        cmd=stanley_steer(path(0.),0.,0.,.2,3.,.34,.52,CFG)
        self.assertLess(cmd,0.)
        self.assertAlmostEqual(
            stanley_steer(path(0.),0.,0.,0.,3.,.34,.52,CFG),0.)

    def test_high_speed_reduces_lateral_gain(self):
        slow=stanley_steer(path(.4),0.,0.,0.,2.,.34,.52,CFG)
        fast=stanley_steer(path(.4),0.,0.,0.,8.,.34,.52,CFG)
        self.assertLess(fast,slow)

    def test_predicted_stanley_candidate_remains_road_bounded(self):
        x=np.arange(.5,9.,.2)
        y=.018*x*x
        road=RoadCorridor(np.column_stack((x,y)),.95,True,5.,
            np.column_stack((x,y-.95,y+.95)))
        ref=GradualReference(CFG).build(road,0.,2.)
        c=adaptive_pursuit_candidate(road,np.empty((0,2)),CFG,2.,
                                     horizon=4.5,reference=ref)
        self.assertIsNotNone(c)
        self.assertGreater(c.first_collision_m,2.0)

if __name__=='__main__':unittest.main()
