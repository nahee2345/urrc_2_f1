"""Gazebo simulated seconds, not ROS callback wall time, drive lap metres."""
import sys
import pathlib
import unittest
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.planning import SafetyPlanner
from test_offline import CFG


class PhysicsClock(unittest.TestCase):
    def test_odom_speed_integrates_only_physics_time(self):
        p=SafetyPlanner(CFG)
        p.step(np.empty((0,2)),np.empty((0,2)),4.,.10,
               True,True,True,actual_speed=1.5,distance_dt=.02)
        self.assertAlmostEqual(p.lap_learner.distance_m,.03,places=4)

    def test_unchanged_sim_clock_does_not_advance_lap(self):
        p=SafetyPlanner(CFG)
        p.step(np.empty((0,2)),np.empty((0,2)),4.,.10,
               True,True,True,actual_speed=1.5,distance_dt=0.)
        self.assertEqual(p.lap_learner.distance_m,0.)

    def test_missing_odom_never_counts_command_speed(self):
        p=SafetyPlanner(CFG)
        p.step(np.empty((0,2)),np.empty((0,2)),4.,.10,
               True,True,True,actual_speed=None,distance_dt=.02)
        self.assertEqual(p.lap_learner.distance_m,0.)


if __name__=='__main__':unittest.main()
