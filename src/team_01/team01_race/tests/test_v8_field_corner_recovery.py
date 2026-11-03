"""v8 regression from 2026-11-02 real-Gazebo log (synthetic geometric repro).

These tests do NOT claim that Gazebo successfully completed a corner.
"""
from pathlib import Path
import sys
import unittest
import math
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from team01_race.planning import SafetyPlanner
from team01_race.race_strategy import RacingLine
from team01_race.curved_path import adaptive_pursuit_candidate
from test_v7_grid_and_depth_corner import CONFIG


def near_curb(left=.40, right=.40, xmax=3.2):
    x=np.arange(.55,xmax,.16)
    # A conservative local straight: near-side clearance constant throughout
    return np.column_stack((x, np.full_like(x,-right),np.full_like(x,left)))


class FieldCornerV8(unittest.TestCase):
    def test_repro_critical_left_edge_before_log_negative_clearance(self):
        p=SafetyPlanner(CONFIG)
        m=p.step(np.empty((0,2)),np.empty((0,2)),5.,.08,
                 True,True,True,near_curb(left=.04,right=.70),actual_speed=1.8)
        self.assertEqual(m.mode,'BRAKE')
        self.assertEqual(m.speed,0.)
        self.assertEqual(p.edge_alert,'CRITICAL')
        self.assertLess(p.near_edge_m,.235)

    def test_emergency_at_negative_clearance(self):
        p=SafetyPlanner(CONFIG)
        m=p.step(np.empty((0,2)),np.empty((0,2)),5.,.08,
                 True,True,True,near_curb(left=-.07,right=.72),actual_speed=1.8)
        self.assertEqual(m.mode,'BRAKE')
        self.assertIn('curb',m.reason)

    def test_proactive_speed_cap_at_point_27_before_breach(self):
        p=SafetyPlanner(CONFIG)
        for _ in range(10):
            m=p.step(np.empty((0,2)),np.empty((0,2)),5.,.10,
                     True,True,True,near_curb(left=.27,right=.42))
        self.assertEqual(p.edge_alert,'CAUTION')
        self.assertIsNotNone(p.edge_speed_cap_mps)
        self.assertLess(p.edge_speed_cap_mps,1.0)
        self.assertLessEqual(m.speed,p.edge_speed_cap_mps+.001)

    def test_nominal_two_curb_center_does_not_force_stop(self):
        p=SafetyPlanner(CONFIG)
        for _ in range(50):
            m=p.step(np.empty((0,2)),np.empty((0,2)),5.,.10,
                     True,True,True,near_curb(left=.55,right=.55,xmax=5.0))
        self.assertNotEqual(m.mode,'BRAKE')
        self.assertGreater(m.speed,1.5)
        self.assertEqual(p.edge_alert,'NONE')

    def test_adaptive_rejects_candidate_with_car_outside_near_curb(self):
        p=SafetyPlanner(CONFIG)
        road=p._visual_corridor(near_curb(left=.04,right=.70))
        cand=adaptive_pursuit_candidate(road,np.empty((0,2)),CONFIG,1.8,horizon=2.)
        self.assertIsNone(cand)

    def test_opposite_curve_restarts_entry_instead_of_old_exit(self):
        p=SafetyPlanner(CONFIG)
        line=RacingLine(CONFIG)
        from team01_race.planning import RoadCorridor
        def road(heading):
            b=near_curb(left=.60,right=.60,xmax=5.0)
            c=np.column_stack((b[:,0],np.zeros(len(b))))
            return RoadCorridor(c,.6,True,heading,b)
        for _ in range(50):line.update(road(0),.75,.1)
        line.update(road(25),1.,.1)
        for _ in range(45):line.update(road(25),1.,.1)
        self.assertEqual(line.phase,'CORNER_EXIT')
        line.update(road(-24),1.,.1)
        self.assertEqual(line.phase,'CORNER_ENTRY')
        self.assertEqual(line.corner_sign,-1)

    def test_racing_line_offset_gives_way_to_edge(self):
        line=RacingLine(CONFIG)
        from team01_race.planning import RoadCorridor
        for _ in range(50):
            b=near_curb(.6,.6,5.0)
            line.update(RoadCorridor(np.column_stack((b[:,0],np.zeros(len(b)))),.6,True,0,b),1.0,.1)
        for _ in range(15):
            b=near_curb(.6,.6,5.0)
            line.update(RoadCorridor(np.column_stack((b[:,0],np.zeros(len(b)))),.6,True,20,b),1.0,.1)
        self.assertGreater(abs(line.offset_m),0.01)
        b=near_curb(.29,.75,5.0)
        road=RoadCorridor(np.column_stack((b[:,0],(b[:,1]+b[:,2])/2)),.52,True,20,b)
        for _ in range(10):line.update(road,1.,.1)
        self.assertAlmostEqual(line.offset_m,0.,delta=.02)

    def test_no_outside_exit_line_while_bend_remains_steep(self):
        line=RacingLine(CONFIG)
        from team01_race.planning import RoadCorridor
        def road(heading):
            b=near_curb(left=.65,right=.65,xmax=5.)
            c=np.column_stack((b[:,0],np.zeros(len(b))))
            return RoadCorridor(c,.65,True,heading,b)
        for _ in range(50):line.update(road(0),.75,.1)
        for _ in range(45):line.update(road(25),1.,.1)
        self.assertEqual(line.phase,'CORNER_EXIT')
        self.assertAlmostEqual(line.offset_m,0.,delta=.015)

    def test_signal_gate_unchanged(self):
        from team01_race.planning import TrafficGate
        t=TrafficGate(required=4,on_frames=2,off_frames=3)
        for n in (1,2,3,4,4):t.observe(n)
        for n in (0,0,0):t.observe(n)
        self.assertTrue(t.lights_out)

if __name__=='__main__':unittest.main()
