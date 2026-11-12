#!/usr/bin/env python3
"""Offline TEAM 11 race-tactics review. No Gazebo, cameras or start lights.

Runs the real strategy.Planner on an artificial closed circuit. A FAIL is a
confirmed discrepancy in this offline scenario, NOT a real-track lap result.
This review is intentionally separate from the basic test_*.py unit suite.
"""
import csv
import math
import tempfile
import unittest
from pathlib import Path

from routes import generate
from strategy import Track, Planner, Vehicle

class TacticsReview(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        points = []
        for i in range(240):
            angle = 2 * math.pi * i / 240
            points.append(dict(x_m=25 * math.cos(angle),
                               y_m=25 * math.sin(angle), z_m=0.,
                               yaw_rad=angle+math.pi/2, road_width_m=4.0))
        lanes, _ = generate(points)
        for lane, rows in lanes.items():
            with (Path(self.tmp.name) / f"lane_{lane}.csv").open("w", newline="") as f:
                writer=csv.DictWriter(f, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        self.track=Track(self.tmp.name)
        self.planner=Planner(self.track, 1)

    def tearDown(self):
        self.tmp.cleanup()

    def car(self, idx, lane=1, speed=4, stamp=1, name="ego"):
        p=self.track.lanes[lane][idx]
        return Vehicle(p["x_m"],p["y_m"],p["yaw_rad"],speed,stamp,name)

    def test_01_no_departure_without_start_gate(self):
        c=self.planner.step(self.car(10),[],1,False)
        self.assertEqual((c.mode,c.speed),("WAIT_SIGNAL",0))

    def test_02_front_collision_within_three_seconds_triggers_overtake(self):
        cmd=self.planner.step(self.car(10,speed=6),
                              [self.car(20,speed=2,name="front")],1)
        self.assertEqual(cmd.mode,"PASS")

    def test_03_front_ttc_over_three_seconds_does_not_overtake_early(self):
        # 20 track steps = 13.1 m; closing speed 3 m/s, TTC > 3s.
        ego=self.car(10,speed=5)
        front=self.car(30,speed=2,name="front")
        gap,_=self.planner._relative(ego,front,10)
        self.assertGreater((gap-1)/(ego.speed-front.speed),3)
        cmd=self.planner.step(ego,[front],1)
        self.assertNotEqual(cmd.mode,"PASS",
            "User's requested overtaking threshold is 3s, not 5s")

    def test_04_rear_ttc_within_five_seconds_is_defended(self):
        # Rear in lane 2, >3s but <5s away from catching up.
        ego=self.car(30,lane=1,speed=4)
        rear=self.car(5,lane=2,speed=8,name="rear")
        gap,_=self.planner._relative(ego,rear,30)
        ttc=(-gap-1)/(rear.speed-ego.speed)
        self.assertGreater(ttc,3)
        self.assertLess(ttc,5)
        cmd=self.planner.step(ego,[rear],1)
        self.assertEqual(cmd.mode,"DEFEND",
            "Rear predicted catch-up <=5s should be defense candidate")

    def test_05_no_defense_when_car_ahead(self):
        cmd=self.planner.step(self.car(30,lane=1,speed=4),
            [self.car(35,lane=1,speed=1,name="front"),
             self.car(20,lane=2,speed=10,name="rear")],1)
        self.assertNotEqual(cmd.mode,"DEFEND")

    def test_06_blocked_adjacent_path_must_not_be_entered(self):
        ego=self.car(10,lane=1,speed=7)
        other=[self.car(15,lane=1,speed=1,name="front"),
               self.car(12,lane=2,speed=2,name="middle"),
               self.car(12,lane=3,speed=3,name="left")]
        c=self.planner.step(ego,other,1)
        self.assertNotEqual(c.lane,2)
        self.assertNotEqual(c.lane,3)

    def test_07_corner_returns_outer_path_to_center(self):
        self.track.curvature=[.20]*self.track.n
        c=self.planner.step(self.car(10,lane=1,speed=5),[],1)
        self.assertEqual(c.lane,2,
            "Corner control should select middle path 2")

    def test_08_corner_disables_passing_and_caps_speed(self):
        self.track.curvature=[.20]*self.track.n
        c=self.planner.step(self.car(10,speed=7),
           [self.car(14,speed=1,name="front")],1)
        self.assertNotEqual(c.mode,"PASS")
        self.assertLessEqual(c.speed,10)

    def test_09_steer_and_rate_are_bounded(self):
        first=self.planner.step(self.car(10),[],1.0)
        second=self.planner.step(self.car(10),[],1.05)
        for cmd in (first,second):
            self.assertLessEqual(abs(cmd.steer),math.radians(30)+1e-8)
        self.assertLessEqual(abs(second.steer-first.steer),
                             math.radians(90)*.05+1e-8)

    def test_10_stale_opponent_is_not_trusted(self):
        c=self.planner.step(self.car(10),[self.car(15,name="stale",stamp=-2)],1)
        self.assertNotEqual(c.mode,"PASS")

    def test_11_bad_pose_stops(self):
        ego=self.car(10)
        ego.x=float("nan")
        c=self.planner.step(ego,[],1)
        self.assertEqual((c.mode,c.speed),("STOP",0))

    def test_12_straight_return_after_lane_two_needs_no_unnecessary_recross(self):
        # A completed lane-2 overtake must not force a return to lane 1/3.
        self.planner.lane=2
        self.planner.target=2
        self.planner.mode="CRUISE"
        c=self.planner.step(self.car(10,lane=2),[],1)
        self.assertEqual(c.lane,2)

if __name__=="__main__":
    print("TEAM11 OFFLINE TACTICS REVIEW | no camera/no Gazebo/no signal light",flush=True)
    result=unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(TacticsReview))
    print("RESULT: PASS %d / %d, FAIL %d, ERROR %d" %
          (result.testsRun-len(result.failures)-len(result.errors),
           result.testsRun,len(result.failures),len(result.errors)),flush=True)
    if not result.wasSuccessful():
        print("NOT RACE-READY: discrepancies above require correction",flush=True)
    raise SystemExit(0 if result.wasSuccessful() else 1)
