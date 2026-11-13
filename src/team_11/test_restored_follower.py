"""Restored route-follower regression: 1 right, 2 centre, 3 left."""
import csv
import math
import tempfile
import unittest
from pathlib import Path
from routes import generate
from strategy import Track
from team_11.restored_follower import RestoredFollower,smoothstep5

class RestoredFollowerTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        points=[]
        for i in range(240):
            t=2*math.pi*i/240
            points.append({"x_m":30*math.cos(t),"y_m":30*math.sin(t),
                           "z_m":0.0,"yaw_rad":t+math.pi/2,"road_width_m":4.0})
        lanes,_=generate(points)
        for lane,rows in lanes.items():
            with (Path(self.tmp.name)/f"lane_{lane}.csv").open("w",newline="") as fh:
                writer=csv.DictWriter(fh,fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        self.track=Track(self.tmp.name)
    def tearDown(self):
        self.tmp.cleanup()
    def car(self,idx,lane):
        return self.track.lanes[lane][idx]
    def test_quintic_ends_and_midpoint(self):
        self.assertEqual(smoothstep5(0),0)
        self.assertEqual(smoothstep5(1),1)
        self.assertAlmostEqual(smoothstep5(.5),.5)
        self.assertAlmostEqual(smoothstep5(.25),1-smoothstep5(.75))
    def test_right_center_left_orientation(self):
        i=35
        p=self.car(i,2)
        a=self.car(i,1);b=self.car(i,3)
        ux=-math.sin(p["yaw_rad"])
        uy=math.cos(p["yaw_rad"])
        right=(a["x_m"]-p["x_m"])*ux+(a["y_m"]-p["y_m"])*uy
        left=(b["x_m"]-p["x_m"])*ux+(b["y_m"]-p["y_m"])*uy
        self.assertLess(right,0)
        self.assertGreater(left,0)
    def test_lane_change_reference_starts_smoothly(self):
        f=RestoredFollower(self.track,1)
        p=self.car(30,1)
        a=f.guidance(p["x_m"],p["y_m"],p["yaw_rad"],5,1,1)
        b=f.guidance(p["x_m"],p["y_m"],p["yaw_rad"],5,1.05,2)
        self.assertTrue(b.changing)
        self.assertEqual(b.target_lane,2)
        self.assertLessEqual(abs(b.steer-a.steer),1.57*.05+1e-6)
        self.assertLessEqual(abs(b.steer),math.pi/6+1e-6)
    def test_initial_lane_three_preserved(self):
        f=RestoredFollower(self.track,3)
        p=self.car(20,3)
        out=f.guidance(p["x_m"],p["y_m"],p["yaw_rad"],2,1,3)
        self.assertEqual(out.target_lane,3)
        self.assertFalse(out.changing)
    def test_nonfinite_and_clock_rejected(self):
        f=RestoredFollower(self.track,1)
        p=self.car(20,1)
        with self.assertRaises(ValueError):
            f.guidance(float("nan"),0,0,2,1,1)
        f.guidance(p["x_m"],p["y_m"],p["yaw_rad"],2,1,1)
        with self.assertRaises(ValueError):
            f.guidance(p["x_m"],p["y_m"],p["yaw_rad"],2,1,1)

if __name__=="__main__": unittest.main()
