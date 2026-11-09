import math
import unittest
from core import (Waypoint, lane_spacing, generate_lanes, closest_lane,
                  LaneDecision, blocked_lanes, commanded_speed, smoothstep5)

def ring(n=800, radius=60., width=2.):
    return [
        Waypoint(radius*2*math.pi*i/n,
                 radius*math.cos(2*math.pi*i/n),
                 radius*math.sin(2*math.pi*i/n),0.,
                 2*math.pi*i/n+math.pi/2,width)
        for i in range(n)
    ]

class LaneTests(unittest.TestCase):
    def setUp(self):
        self.pts=ring()
    def test_three_paths(self):
        lanes=generate_lanes(self.pts)
        self.assertEqual(len(lanes),3)
        self.assertAlmostEqual(lane_spacing(self.pts),.535)
        self.assertEqual(closest_lane(self.pts,*lanes[0][0][1:3]),0)
        self.assertEqual(closest_lane(self.pts,*lanes[2][0][1:3]),2)
    def test_edge_margin(self):
        for lane in generate_lanes(self.pts):
            for p in lane[::40]:
                self.assertLessEqual(
                    math.hypot(p[1]-60*math.cos(p[0]/60),
                               p[2]-60*math.sin(p[0]/60)),.536)
    def test_persistent_change(self):
        policy=LaneDecision(1)
        self.assertEqual(policy.update({1},0),0)
        self.assertEqual(policy.update(set(),9),0)
        self.assertEqual(policy.update(set(),3),0)
        self.assertAlmostEqual(policy.offset(.535),.535)
    def test_obstacle(self):
        p=self.pts[100]
        spacing=lane_spacing(self.pts)
        x=p.x-spacing*math.sin(p.yaw)
        y=p.y+spacing*math.cos(p.yaw)
        self.assertEqual(blocked_lanes(self.pts,self.pts[80].s,[(x,y)]),{0})
    def test_all_blocked(self):
        self.assertIsNone(LaneDecision(1).update({0,1,2},0))
    def test_smooth(self):
        self.assertEqual(smoothstep5(0.),0.)
        self.assertEqual(smoothstep5(1.),1.)
        self.assertAlmostEqual(smoothstep5(.5),.5)
    def test_speed(self):
        self.assertEqual(commanded_speed(math.radians(10)),10)
        self.assertEqual(commanded_speed(math.radians(10.1)),8)
        self.assertEqual(commanded_speed(math.radians(-11)),8)

if __name__=="__main__":
    unittest.main()
