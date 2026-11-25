import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team09_awd.core import plan,speed_limit,plan_local,footprint_clearance,interpolate_pose,compensate,ResumeGate,rollout,external_points

class PlannerTest(unittest.TestCase):
    def walls(self):
        x=np.linspace(-2,22,900)
        return np.vstack([np.column_stack([x,np.full_like(x,y)]) for y in [-1.5,1.5]])
    def test_straight(self):
        k,free,c=plan(self.walls());self.assertLess(abs(k),.06);self.assertGreater(free,10);self.assertGreater(c,.35)
    def test_front_obstacle(self):
        wall=np.column_stack([np.full(100,2.),np.linspace(-1.5,1.5,100)])
        _,free,_=plan(np.vstack([self.walls(),wall]));self.assertLess(free,3.)
    def test_empty_and_invalid(self):
        self.assertEqual(plan([]),(0.,0.,0.));self.assertEqual(plan([[np.nan,1]]),(0.,0.,0.))
    def test_curve_slowdown(self):
        self.assertLess(speed_limit(.8,15,1,20),speed_limit(.01,15,1,20))
    def test_braking_distance(self):
        for d in [1.,2.,5.,10.]:
            v=speed_limit(0,d,1,50);self.assertLessEqual(v*.20+v*v/8,d-.45+1e-6)
    def test_blocked(self):
        a=np.linspace(-math.pi,math.pi,360)
        _,d,_=plan(np.column_stack([.35*np.cos(a),.35*np.sin(a)]));self.assertLess(d,.9)
    def test_left_and_right_turn(self):
        # 90 degree road bend at radius 3m, both walls, mirrored for right turn.
        a=np.linspace(0,np.pi/2,400)
        p=np.vstack([np.column_stack([r*np.sin(a),3-r*np.cos(a)]) for r in [1.5,4.5]])
        for sign in [1,-1]:
            q=p.copy();q[:,1]*=sign
            k,_,_=plan(q);self.assertGreater(k*sign,0)



class RegressionTest(unittest.TestCase):
    def corridor(self,right=-.27,left=2.73):
        x=np.linspace(-2,35,1200)
        return np.vstack([np.column_stack([x,np.full_like(x,y)]) for y in [right,left]])
    def test_near_wall_is_not_circle_collision(self):
        r=plan_local(self.corridor());self.assertAlmostEqual(r.clearance,.13,places=2)
        self.assertNotEqual(r.mode,'BLOCKED');self.assertGreater(r.free,.85)
    def test_narrow_but_passable(self):
        r=plan_local(self.corridor(-.32,.32));self.assertNotEqual(r.mode,'BLOCKED');self.assertGreater(r.free,10.)
    def test_actual_footprint_overlap(self):
        p=np.vstack([self.corridor(),[[.20,.13]]]);self.assertEqual(plan_local(p).mode,'BLOCKED');self.assertEqual(footprint_clearance(p),0.)
    def test_close_external_obstacle_kept(self):
        self.assertLess(footprint_clearance(self.corridor()),.14)
        p=external_points([[0.,.27],[0.,.1],[.27,0.]])
        self.assertEqual(len(p),2);self.assertAlmostEqual(p[0,1],.27)
    def test_resume_requires_distinct_frames(self):
        g=ResumeGate();self.assertFalse(g.update(True,1));self.assertFalse(g.update(True,1));self.assertFalse(g.update(True,2));self.assertTrue(g.update(True,3));self.assertFalse(g.update(False,4));self.assertFalse(g.update(True,5));self.assertFalse(g.update(True,6));self.assertTrue(g.update(True,7))
    def test_odometry_interpolation_and_stale(self):
        h=[(1.,0.,0.,3.13),(1.1,1.,0.,-3.13)];p=interpolate_pose(h,1.05)
        self.assertAlmostEqual(p[0],.5);self.assertLess(abs(abs(p[2])-math.pi),.02);self.assertIsNone(interpolate_pose(h,2.))
    def test_motion_compensation(self):
        np.testing.assert_allclose(compensate([[3.,0.]],(0.,0.,0.),(1.,0.,math.pi/2)),[[0.,-2.]],atol=1e-6)
    def test_steering_rate_reachable(self):
        s,p=rollout(1.5,0.,10.);self.assertLess(abs(p[1,2]),.01);self.assertTrue(np.isfinite(p).all())
    def test_centre_bias_away_from_wall(self):
        r=plan_local(self.corridor());self.assertGreater(r.centre_error,0.);self.assertGreater(r.curvature,0.)
    def test_parked_car_has_bypass_candidate(self):
        p=self.corridor(-1.5,1.5);y=np.linspace(-.12,.12,50);x=np.linspace(2.75,3.25,80)
        car=np.vstack([np.column_stack([np.full_like(y,2.75),y]),np.column_stack([x,np.full_like(x,.12)]),np.column_stack([x,np.full_like(x,-.12)])])
        r=plan_local(np.vstack([p,car]));self.assertNotEqual(r.mode,'BLOCKED');self.assertGreater(abs(r.curvature),.01);self.assertGreater(r.free,3.)
    def test_tighter_corner_and_higher_straight_limit(self):
        self.assertLess(speed_limit(.6,20,1,10),speed_limit(.6,20,1,10,lateral=2.5));self.assertGreater(speed_limit(0,32,1,10),6.)

if __name__=='__main__':unittest.main()
