import math
import unittest
import numpy as np
from team09_awd.racing import (fit_route,plan_racing,route_rollout,preview_speed,
    proximity_cap,combine_vision,CurveTracker,check_route)
from team09_awd.vision import pixels_to_ground,road_preview,CAMERA_PITCH,CAMERA_HEIGHT
from team09_awd.core import speed_limit


def walls(a=0.,offset=0.):
    x=np.linspace(-2,40,3000)
    return np.vstack([np.c_[x,offset+a*x*x+sign*1.2*np.sqrt(1+(2*a*x)**2)] for sign in (-1,1)])

class RacePreview(unittest.TestCase):
    def test_parallel_side_wall_no_fixed_2_5_cap(self):
        p=walls(offset=-.6);r=plan_racing(p,speed=2.5,wall_state=(-.6,0,1))
        self.assertEqual(r.route_kind,'CURVED_LOCAL');self.assertGreater(r.preview_limit,8.)
        self.assertGreater(r.proximity_limit,2.5)
    def test_near_wall_slowdown_retained(self):
        self.assertEqual(proximity_cap(walls(),0,5,.1),.45)
        self.assertEqual(proximity_cap(walls(),0,5,.2),1.2)
    def test_closing_toward_wall_slows(self):
        self.assertEqual(proximity_cap([[.5,.4],[.4,.4]],.4,10,.26),2.5)
    def test_curve_feedforward_both_directions(self):
        for sign in (-1,1):
            r=plan_racing(walls(sign*.015),speed=5.)
            self.assertEqual(r.route_kind,'CURVED_LOCAL');self.assertGreater(sign*r.road_curvature,.015)
    def test_future_corner_brakes_early(self):
        s=np.arange(0,30,.1);k=np.zeros_like(s);k[s>10]=.30
        fast=preview_speed(s,np.zeros_like(s),29,18)
        slow=preview_speed(s,k,29,18)
        self.assertLess(slow,fast)
        self.assertLessEqual(slow**2,2.5/.30+2*4*10.1+.09)
    def test_profile_stops_within_observed_distance(self):
        s=np.arange(0,30,.1)
        for free in (5.,10.,20.):
            v=preview_speed(s,np.zeros_like(s),free,18)
            self.assertLessEqual(v*.2+v*v/8,free-.45+1e-6)
    def test_front_barrier_never_ignored(self):
        p=np.vstack([walls(),np.c_[np.full(100,2.),np.linspace(-1.2,1.2,100)]])
        r=plan_racing(p,speed=5.)
        self.assertLess(r.free,2.5)
    def test_bad_camera_path_rejected(self):
        x=np.linspace(1,15,30);p=np.array([0.,0.,0.])
        mixed,used=combine_vision(p,np.c_[x,np.ones_like(x)*2],20,2.4)
        self.assertFalse(used);np.testing.assert_equal(mixed,p)
    def test_consistent_camera_refines_shape(self):
        x=np.linspace(1,15,30);p=np.array([.005,0,0])
        _,used=combine_vision(p,np.c_[x,.005*x*x+.09*x],20,2.4)
        self.assertTrue(used)
    def test_reactive_fallback_for_ambiguous_road(self):
        r=plan_racing([],speed=1.);self.assertEqual(r.mode,'BLOCKED');self.assertEqual(r.route_kind,'REACTIVE')
    def test_odometry_curve_prediction(self):
        t=CurveTracker();f=(np.array([0.,0.,.5]),20.,2.4,1.)
        t.update(f,(0,0,0),1.)
        out=t.update((np.array([0.,0.,.4]),20.,2.4,1.),(1,.1,0),1.05)
        self.assertAlmostEqual(out[0][2],.4)
    def test_no_old_curve_after_missing_fit(self):
        t=CurveTracker();t.update((np.array([0,0,0]),10,2.4,1),(0,0,0),1.)
        self.assertIsNone(t.update(None,(0,0,0),1.05));self.assertIsNone(t.path)

class CalibratedVision(unittest.TestCase):
    def test_ground_projection_centre(self):
        k=(320,320,320,240);pt=pixels_to_ground([[320,300]],k)
        self.assertGreater(pt[0,0],.6);self.assertAlmostEqual(pt[0,1],0)
    def test_horizon_rejected(self):
        self.assertFalse(np.isfinite(pixels_to_ground([[320,100]],(320,320,320,240))).all())
    def test_synthetic_road_recovered(self):
        k=(320,320,320,240);v,u=np.mgrid[0:480,0:640]
        xy=pixels_to_ground(np.c_[u.ravel(),v.ravel()],k).reshape(480,640,2)
        image=np.zeros((480,640,3),np.uint8)
        road=np.isfinite(xy).all(axis=2)&(xy[:,:,0]>.5)&(abs(xy[:,:,1])<1.2)
        image[road]=[70,70,70]
        result,_=road_preview(image,k)
        self.assertIsNotNone(result);self.assertLess(max(abs(result[:,1])),.03)
    def test_blank_image_rejected(self):
        self.assertIsNone(road_preview(np.zeros((480,640,3),np.uint8),(320,320,320,240))[0])
