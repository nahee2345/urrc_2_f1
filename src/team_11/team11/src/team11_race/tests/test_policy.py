"""Pure offline tests: no ROS, Gazebo or track CSV needed."""
import math
import unittest
import numpy as np
from team11_race.geometry import (ThreeLanePlanner, interpret_scan,
                                 curve_speed, change_distance, quintic)
from team11_race.signal_vision import SignalGate,red_lamp_count


def scan_box(left=1.3,right=-1.3,obstacles=()):
    # Own front GPU LiDAR: x forward, +y left, no world coordinates.
    angle_min=-2.35
    resolution=4.70/720
    ranges=np.full(721,float('inf'),dtype=float)
    for i in range(721):
        angle=angle_min+i*resolution
        sine,cosine=math.sin(angle),math.cos(angle)
        if sine>1e-7:
            distance=left/sine
        elif sine<-1e-7:
            distance=right/sine
        else:
            distance=float('inf')
        if distance>0 and distance<55:
            ranges[i]=distance
        for x,y,radius in obstacles:
            projection=x*cosine+y*sine
            delta=x*x+y*y-projection*projection
            if projection<=0 or delta>radius*radius:
                continue
            hit=projection-math.sqrt(max(0.,radius*radius-delta))
            if hit>0:
                ranges[i]=min(ranges[i],hit)
    return ranges,angle_min,resolution


class GeometryPolicyTests(unittest.TestCase):
    def test_curve_schedule(self):
        self.assertEqual(curve_speed(0.02,False),15.)
        self.assertAlmostEqual(curve_speed(.030,True),7.)
        self.assertAlmostEqual(curve_speed(.055,True),6.)
        self.assertEqual(curve_speed(.085,True),5.)
        self.assertTrue(change_distance(15.,.75)>=18.)
        self.assertEqual(quintic(0),0.)
        self.assertEqual(quintic(1),1.)

    def test_road_boundary_and_physical_grid_side(self):
        for left,right,expected in [(1.3,-1.3,1),
                                    (1.6,-.4,2),(.4,-1.6,0),
                                    (1.75,-.25,2),(.25,-1.75,0)]:
            data,a,d=scan_box(left,right)
            corridor,_,_=interpret_scan(data,a,d,.1,55.)
            self.assertIsNotNone(corridor)
            ctrl=ThreeLanePlanner()
            plan=ctrl.update(data,a,d,.1,55.,.05)
            self.assertEqual(plan.lane,expected)

    def test_no_sensor_fail_closed(self):
        ctrl=ThreeLanePlanner()
        result=ctrl.update([float('inf')]*721,-2.35,4.7/720,.1,55.,.05)
        self.assertEqual(result.speed,0.)
        self.assertEqual(result.state,"NO_ROAD")

    def test_original_right_lane_chooses_middle_when_blocked(self):
        ctrl=ThreeLanePlanner()
        empty,a,d=scan_box(1.6,-.4)
        ctrl.update(empty,a,d,.1,55.,.05)
        ctrl.last_speed=15.
        ctrl.last_change_distance=100.
        ctrl.last_front=12.
        # obstacle on our right lane at 11m
        close,_,_=scan_box(1.6,-.4,((11.2,-.15,.22),))
        out=ctrl.update(close,a,d,.1,55.,.1)
        self.assertEqual(ctrl.destination,1)
        self.assertEqual(out.state,"SHIFT")

    def test_center_lane_chooses_left_on_tie(self):
        ctrl=ThreeLanePlanner()
        empty,a,d=scan_box(1.3,-1.3)
        ctrl.update(empty,a,d,.1,55.,.05)
        ctrl.last_speed=15.
        ctrl.last_change_distance=100.
        ctrl.last_front=12.
        close,_,_=scan_box(1.3,-1.3,((11.2,0.,.22),))
        out=ctrl.update(close,a,d,.1,55.,.1)
        self.assertEqual(ctrl.destination,0)
        self.assertEqual(out.state,"SHIFT")

    def test_no_rear_lidar_implemented(self):
        # The current request explicitly leaves defensive rear LiDAR as design only.
        from pathlib import Path
        model=Path(__file__).parents[1]/"models/team11_car/model.sdf"
        self.assertNotIn('rear_lidar',model.read_text())
        self.assertNotIn('<static>true</static>',model.read_text())


class CameraGateTests(unittest.TestCase):
    def test_five_then_all_off(self):
        cam=SignalGate()
        dark=np.zeros((240,640,3),dtype=np.uint8)
        lit=dark.copy()
        for k in range(5):
            x=140+k*70
            lit[65:71,x:x+8,0]=255
        self.assertEqual(red_lamp_count(lit),5)
        self.assertFalse(cam.observe(dark))
        self.assertFalse(cam.observe(lit))
        self.assertFalse(cam.observe(lit))
        for k in range(3):self.assertFalse(cam.observe(dark))
        self.assertTrue(cam.observe(dark))


if __name__=="__main__":
    unittest.main()
