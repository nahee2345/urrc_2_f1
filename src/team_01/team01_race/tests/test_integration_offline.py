"""ROS/Gazebo-free acceptance checks with synthetic sensor readings."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.planning import TrafficGate,estimate_road_corridor,SafetyPlanner
from team01_race.perception import red_signal_features,depth_geometry
from test_offline import CFG,synthetic_road


def lit_frame(count, dark=False):
    import cv2
    im=np.zeros((180,420,3),np.uint8)
    for i in range(5):
        # Keep 5 unlit static lenses in the image even after lights-out.
        cv2.circle(im,(45+75*i,34),7,(100,22,20),-1)
        if i<count:
            cv2.circle(im,(45+75*i,34),7,(255,8,8),-1)
    return im


def rails(road_fn, robot=(0.,0.,0.), half_width=.9):
    w=np.arange(robot[0],robot[0]+12.,.075)
    # Mock a lidar with two continuously detected barriers.
    return transform_rails(w,road_fn,robot,half_width)


def transform_rails(w,fn,robot,half_width):
    rx,ry,yaw=robot
    x=np.r_[w,w]-rx
    y=np.r_[fn(w)+half_width,fn(w)-half_width]-ry
    c,s=math.cos(yaw),math.sin(yaw)
    pts=np.stack((c*x+s*y,-s*x+c*y),axis=1)
    return pts[(pts[:,0]>.2)&(pts[:,0]<12)&(np.abs(pts[:,1])<4.)]


class IntegrationOfflineTests(unittest.TestCase):
    def test_four_red_to_zero_count_only(self):
        gate=TrafficGate(required=4,on_frames=2,off_frames=3)
        for n in (0,1,2,3,4,4,4,3,2):
            self.assertFalse(gate.observe(n))
        for n in (0,0):self.assertFalse(gate.observe(n))
        self.assertTrue(gate.observe(0))
        self.assertEqual(gate.status,'LIGHTS_OUT')

    def test_camera_four_to_zero_from_brightness(self):
        g=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        frames=[]
        frames.extend([(0,.1*i) for i in range(10)])
        frames.extend([(1,1.1),(2,1.7),(3,2.3),(4,2.9),(4,3.0),(4,3.1)])
        frames.extend([(0,4.1),(0,4.2),(0,4.3)])
        for n,t in frames:
            blob,intensity=red_signal_features(lit_frame(n))
            out=g.observe(blob,intensity,timestamp=t)
        self.assertEqual(g.maximum_blobs,4)
        self.assertTrue(out)

    def test_camera_unlit_red_appearance_is_not_start(self):
        g=TrafficGate(required=4,on_frames=2,off_frames=3)
        for i in range(130):
            blobs,level=red_signal_features(lit_frame(0))
            self.assertFalse(g.observe(blobs,level,timestamp=i*.05))

    def test_camera_flicker_while_red_does_not_launch(self):
        g=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        sequence=[(0,.05*i) for i in range(10)] + [(1,1.0),(2,1.5),(3,2.0),
                  (4,2.5),(4,2.6),(4,2.7),(0,2.8),(0,2.9),
                  (4,3.0),(4,3.1),(4,3.2),(0,3.3),(0,3.4),(0,4.8),(0,4.9),(0,5.0)]
        for n,t in sequence[:-1]:
            k,energy=red_signal_features(lit_frame(n))
            self.assertFalse(g.observe(k,energy,t))
        k,energy=red_signal_features(lit_frame(0))
        self.assertTrue(g.observe(k,energy,5.0))

    def test_three_red_not_enough(self):
        g=TrafficGate(required=4,on_frames=2,off_frames=3)
        for n in (0,1,2,3,3,3,0,0,0):self.assertFalse(g.observe(n))

    def test_guardrail_centerline_and_wrong_side(self):
        x=np.arange(.4,7,.08)
        road=lambda x:.025*x*x
        pts=np.vstack((np.column_stack((x,road(x)+.9)),np.column_stack((x,road(x)-.9))))
        result=estimate_road_corridor(pts)
        self.assertTrue(result.valid)
        self.assertGreater(result.heading_deg,3.)
        one_side=pts[:len(x)]
        self.assertFalse(estimate_road_corridor(one_side).valid)

    def test_road_steering_left_right_and_straight(self):
        x=np.arange(.4,7,.06)
        for a in (0.,.02,-.02):
            center=a*x*x
            walls=np.vstack((np.column_stack((x,center+.9)),np.column_stack((x,center-.9))))
            p=SafetyPlanner(CFG)
            for _ in range(35):
                m=p.step(np.empty((0,2)),walls,4.,.1,True,True,True)
            self.assertTrue(p.road.valid)
            self.assertGreater(m.speed,0.)
            if a>0:self.assertLess(m.steering,-.002)
            elif a<0:self.assertGreater(m.steering,.002)
            else:self.assertLess(abs(m.steering),math.radians(8))  # stable off-centre racing line

    def test_unknown_road_speed_limit(self):
        p=SafetyPlanner({**CFG,'unmapped_speed_mps':.75})
        for _ in range(100):
            m=p.step(np.empty((0,2)),np.empty((0,2)),3.,.1,True,True,True)
        self.assertLessEqual(m.speed,.751)
        self.assertFalse(p.road.valid)

    def test_road_cant_identify_grade_as_obstacle(self):
        for g in (.08,-.08,0.):
            scene=depth_geometry(synthetic_road(g))
            self.assertTrue(scene.valid)
            self.assertEqual(len(scene.obstacles),0)

    def test_mock_curve_tracking(self):
        # Deliberately low-speed kinematics, NOT Gazebo or a physics test.
        fn=lambda x:.23*np.sin(x/3.2)
        x,y,yaw=0.,0.,0.
        planner=SafetyPlanner(CFG)
        maxerr=0.
        modes=set()
        for _ in range(150):
            cloud=rails(fn,(x,y,yaw))
            motion=planner.step(np.empty((0,2)),cloud,4.,.1,True,True,True)
            yaw+=motion.speed*math.tan(-motion.steering)/.34*.1
            x+=motion.speed*math.cos(yaw)*.1
            y+=motion.speed*math.sin(yaw)*.1
            maxerr=max(maxerr,abs(y-fn(x)))
            modes.add(motion.mode)
        self.assertGreater(x,4.)
        self.assertLess(maxerr,.65, (x,y,yaw,maxerr,modes))

if __name__=='__main__':unittest.main()
