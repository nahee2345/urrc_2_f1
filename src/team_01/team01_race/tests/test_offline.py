import math
import pathlib
import sys
import unittest
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.perception import depth_geometry, laser_to_xy, red_blob_count
from team01_race.planning import TrafficGate, SafetyPlanner

CFG=dict(straight_speed_mps=5.,corner_speed_factor=.30,
         corner_threshold_deg=10., max_center_steer_deg=29.8,
         wheel_base_m=.34, max_lateral_accel_mps2=5.,max_braking_mps2=5.,
         accel_mps2=2.,reaction_delay_s=.3,emergency_margin_m=.55,
         path_half_width_m=.18,overtake_offset_m=.48,
         overtake_advantage_mps=.8,rear_defend_min_gap_m=1.5,
         unsafe_allow_without_front_lidar=False)


def synthetic_road(grade=0., h=240,w=424):
    yy,xx=np.mgrid[:h,:w]; f=w/(2*np.tan(np.deg2rad(92)/2))
    zcam=-(yy-(h-1)/2)/f
    p=np.deg2rad(11)
    zx=-np.sin(p)+np.cos(p)*zcam
    dx=np.cos(p)+np.sin(p)*zcam
    denom=zx-grade*dx
    d=(grade*.214-.04-.112)/np.where(abs(denom)<1e-6,np.nan,denom)
    d[(d<.12)|(d>13.8)]=np.inf
    return d.astype(np.float32)


class OfflineTests(unittest.TestCase):
    def test_gate_requires_five_then_dark(self):
        g=TrafficGate(required=5,on_frames=2,off_frames=3)
        for v in [0,0,1,2,3,4,0,0,0]:
            self.assertFalse(g.observe(v))
        for v in [5,5,4,3,0,0]:
            self.assertFalse(g.observe(v))
        self.assertTrue(g.observe(0))

    def test_sensor_fails_closed(self):
        p=SafetyPlanner(CFG)
        for enabled,depth,laser in [(False,True,True),(True,False,True),
                                    (True,True,False)]:
            cmd=p.step(np.empty((0,2)),np.empty((0,2)),None,.05,
                       enabled,depth,laser)
            self.assertEqual(cmd.speed,0.)

    def test_straight_steering_and_acceleration(self):
        p=SafetyPlanner(CFG)
        m=p.step(np.empty((0,2)),np.empty((0,2)),None,.1,True,True,True)
        self.assertEqual(m.mode,'GRID')
        self.assertAlmostEqual(m.steering,0.,places=5)
        self.assertGreater(m.speed,0.)
        self.assertLessEqual(m.speed,.2+1e-6)

    def test_front_hazard_stops(self):
        p=SafetyPlanner(CFG)
        obstacle=np.array([[.48,y] for y in np.linspace(-1.,1.,31)])
        m=p.step(obstacle,obstacle,None,.1,True,True,True)
        self.assertEqual(m.speed,0.)
        self.assertEqual(m.mode,'BRAKE')

    def test_rear_vehicle_never_causes_aggressive_swerve(self):
        p=SafetyPlanner(CFG)
        m=p.step(np.empty((0,2)),np.empty((0,2)),.7,.1,True,True,True)
        self.assertEqual(m.mode,'GRID')
        self.assertTrue(p.defend)
        self.assertEqual(p.overtake_side,0)
        self.assertAlmostEqual(m.steering,0.,places=6)

    def test_grade_not_obstacle(self):
        for grade in [0., .08, -.08]:
            scene=depth_geometry(synthetic_road(grade),min_road_points=45)
            self.assertTrue(scene.valid,grade)
            self.assertGreater(scene.ground_count,45)
            self.assertLess(len(scene.obstacles),12,grade)

    def test_slope_positive_uphill(self):
        scene=depth_geometry(synthetic_road(.08))
        self.assertGreater(scene.slope_deg,3.)

    def test_red_blob_count(self):
        img=np.zeros((180,400,3),np.uint8)
        import cv2
        for x in (45,105,165,225,285):cv2.circle(img,(x,32),5,(255,0,0),-1)
        self.assertEqual(red_blob_count(img),5)

    def test_lidar_xy(self):
        xy=laser_to_xy([2.,float('inf')],-.1,.2,sensor_x=.187)
        self.assertEqual(xy.shape,(1,2))
        self.assertGreater(xy[0,0],2.)

    def test_high_visibility_yellow_body(self):
        sdf=ET.parse(str(pathlib.Path(__file__).resolve().parents[1]
                         /'models/team01_car/model.sdf'))
        for name in ('chassis_visual','front_wing_visual','nose_visual','rear_wing_visual'):
            visual=sdf.find(f".//visual[@name='{name}']")
            self.assertIsNotNone(visual)
            r,g,b,a=map(float, visual.findtext('material/diffuse').split())
            self.assertGreater(r,.98)
            self.assertGreater(g,.83)
            self.assertLess(b,.03)
            self.assertEqual(a,1.)

    def test_local_launcher_shares_gazebo_partition(self):
        script=(pathlib.Path(__file__).resolve().parents[1]
                /'scripts/run_local_monza.sh').read_text()
        self.assertIn('export GZ_PARTITION=',script)
        self.assertIn('ros2 run ros_gz_sim create',script)
        self.assertIn('gz model --list',script)
        self.assertIn('tail -1 || true',script)
        self.assertIn('set +u\nsource /opt/ros/jazzy/setup.bash\nset -u',script)

    def test_vehicle_bounds_and_joints(self):
        sdf=ET.parse(str(pathlib.Path(__file__).resolve().parents[1]
                         /'models/team01_car/model.sdf'))
        m=sdf.find('.//model')
        self.assertEqual(len(m.findall('link')),7)
        self.assertEqual(len(m.findall('joint')),6)
        j=[n for n in m.findall('joint') if n.attrib['name'].endswith('steer_joint')]
        self.assertEqual(len(j),2)
        for joint in j:
            axis=joint.find('axis/limit')
            self.assertLessEqual(float(axis.findtext('upper')),math.radians(35))
        sensors=m.findall('.//sensor')
        self.assertEqual(len(sensors),6)
        self.assertEqual({s.get('name') for s in sensors},
                         {'front_rgb','road_rgb','front_depth',
                          'front_lidar','rear_lidar','team01_imu'})
        imu=m.find(".//sensor[@name='team01_imu']")
        self.assertEqual(imu.get('type'),'imu')
        self.assertGreaterEqual(float(imu.findtext('update_rate')),50.)
        chassis=m.find("link[@name='base_link']")
        front=chassis.find("collision[@name='front_wing_collision']")
        self.assertAlmostEqual(float(front.findtext('geometry/box/size').split()[1]),.25)
        self.assertAlmostEqual(float(front.findtext('geometry/box/size').split()[0]),.08)
        plugins=[x.get('name','') for x in m.findall('plugin')]
        self.assertTrue(any('AckermannSteering' in name for name in plugins))
        self.assertTrue(any('systems::Imu' in name for name in plugins))
        self.assertTrue(any('JointStatePublisher' in name for name in plugins))


if __name__=='__main__':unittest.main()
