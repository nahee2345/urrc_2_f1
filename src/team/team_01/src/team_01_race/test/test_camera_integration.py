import math
import json
import os
from pathlib import Path
import time
import unittest

# Keep test actuators and synthetic race inputs away from a running competition.
os.environ['ROS_DOMAIN_ID'] = '120'
import cv2
import numpy as np
import pytest
import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from team_01_race.image_codec import from_bgr
from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu
from std_msgs.msg import Bool, Float32, String
from team_01_race.vision import CameraVision, VisionConfig


@pytest.mark.launch_test
def generate_test_description():
    config = str(Path(__file__).resolve().parents[1]/'config/racing.yaml')
    nodes = []
    for executable,name,extra in [
        ('sensor_tf_node','team_01_sensor_tf',{}),
        ('camera_perception_node','team_01_camera_perception',{'corner_braking_preview.enabled':False}),
        ('lidar_obstacle_node','team_01_lidar_obstacle',{}),
        ('decision_node','team_01_decision',{'require_official_start':False}),
        ('control_adapter_node','team_01_control_adapter',{'control.max_accel_mps2':100.0})]:
        nodes.append(launch_ros.actions.Node(package='team_01_race',executable=executable,name=name,
                                             parameters=[config,{'primary_steering_source':'auto','depth_corner.enabled':False,'use_sim_time':False,'camera_geometry.prefer_bundle':True},extra]))
    return launch.LaunchDescription(nodes+[launch_testing.actions.ReadyToTest()])


class TestCameraPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()

    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('camera_pipeline_test')
        self.camera = self.node.create_publisher(Image,'/team_01/camera/image_raw',qos_profile_sensor_data)
        self.info_pub = self.node.create_publisher(CameraInfo,'/team_01/camera/camera_info',qos_profile_sensor_data)
        self.scan = self.node.create_publisher(LaserScan,'/team_01/scan',qos_profile_sensor_data)
        self.speed=[];self.steering=[];self.modes=[];self.transforms={};self.parent_frames=set();self.lateral_accel=[]
        def tf_callback(msg):
            for transform in msg.transforms:
                self.transforms[transform.child_frame_id]=transform
                self.parent_frames.add(transform.header.frame_id)
        self.node.create_subscription(TFMessage,'/tf_static',tf_callback,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(Float32,'/team_01/speed',lambda m:self.speed.append(m.data),10)
        def steering_callback(msg):
            self.steering.append(msg.data)
            if self.speed:self.lateral_accel.append(self.speed[-1]**2*abs(math.tan(msg.data))/.34)
        self.node.create_subscription(Float32,'/team_01/steering',steering_callback,10)
        self.node.create_subscription(String,'/team_01/debug/mode',lambda m:self.modes.append(m.data),10)
        from team_01_race.geometry import vehicle_geometry
        g=vehicle_geometry(Path(__file__).resolve().parents[1]/'models/team_01_car/model.sdf')
        cfg=VisionConfig();pose=g['camera_pose'];cfg.camera_x=pose[0];cfg.camera_y=pose[1];cfg.camera_height=g['camera_height_m'];cfg.camera_roll,cfg.camera_pitch,cfg.camera_yaw=pose[3:]
        c = CameraVision(cfg);self.fixture_vision=c
        import xml.etree.ElementTree as ET
        model=ET.parse(Path(__file__).resolve().parents[1]/'models/team_01_car/model.sdf')
        fov=float(model.findtext("model/link[@name='camera_link']/sensor/camera/horizontal_fov"))
        focal=320./math.tan(fov/2)
        self.k = np.array([[focal,0,320],[0,focal,240],[0,0,1]])
        self.image=np.zeros((480,640,3),np.uint8);self.image[:]=(100,170,100)
        polygon=c.project([[.3,1.2],[.3,-1.2],[30,-1.2],[30,1.2]],self.k).astype(np.int32)
        cv2.fillPoly(self.image,[polygon],(65,65,65))
        # A thick fixed-pixel stroke at the 120deg lens invents a narrowing
        # corridor in metres. Render the boundary edge, not an inward wall slab.
        for y in [-1.2,1.2]:cv2.polylines(self.image,[c.project([[.3,y],[30,y]],self.k).astype(np.int32)],False,(255,255,255),2)
        self.imu_pub=self.node.create_publisher(Imu,'/team_01/imu',qos_profile_sensor_data)
        self.emergency_pub=self.node.create_publisher(Bool,'/team_01/emergency_stop',10)
        self.pitch=0.;self.imu_invalid=False;self.ramp_states=[];self.ramp_limits=[]
        self.node.create_subscription(String,'/team_01/debug/ramp_state',lambda m:self.ramp_states.append(m.data),10)
        self.node.create_subscription(Float32,'/team_01/debug/ramp_speed_limit',lambda m:self.ramp_limits.append(m.data),10)
        # This integration covers pixel near control, start, ramp and watchdogs.
        # Far pixel curvature has its own fixture suite; provide a separate
        # measured straight preview so preview bands do not mask ramp behavior.
        self.far_preview=self.node.create_publisher(String,'/team_01/camera/braking_preview',10)
        far_x=np.arange(.1,22.01,.1);self.far_path=np.column_stack((far_x,np.zeros_like(far_x))).tolist()
        self.red=False
        self.obstacle=False

    def tearDown(self):self.node.destroy_node()

    def spin_for(self,seconds,camera=True,lidar=True,imu=True):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            stamp=self.node.get_clock().now().to_msg()
            if imu:
                msg=Imu();msg.header.stamp=stamp;msg.header.frame_id='team_01/imu_link'
                msg.orientation.y=math.sin(math.radians(self.pitch)/2)
                msg.orientation.w=0. if self.imu_invalid else math.cos(math.radians(self.pitch)/2)
                self.imu_pub.publish(msg)
            if camera:
                info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='team_01/camera_optical_frame'
                info.width=640;info.height=480;info.k=self.k.flatten().tolist();info.d=[0.0]*5
                self.info_pub.publish(info)
                image=self.image.copy()
                for i in range(5):cv2.circle(image,(275+22*i,70),7,(0,0,255) if self.red else (0,0,0),-1)
                msg=from_bgr(image);msg.header=info.header;self.camera.publish(msg)
                self.far_preview.publish(String(data=json.dumps(dict(stamp=stamp.sec+stamp.nanosec*1e-9,frame_id='team_01/base_link',confidence=1.,path=self.far_path))))
            if lidar:
                scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='team_01/laser_frame'
                scan.angle_min=-math.pi/2;scan.angle_max=math.pi/2;scan.angle_increment=math.pi/180
                scan.range_min=.04;scan.range_max=6.;scan.ranges=[math.inf]*181
                if self.obstacle:
                    for degree in range(-3,4):scan.ranges[degree+90]=3.8/math.cos(math.radians(degree))
                self.scan.publish(scan)
            for _ in range(12):
                rclpy.spin_once(self.node,timeout_sec=.001)
            time.sleep(.025)

    def test_visual_start_red_stop_and_sensor_watchdogs(self):
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and any(pub.get_subscription_count()<1 for pub in (self.camera,self.scan,self.imu_pub)):
            rclpy.spin_once(self.node,timeout_sec=.05)
        for pub in (self.camera,self.scan,self.imu_pub):
            self.assertGreater(pub.get_subscription_count(),0,'synthetic sensor subscription was not discovered')
        self.spin_for(.6)
        required_frames={'team_01/laser_frame','team_01/camera_link','team_01/camera_optical_frame','team_01/imu_link'}
        deadline=time.monotonic()+5
        while (not self.speed or not required_frames<=self.transforms.keys()) and time.monotonic()<deadline:self.spin_for(.1)
        self.assertTrue(self.speed)
        self.assertTrue(required_frames<=self.transforms.keys())
        self.assertIn('team_01/base_link',self.parent_frames)
        self.assertGreater(self.transforms['team_01/camera_link'].transform.translation.z,
                           self.transforms['team_01/laser_frame'].transform.translation.z)
        self.assertEqual(self.speed[-1],0.)  # An initially dark image is not a start.
        self.red=True;self.spin_for(.6)
        self.assertEqual(self.speed[-1],0.)
        self.red=False;self.spin_for(1.2)
        self.assertIn('CENTER',self.modes)
        self.assertGreater(self.speed[-1],0.)
        self.assertTrue(all(v <= 7.00001 for v in self.speed))
        self.assertTrue(all(abs(v) <= .5235991 for v in self.steering))
        self.assertEqual(self.ramp_states[-1],'NORMAL')
        self.assertGreater(self.speed[-1],6.8)
        before=self.speed[-1]
        self.pitch=7.;self.spin_for(.4)
        self.assertEqual(self.ramp_states[-1],'RAMP')
        self.assertEqual(self.ramp_limits[-1],5.)
        # Confirmed ramp safety ceiling now has final-speed priority, like
        # confirmed corner/hold reductions; ordinary cruise acceleration stays smooth.
        self.assertLessEqual(self.speed[-1],5.00001)
        self.spin_for(2.)
        self.assertLessEqual(self.speed[-1],5.01)
        self.pitch=-7.;self.spin_for(.7)
        self.assertEqual(self.ramp_states[-1],'RAMP')
        self.pitch=0.;self.spin_for(.9)
        self.assertEqual(self.ramp_states[-1],'NORMAL')
        self.spin_for(.6,imu=False)
        self.assertEqual(self.ramp_states[-1],'IMU_STALE')
        self.assertEqual(self.ramp_limits[-1],5.)
        self.spin_for(1.)
        self.assertEqual(self.ramp_states[-1],'NORMAL')
        self.imu_invalid=True;self.spin_for(.2)
        self.assertEqual(self.ramp_states[-1],'IMU_STALE')
        self.imu_invalid=False;self.spin_for(.5)
        straight_image = self.image.copy()
        camera = self.fixture_vision
        x = np.linspace(.3,15.,300)
        left = np.column_stack((x,.03*x*x+1.2));right = np.column_stack((x,.03*x*x-1.2))
        self.image[:]=(100,170,100)
        cv2.fillPoly(self.image,[camera.project(np.vstack((left,right[::-1])),self.k).astype(np.int32)],(65,65,65))
        for wall in (left,right):
            cv2.polylines(self.image,[camera.project(wall,self.k).astype(np.int32)],False,(255,255,255),2)
        self.spin_for(.7)
        self.assertGreater(max(abs(v) for v in self.steering[-30:]),.005)
        self.assertLessEqual(max(self.lateral_accel),3.1)
        self.image = straight_image
        self.red=True;self.spin_for(.35)
        self.assertEqual(self.speed[-1],0.)
        self.assertEqual(self.modes[-1],'STOP_RED')
        self.pitch=7.;self.obstacle=True;self.spin_for(.4)
        self.red=False;self.spin_for(1.0)
        self.assertTrue(any(mode in ('LEFT_PASS','RIGHT_PASS') for mode in self.modes[-60:]))
        self.assertEqual(self.ramp_states[-1],'RAMP')
        self.assertLess(self.speed[-1],5.)
        self.emergency_pub.publish(Bool(data=True));self.spin_for(.2)
        self.assertEqual(self.speed[-1],0.)
        self.assertEqual(self.modes[-1],'EMERGENCY_STOP')
        self.emergency_pub.publish(Bool(data=False));self.spin_for(.3)
        self.obstacle=False;self.spin_for(.3)
        self.assertIn(self.modes[-1],('LEFT_PASS','RIGHT_PASS'))
        # The synthetic camera is stationary; no relative motion proves the
        # car's rear has cleared. A new visual race cycle must reset the hold.
        self.pitch=0.;self.red=True;self.spin_for(.35)
        self.red=False;self.spin_for(.8)
        self.assertEqual(self.modes[-1],'CENTER')
        self.assertGreater(self.speed[-1],0.)
        self.spin_for(.5,lidar=False)
        self.assertEqual(self.speed[-1],0.)
        self.assertEqual(self.modes[-1],'EMERGENCY_STOP')
        self.spin_for(.8)
        # The preceding test accelerates to 7 m/s with a test-only 100 m/s²
        # ramp. Restoring LiDAR must not bypass the physical braking hold.
        self.assertEqual(self.speed[-1],0.)
        self.spin_for(2.5)
        self.assertGreater(self.speed[-1],0.)
        self.spin_for(.8,camera=False)
        self.assertEqual(self.speed[-1],0.)
