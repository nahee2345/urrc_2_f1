"""Real Decision/Control nodes with future Camera paths; no steering-based entry."""
import json
import math
import os
from pathlib import Path
import time
import unittest
import yaml
os.environ['ROS_DOMAIN_ID']='94'
import numpy as np
import pytest
import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Header, String
from vision_msgs.msg import Detection3DArray
from nav_msgs.msg import Path as PathMsg
from rclpy.qos import qos_profile_sensor_data
from team_01_race.ros_support import path_message


@pytest.mark.launch_test
def generate_test_description():
    package=Path(__file__).resolve().parents[1]
    parameters=[str(package/'config/racing.yaml'),str(package/'config/corner_control.yaml'),{'primary_steering_source':'auto','depth_corner.enabled':False,'use_sim_time':False}]
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='team_01_race',executable='decision_node',name='team_01_decision',
                                parameters=[*parameters,{'require_official_start':False}]),
        launch_ros.actions.Node(package='team_01_race',executable='control_adapter_node',name='team_01_control_adapter',parameters=parameters),
        launch_testing.actions.ReadyToTest()])


class TestCornerPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def test_left_right_preview_hysteresis_exit_and_stop(self):
        node=rclpy.create_node('corner_pipeline_test')
        root=Path(__file__).resolve().parents[1]
        policy=yaml.safe_load((root/'config/corner_control.yaml').read_text())['/**']['ros__parameters']['corner_control']
        straight_limit=policy['straight_max_speed_mps'];mild_limit=policy['mild_max_speed_mps'];sharp_limit=policy['sharp_max_speed_mps']
        accel=yaml.safe_load((root/'config/racing.yaml').read_text())['team_01_control_adapter']['ros__parameters']['control']['max_accel_mps2']
        try:
            publishers={name:node.create_publisher(PathMsg,'/team_01/camera/'+name,10)
                        for name in ('center_path','left_boundary','right_boundary')}
            status_pub=node.create_publisher(DiagnosticArray,'/team_01/camera/path_status',10)
            obstacles=node.create_publisher(Detection3DArray,'/team_01/perception/obstacles',10)
            imu=node.create_publisher(Imu,'/team_01/imu',qos_profile_sensor_data)
            emergency=node.create_publisher(Bool,'/team_01/emergency_stop',10)
            controls=[];node.create_subscription(String,'/team_01/debug/corner_control',lambda m:controls.append(json.loads(m.data)),10)
            speed=[];requested=[];states=[];steering=[];debug=[]
            node.create_subscription(Float32,'/team_01/speed',lambda m:speed.append(m.data),10)
            node.create_subscription(Float32,'/team_01/internal/requested_speed',lambda m:requested.append(m.data),10)
            node.create_subscription(String,'/team_01/debug/corner_state',lambda m:states.append(m.data),10)
            node.create_subscription(Float32,'/team_01/debug/desired_steering',lambda m:steering.append(m.data),10)
            node.create_subscription(String,'/team_01/debug/corner_status',lambda m:debug.append(m.data),10)
            preview_pub=node.create_publisher(String,'/team_01/camera/braking_preview',10)
            far_x=np.arange(.1,22.01,.1);far_straight=np.column_stack((far_x,np.zeros_like(far_x)))
            far=far_straight
            x=np.arange(.1,5.01,.1)
            straight=np.column_stack((x,np.zeros_like(x)))
            selected=straight
            def spin(seconds):
                deadline=time.monotonic()+seconds
                while time.monotonic()<deadline:
                    header=Header();header.frame_id='team_01/base_link';header.stamp=node.get_clock().now().to_msg()
                    for name,points in [('center_path',selected),('left_boundary',selected+[0.,1.2]),('right_boundary',selected-[0.,1.2])]:
                        publishers[name].publish(path_message(points,header))
                    message=DiagnosticArray();message.header=header
                    message.status=[DiagnosticStatus(name='camera_path',level=DiagnosticStatus.OK,
                        values=[KeyValue(key=k,value=v) for k,v in dict(confidence='1.0',left_observed='1',right_observed='1',traffic_light='LIGHTS_OUT').items()])]
                    status_pub.publish(message)
                    if far is not None:
                        preview_pub.publish(String(data=json.dumps(dict(stamp=header.stamp.sec+header.stamp.nanosec*1e-9,frame_id=header.frame_id,confidence=1.,path=far.tolist()))))
                    detections=Detection3DArray();detections.header=header;obstacles.publish(detections)
                    attitude=Imu();attitude.header.stamp=header.stamp;attitude.header.frame_id='team_01/imu_link';attitude.orientation.w=1.;imu.publish(attitude)
                    for _ in range(20):rclpy.spin_once(node,timeout_sec=.0005)
                    time.sleep(.02)
            # Start the acceleration observation after DDS connects both nodes;
            # discovery time is not controller acceleration time.
            discovery=time.monotonic()+8.
            while time.monotonic()<discovery and (publishers['center_path'].get_subscription_count()<2 or status_pub.get_subscription_count()<2):
                rclpy.spin_once(node,timeout_sec=.02)
            spin(straight_limit/accel+3.)
            self.assertTrue(speed)
            self.assertGreater(speed[-1],straight_limit-.1)
            self.assertEqual(states[-1],'STRAIGHT')
            fx=np.arange(.1,8.01,.1);far=np.column_stack((fx,.175*np.maximum(0.,fx-6.)**2))
            spin(.3)
            self.assertLessEqual(requested[-1],sharp_limit+.0001)
            self.assertLessEqual(speed[-1],sharp_limit+.0001)
            self.assertLess(abs(steering[-1]),.005)
            far=far_straight;spin((straight_limit-sharp_limit)/accel+2.)
            self.assertGreater(speed[-1],straight_limit-.1)
            for sign in (1.,-1.):
                selected=np.column_stack((x,sign*.20*np.maximum(0.,x-4.)**2))
                before=speed[-1];control_start=len(controls);spin(.25)
                self.assertIn('CORNER_APPROACH',states)
                self.assertIn(states[-1],('CORNER_APPROACH','CORNER_COMMIT'))
                braking=[sample for sample in controls[control_start:] if sample['corner_braking']]
                self.assertTrue(braking)
                for sample in braking:self.assertLessEqual(sample['final_speed'],sample['requested_speed']+1e-5)  # Compare one control snapshot; topic callbacks arrive independently.
                self.assertLess(speed[-1],before)
                self.assertLess(abs(steering[-1]),.005)  # Future corner, still straight PP target.
                spin(1.4)
                self.assertLessEqual(speed[-1],mild_limit+.01)
                self.assertLessEqual(requested[-1],mild_limit+.01)
                selected=np.column_stack((x,sign*.10*x*x));spin(.65)
                self.assertEqual(states[-1],'CORNER_COMMIT')
                self.assertLessEqual(speed[-1],mild_limit+.01)
                selected=straight;spin(.04)
                self.assertNotEqual(states[-1],'STRAIGHT')
                selected=np.column_stack((x,sign*.10*x*x));spin(.3)
                exit_speed=speed[-1]
                selected=straight;spin(1.3)
                self.assertIn('CORNER_EXIT',states)
                self.assertEqual(states[-1],'STRAIGHT')
                self.assertLessEqual(speed[-1],exit_speed+accel*1.3+.15)  # Existing acceleration limit, not a cruise-speed step.
                spin(max(0.,straight_limit-speed[-1])/accel+.8)
                self.assertGreater(speed[-1],straight_limit-.1)
            self.assertTrue(all(0.<=v<=straight_limit+.0001 for v in speed))
            self.assertTrue(any(c['corner_braking'] for c in controls))
            self.assertTrue(all(c['final_speed']<=c['requested_speed']+1e-6 for c in controls if c['corner_braking']))
            self.assertTrue(any('required_braking_distance=' in value for value in debug))
            self.assertTrue(any('active_speed_limiter=' in value for value in debug))
            emergency.publish(Bool(data=True));spin(.15)
            self.assertEqual(speed[-1],0.)
        finally:node.destroy_node()
