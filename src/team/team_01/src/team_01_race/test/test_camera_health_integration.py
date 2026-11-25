"""Exercise the installed standalone camera executable with real ROS messages."""
import os
import time
import unittest
os.environ['ROS_DOMAIN_ID'] = '97'
import numpy as np
import pytest
import launch
from launch.actions import ExecuteProcess
import launch_testing.actions
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String
from team_01_race.image_codec import from_bgr


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        ExecuteProcess(cmd=['ros2', 'run', 'team_01_race', 'camera_perception_node',
                            '--ros-args', '-p', 'use_sim_time:=false'], output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestHealth(unittest.TestCase):
    def test_health_independent_of_calibration_path_and_lights(self):
        rclpy.init()
        node = rclpy.create_node('camera_health_test')
        try:
            image = node.create_publisher(Image, '/team_01/camera/image_raw', qos_profile_sensor_data)
            info = node.create_publisher(CameraInfo, '/team_01/camera/camera_info', qos_profile_sensor_data)
            ready = []; paths = []; lights = []
            node.create_subscription(Bool, '/team_01/debug/camera_ready', lambda m: ready.append(m.data), 10)
            node.create_subscription(Bool, '/team_01/debug/path_ready', lambda m: paths.append(m.data), 10)
            node.create_subscription(String, '/team_01/traffic_light_state', lambda m: lights.append(m.data), 10)
            def spin(duration, valid=None, calibrated=False):
                deadline = time.monotonic()+duration
                while time.monotonic() < deadline:
                    if valid is not None:
                        msg = from_bgr(np.zeros((480, 640, 3), np.uint8))
                        msg.header.stamp = node.get_clock().now().to_msg()
                        if not valid: msg.data = b''
                        if calibrated:
                            c = CameraInfo(); c.header = msg.header; c.width = 640; c.height = 480
                            c.k = [320., 0., 320., 0., 320., 240., 0., 0., 1.]
                            info.publish(c)
                        image.publish(msg)
                    end = time.monotonic()+.04
                    while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=.002)
            deadline=time.monotonic()+8.
            while time.monotonic()<deadline and image.get_subscription_count()<1:
                rclpy.spin_once(node,timeout_sec=.02)
            spin(.3)
            assert ready and not ready[-1], ready
            def until_ready(calibrated=False):
                # Large best-effort Image samples can be dropped by DDS;
                # await the observed state instead of a fixed final tick.
                deadline=time.monotonic()+3.
                while time.monotonic()<deadline:
                    spin(.2,True,calibrated)
                    if ready and ready[-1] and (not calibrated or (lights and lights[-1]=='UNKNOWN')):return
                raise AssertionError('normal camera frames did not become ready')
            until_ready()  # Decodable black frames, no calibration/path/red lamps.
            assert ready[-1] and not paths[-1], (ready, paths)
            spin(.3,True,True)
            until_ready(True)
            assert ready[-1] and not paths[-1] and lights[-1] == 'UNKNOWN', (ready, paths, lights)
            spin(.4, False, True)
            assert not ready[-1], ready
            until_ready(True)
            assert ready[-1], ready
            spin(.45)
            assert not ready[-1] and not paths[-1], (ready, paths)
        finally:
            node.destroy_node()
            rclpy.shutdown()
