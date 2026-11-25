"""Real ROS TF/LiDAR classification -> Decision/Control, synthetic ray scans."""
import os
os.environ['ROS_DOMAIN_ID']='98'
import json
from pathlib import Path
import sys
import time
import unittest
sys.path.insert(0,str(Path(__file__).parent))
import pytest
import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String, Float32, Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from vision_msgs.msg import Detection3DArray
from team_01_race.ros_support import path_message
from test_lidar_wall_filter import corridor, ray_scan


@pytest.mark.launch_test
def generate_test_description():
    config=str(Path(__file__).resolve().parents[1]/'config/racing.yaml')
    nodes=[]
    for exe,name,extra in [('sensor_tf_node','team_01_sensor_tf',{}),
        ('lidar_obstacle_node','team_01_lidar_obstacle',{}),('decision_node','team_01_decision',{'require_official_start':False}),
        ('control_adapter_node','team_01_control_adapter',{})]:
        nodes.append(launch_ros.actions.Node(package='team_01_race',executable=exe,name=name,
                         parameters=[config,{'primary_steering_source':'auto','depth_corner.enabled':False,'use_sim_time':False},extra],output='screen'))
    return launch.LaunchDescription(nodes+[launch_testing.actions.ReadyToTest()])


class TestWallPipeline(unittest.TestCase):
    def test_corner_walls_real_car_stale_geometry_and_critical_ttc(self):
        rclpy.init();n=rclpy.create_node('wall_pipeline_test')
        try:
            pubs={key:n.create_publisher(PathMsg,'/team_01/camera/'+key,10) for key in ('center_path','left_boundary','right_boundary')}
            meta=n.create_publisher(DiagnosticArray,'/team_01/camera/path_status',10)
            scan=n.create_publisher(LaserScan,'/team_01/scan',qos_profile_sensor_data)
            imu=n.create_publisher(Imu,'/team_01/imu',qos_profile_sensor_data)
            obstacles=[];modes=[];speeds=[];debug=[]
            n.create_subscription(Detection3DArray,'/team_01/perception/obstacles',lambda m:obstacles.append(m),10)
            n.create_subscription(String,'/team_01/debug/mode',lambda m:modes.append(m.data),10)
            n.create_subscription(Float32,'/team_01/speed',lambda m:speeds.append(m.data),10)
            n.create_subscription(String,'/team_01/debug/lidar_classification',lambda m:debug.append(json.loads(m.data)),10)
            def spin(kind,duration,objects=(),confidence=1.,light='LIGHTS_OUT',camera=True,frame='team_01/laser_frame'):
                path,left,right=corridor(kind);ranges,angle,increment=ray_scan(left,right,objects,beams=721)
                until=time.monotonic()+duration
                while time.monotonic()<until:
                    stamp=n.get_clock().now().to_msg();header=Header(stamp=stamp,frame_id='team_01/base_link')
                    if camera:
                        for key,points in [('center_path',path),('left_boundary',left),('right_boundary',right)]:pubs[key].publish(path_message(points,header))
                        status=DiagnosticStatus(name='camera_path',message=light)
                        status.values=[KeyValue(key=k,value=v) for k,v in dict(confidence=str(confidence),left_observed='1',right_observed='1',traffic_light=light).items()]
                        m=DiagnosticArray();m.header=Header(stamp=stamp,frame_id='team_01/camera_optical_frame');m.status=[status];meta.publish(m)
                    m=LaserScan();m.header=Header(stamp=stamp,frame_id=frame);m.angle_min=angle;m.angle_max=-angle;m.angle_increment=increment
                    m.range_min=.04;m.range_max=6.;m.ranges=ranges;scan.publish(m)
                    m=Imu();m.header=Header(stamp=stamp,frame_id='team_01/imu_link');m.orientation.w=1.;imu.publish(m)
                    end=time.monotonic()+.04
                    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.001)
            deadline=time.monotonic()+8.
            while time.monotonic()<deadline:
                if (scan.get_subscription_count() >= 1 and meta.get_subscription_count() >= 2 and
                        n.count_publishers('/team_01/debug/mode') >= 1 and n.count_publishers('/team_01/speed') >= 1):
                    break
                rclpy.spin_once(n,timeout_sec=.02)
            self.assertGreaterEqual(meta.get_subscription_count(),2,'Camera packet subscribers did not discover test publishers')
            spin('straight',2.)
            self.assertIn('CENTER',modes);self.assertGreater(speeds[-1],0.)
            for kind in ('left','right','s'):
                obstacles.clear();modes.clear();spin(kind,1.1)
                if not modes or modes[-1]!='CENTER':spin(kind,.6)  # settle after fixture generation / DDS delivery
                self.assertTrue(obstacles)
                self.assertTrue(all(not m.detections for m in obstacles),kind)
                self.assertEqual(modes[-1],'CENTER',(kind,modes[-10:]))
            spin('straight',.3,light='RED')
            obstacles.clear();modes.clear();spin('straight',1.4,objects=[(4.,0.,.3,.3)])
            self.assertTrue(any(m.detections for m in obstacles))
            self.assertTrue(any(m in ('LEFT_PASS','RIGHT_PASS') for m in modes),modes[-20:])
            spin('left',.3,light='RED')
            obstacles.clear();modes.clear();spin('left',1.2,objects=[(1.7,.22*1.7**2,.3,.3)])
            self.assertTrue(any(m.detections for m in obstacles))
            self.assertTrue(any(m in ('LEFT_PASS','RIGHT_PASS','STOP_BLOCKED','WAIT_PASS_PATH','EMERGENCY_STOP') for m in modes))
            spin('straight',.4,light='RED')
            obstacles.clear();spin('straight',.9,objects=[(.45,0.,.3,.3)])
            self.assertTrue(any(m.detections for m in obstacles))
            self.assertEqual(modes[-1],'EMERGENCY_STOP');self.assertEqual(speeds[-1],0.)
            spin('straight',.4,light='RED')
            spin('straight',.8)
            # Leave enough time for the 1 Hz classifier diagnostic as well as
            # the 0.25 s safety timeout; the timeout itself is unchanged.
            spin('straight',1.3,camera=False)
            self.assertIn(modes[-1],('EMERGENCY_STOP','WAIT_PATH'));self.assertEqual(speeds[-1],0.)
            spin('straight',.7)
            spin('straight',.5,frame='missing_lidar_frame')
            self.assertEqual(modes[-1],'EMERGENCY_STOP');self.assertEqual(speeds[-1],0.)
            self.assertTrue(any(d.get('wall_rejected_points',0)>20 for d in debug))
            self.assertTrue(any(d.get('state')=='NO_FRESH_CAMERA_CORRIDOR' for d in debug))
        finally:
            n.destroy_node();rclpy.shutdown()
