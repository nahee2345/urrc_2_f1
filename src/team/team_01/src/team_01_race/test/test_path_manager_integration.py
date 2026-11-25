"""Real router/Decision/Control/obstacle nodes with stamped wall scans."""
import os
os.environ['ROS_DOMAIN_ID']='119'
import math,json,time,unittest
from pathlib import Path
import numpy as np
import pytest,launch,launch_ros.actions,launch_testing.actions
import rclpy
from std_msgs.msg import Bool,Float32,String,Header
from sensor_msgs.msg import LaserScan,Imu
from nav_msgs.msg import Path as PathMsg
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from team_01_race.camera_bundle import encode_bundle

@pytest.mark.launch_test
def generate_test_description():
 root=Path(__file__).resolve().parents[1]
 files=[str(root/'config'/n) for n in ('racing.yaml','corner_control.yaml','steering_guard.yaml','corner_path_recovery.yaml','corner_braking_preview.yaml')]
 return launch.LaunchDescription([
  *[launch_ros.actions.Node(package='team_01_race',executable=exe,name=name,parameters=[*files,{'primary_steering_source':'auto','depth_corner.enabled':False,'use_sim_time':False,'require_official_start':False}]) for exe,name in [('path_manager_node','team_01_path_manager'),('decision_node','team_01_decision'),('control_adapter_node','team_01_control_adapter'),('lidar_obstacle_node','team_01_lidar_obstacle')]],launch_testing.actions.ReadyToTest()])

class TestManager(unittest.TestCase):
 def test_lidar_fallback_red_obstacle_sensor_loss_and_reacquisition(self):
  rclpy.init();n=rclpy.create_node('path_manager_test')
  try:
   camera=n.create_publisher(String,'/team_01/camera/geometry_bundle',10)
   scan=n.create_publisher(LaserScan,'/team_01/scan',qos_profile_sensor_data)
   healthy=n.create_publisher(Bool,'/team_01/debug/camera_ready',10)
   legacy_selected=n.create_publisher(PathMsg,'/team_01/selected_path',10)
   imu=n.create_publisher(Imu,'/team_01/imu',qos_profile_sensor_data)
   speeds=[];sources=[];reasons=[];controls=[];classifications=[]
   n.create_subscription(String,'/team_01/debug/lidar_classification',lambda m:classifications.append(json.loads(m.data)),10)
   n.create_subscription(String,'/team_01/debug/corner_control',lambda m:controls.append(json.loads(m.data)),10)
   n.create_subscription(Float32,'/team_01/speed',lambda m:speeds.append(m.data),10)
   n.create_subscription(String,'/team_01/debug/path_manager',lambda m:sources.append(json.loads(m.data)['source']),10)
   n.create_subscription(String,'/team_01/debug/control_stop_reason',lambda m:reasons.append(m.data),10)
   x=np.arange(.1,5.01,.1);p=np.column_stack((x,np.zeros(len(x))))
   def frames(count,valid_camera=True,red=False,laser=True,obstacle=False,alive=True,empty_legacy_path=False,camera_delay=0.):
    for _ in range(count):
     h=Header(frame_id='team_01/base_link',stamp=n.get_clock().now().to_msg())
     d=dict(path=p if valid_camera else None,left=p+[0,1.2] if valid_camera else None,right=p-[0,1.2] if valid_camera else None,confidence=1. if valid_camera else 0.,observed=(valid_camera,valid_camera),light='RED' if red else 'LIGHTS_OUT',reason='test')
     camera_header=Header(frame_id=h.frame_id,stamp=Time(nanoseconds=n.get_clock().now().nanoseconds-int(camera_delay*1e9)).to_msg())
     camera.publish(String(data=encode_bundle(d,camera_header)));healthy.publish(Bool(data=alive))
     if empty_legacy_path:legacy_selected.publish(PathMsg(header=h))
     a=Imu(header=h);a.orientation.w=1.;imu.publish(a)
     if laser:
      s=LaserScan(header=h,angle_min=-math.pi/2,angle_max=math.pi/2,angle_increment=math.pi/360,range_min=.04,range_max=6.)
      angles=s.angle_min+np.arange(361)*s.angle_increment
      ranges=np.minimum(1.2/np.maximum(abs(np.sin(angles)),1e-9),1e9)
      ranges[ranges>6.]=math.inf
      if obstacle:
       hit=(abs(np.tan(angles)*.5)<.15)&(np.cos(angles)>.1);ranges[hit]=.5/np.cos(angles[hit])
      s.ranges=ranges.tolist();scan.publish(s)
     end=time.monotonic()+.04
     while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.001)
   end=time.monotonic()+8
   while time.monotonic()<end and (camera.get_subscription_count()<1 or scan.get_subscription_count()<2 or imu.get_subscription_count()<2 or n.count_publishers('/team_01/speed')<1):rclpy.spin_once(n,timeout_sec=.02)
   frames(40);self.assertIn('CAMERA_PATH',sources);self.assertGreater(speeds[-1],0.)
   before=len(reasons);frames(12,empty_legacy_path=True)
   self.assertGreater(speeds[-1],0.)
   self.assertNotIn('invalid_selected_path_or_speed',reasons[before:])
   before=len(sources)
   frames(25,camera_delay=.30)
   self.assertIn('LIDAR_PATH',sources[before:])
   self.assertGreater(speeds[-1],0.)
   self.assertLessEqual(speeds[-1],2.00001)
   self.assertTrue(any(d.get('state')=='LIDAR_CORRIDOR' or d.get('geometry_source')=='LIDAR_PATH' for d in classifications))
   frames(20)
   self.assertEqual(sources[-1],'CAMERA_PATH')
   # A fresh geometry packet alone must not restore a Camera that is still
   # warming up after an image timeout; the measured LiDAR remains available.
   before=len(sources)
   frames(15,alive=False)
   self.assertIn('LIDAR_PATH',sources[before:])
   self.assertGreater(speeds[-1],0.)
   frames(20)
   self.assertEqual(sources[-1],'CAMERA_PATH')
   frames(35,False,alive=False);self.assertIn('LIDAR_PATH',sources);self.assertGreater(speeds[-1],0.);self.assertLessEqual(speeds[-1],2.00001)
   frames(8,False,red=True);self.assertEqual(speeds[-1],0.)
   frames(35,False);self.assertGreater(speeds[-1],0.)
   frames(12,False,obstacle=True)
   critical=[c for c in controls if 'FRONT_OBSTACLE_0P5M' in c['stop_reason']]
   self.assertTrue(critical);self.assertTrue(all(c['final_speed']==0. for c in critical))
   # A stationary test cannot prove rear clearance after removing the object.
   frames(8,red=True);frames(30);frames(18,False,laser=False);self.assertEqual(speeds[-1],0.)
   frames(35);self.assertGreater(speeds[-1],0.)
  finally:n.destroy_node();rclpy.shutdown()
