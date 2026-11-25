"""Default pipeline: real stamped scans, Camera preview-only and start gate."""
import os
os.environ['ROS_DOMAIN_ID']='123'
import math,json,time,unittest
from pathlib import Path
import numpy as np
import pytest,launch,launch_ros.actions,launch_testing.actions
import rclpy
from std_msgs.msg import Bool,Float32,String,Header
from sensor_msgs.msg import LaserScan,Imu
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from team_01_race.camera_bundle import encode_bundle

@pytest.mark.launch_test
def generate_test_description():
 root=Path(__file__).resolve().parents[1]
 files=[str(root/'config'/n) for n in ('racing.yaml','path_manager.yaml','corner_control.yaml','steering_guard.yaml','corner_path_recovery.yaml','corner_braking_preview.yaml','depth_corner.yaml','speed_profile_standard.yaml')]
 return launch.LaunchDescription([
  *[launch_ros.actions.Node(package='team_01_race',executable=exe,name=name,parameters=[*files,{'use_sim_time':False,'require_official_start':False}]) for exe,name in [('path_manager_node','team_01_path_manager'),('decision_node','team_01_decision'),('control_adapter_node','team_01_control_adapter'),('lidar_obstacle_node','team_01_lidar_obstacle')]],launch_testing.actions.ReadyToTest()])

class TestPrimary(unittest.TestCase):
 def test_camera_loss_lidar_loss_recovery_and_red_gate(self):
  rclpy.init();n=rclpy.create_node('lidar_primary_test')
  try:
   camera=n.create_publisher(String,'/team_01/camera/geometry_bundle',10)
   scan=n.create_publisher(LaserScan,'/team_01/scan',qos_profile_sensor_data)
   healthy=n.create_publisher(Bool,'/team_01/debug/camera_ready',10)
   far_pub=n.create_publisher(String,'/team_01/camera/braking_preview',10)
   from team_01_race.far_geometry import analyze_preview
   fx=np.arange(.1,24.01,.1);fp=np.column_stack((fx,np.zeros(len(fx))))
   fg=analyze_preview(fp,fp+[0,1.2],fp-[0,1.2],1.)
   fg.update(distance_to_first_significant_curvature=2.,curvature_supported_preview_m=24.,curvature_samples=[dict(distance=2.+.1*i,curvature=.5,wall_agreement=True) for i in range(13)])
   depth=n.create_publisher(String,'/team_01/depth/corner_geometry',10)
   odometry=n.create_publisher(Odometry,'/team_01/odometry',qos_profile_sensor_data)
   imu=n.create_publisher(Imu,'/team_01/imu',qos_profile_sensor_data)
   speeds=[];commands=[];pp=[];classifications=[];speed_traces=[]
   n.create_subscription(String,'/team_01/internal/decision_command',lambda m:commands.append(json.loads(m.data)),10)
   n.create_subscription(String,'/team_01/debug/speed_control',lambda m:speed_traces.append(json.loads(m.data)),10)
   n.create_subscription(String,'/team_01/debug/pp',lambda m:pp.append(json.loads(m.data)),10)
   n.create_subscription(Float32,'/team_01/speed',lambda m:speeds.append(m.data),10)
   n.create_subscription(String,'/team_01/debug/lidar_classification',lambda m:classifications.append(json.loads(m.data)),10)
   x=np.arange(.1,5.01,.1);p=np.column_stack((x,np.zeros(len(x))))
   def frames(count,camera_valid=True,red=False,laser=True,wall=True,far=False):
    for _ in range(count):
     h=Header(frame_id='team_01/base_link',stamp=n.get_clock().now().to_msg())
     # Deliberately disagreeing Camera steering proposal. It must never enter PP.
     cp=p+[0,.6]
     d=dict(path=cp if camera_valid else None,left=cp+[0,1.2] if camera_valid else None,right=cp-[0,1.2] if camera_valid else None,confidence=1. if camera_valid else 0.,observed=(camera_valid,camera_valid),light='RED' if red else 'LIGHTS_OUT',reason='fixture')
     od=Odometry(header=h);od.pose.pose.orientation.w=1.;odometry.publish(od)
     if far:
      depth.publish(String(data=json.dumps(dict(stamp=h.stamp.sec+h.stamp.nanosec*1e-9,depth_valid=True,confidence=.9,depth_corner_candidate=False,level=0,direction=0,distance=None,straight_geometry=True,depth_supported_distance_m=32.,depth_curvature_supported_distance_m=31.,segments=[]))))
      # Deliberately false RGB far corner: Depth straight must retain authority.
      far_pub.publish(String(data=json.dumps(dict(stamp=h.stamp.sec+h.stamp.nanosec*1e-9,frame_id=h.frame_id,confidence=1.,path=fp.tolist(),geometry=fg))))
     camera.publish(String(data=encode_bundle(d,h)));healthy.publish(Bool(data=camera_valid))
     a=Imu(header=Header(frame_id='team_01/imu_link',stamp=h.stamp));a.orientation.w=1.;imu.publish(a)
     if laser:
      s=LaserScan(header=h,angle_min=-math.pi/2,angle_max=math.pi/2,angle_increment=math.pi/360,range_min=.04,range_max=6.)
      angles=s.angle_min+np.arange(361)*s.angle_increment
      ranges=1.2/np.maximum(abs(np.sin(angles)),1e-9)
      ranges[ranges>6.]=math.inf
      if not wall:ranges[:]=math.inf
      elif wall=='single':ranges[angles<0]=math.inf
      s.ranges=ranges.tolist();scan.publish(s)
     end=time.monotonic()+.04
     while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.001)
   end=time.monotonic()+8
   while time.monotonic()<end and (camera.get_subscription_count()<2 or scan.get_subscription_count()<1 or n.count_publishers('/team_01/lidar/synchronized_observation')<1):rclpy.spin_once(n,timeout_sec=.02)
   frames(25,red=True)
   self.assertTrue(speeds);self.assertEqual(speeds[-1],0.)
   frames(40)
   self.assertGreater(speeds[-1],0.)
   normal=[c for c in commands if c['enabled'] and c['speed_trace']['lidar_path_mode']=='NORMAL']
   self.assertTrue(normal);self.assertTrue(all(c['speed_trace']['source_speed_limit']==12. for c in normal))
   self.assertGreater(max(c['speed'] for c in normal),2.)
   frames(275,far=True)
   # 32m measured visibility supports the requested 12m/s ceiling.
   self.assertGreater(max(c['speed'] for c in commands),11.8)
   self.assertGreater(max(speeds),11.8)
   self.assertTrue(any(v.get('source_speed_limit')==12. and v.get('final_target_speed',0)>11.8 and v.get('final_speed',0)>11.8 for v in speed_traces))
   self.assertTrue(any(c['enabled'] and c['path_source']=='LIDAR_PATH' for c in commands))
   self.assertLess(max(abs(v['target_y']) for v in pp[-20:] if v.get('target_y') is not None),.1)
   frames(25,camera_valid=False)
   self.assertGreater(speeds[-1],0.)
   self.assertEqual(commands[-1]['speed_trace']['source_speed_limit'],12.)
   self.assertLessEqual(commands[-1]['speed'],3.5)  # Missing Depth preview has its own conservative band, never a LiDAR cap.
   self.assertEqual(commands[-1]['path_source'],'LIDAR_PATH')
   frames(10,wall='single')
   self.assertEqual(commands[-1]['speed_trace']['lidar_path_mode'],'SINGLE_WALL_FALLBACK')
   self.assertLessEqual(commands[-1]['speed_trace']['source_speed_limit'],10.)
   frames(20)
   # Valid scans but no wall support: brief LiDAR memory, then STOP despite Camera.
   before=len(commands);frames(5,wall=False)
   held=[c for c in commands[before:] if c['enabled'] and c['path_source']=='PREVIOUS_LIDAR_PATH']
   self.assertTrue(held)
   self.assertTrue(all(c['speed']<=1.2 and c['speed_trace']['source_speed_limit']<=1.2 for c in held))
   frames(18,wall=False)
   self.assertFalse(commands[-1]['enabled']);self.assertEqual(speeds[-1],0.)
   frames(45)
   self.assertGreater(speeds[-1],0.)
   self.assertEqual(commands[-1]['path_source'],'LIDAR_PATH')
   frames(18,laser=False)
   self.assertFalse(commands[-1]['enabled']);self.assertEqual(speeds[-1],0.)
   frames(45);self.assertGreater(speeds[-1],0.)
   frames(6,red=True);self.assertEqual(speeds[-1],0.)
   self.assertFalse(any(c['enabled'] and c['path_source'] in ('CAMERA_PATH','FUSED_PATH') for c in commands))
   self.assertTrue(any(d.get('geometry_source')=='LIDAR_PATH' for d in classifications))
  finally:n.destroy_node();rclpy.shutdown()
