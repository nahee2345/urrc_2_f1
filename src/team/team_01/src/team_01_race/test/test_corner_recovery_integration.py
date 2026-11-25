"""Actual Decision/Control/LiDAR nodes: path faults, live scans, detailed reasons."""
import os
os.environ['ROS_DOMAIN_ID']='110'
import math,json,time,unittest
from pathlib import Path
import numpy as np
import pytest,launch,launch_ros.actions,launch_testing.actions
import rclpy
from std_msgs.msg import Bool,Float32,String,Header
from sensor_msgs.msg import LaserScan,Imu
from nav_msgs.msg import Path as PathMsg
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from rclpy.qos import qos_profile_sensor_data
from team_01_race.ros_support import path_message


@pytest.mark.launch_test
def generate_test_description():
 p=Path(__file__).resolve().parents[1]
 files=[str(p/'config'/n) for n in ('racing.yaml','corner_control.yaml','steering_guard.yaml','corner_path_recovery.yaml')]
 return launch.LaunchDescription([
  *[launch_ros.actions.Node(package='team_01_race',executable=executable,name=name,parameters=[*files,{'primary_steering_source':'auto','depth_corner.enabled':False,'use_sim_time':False,'require_official_start':False}]) for executable,name in [('decision_node','team_01_decision'),('control_adapter_node','team_01_control_adapter'),('lidar_obstacle_node','team_01_lidar_obstacle')]],
  launch_testing.actions.ReadyToTest()])


class TestCornerRecovery(unittest.TestCase):
 def test_commit_short_losses_reacquire_timeout_obstacle_reverse_and_resume(self):
  rclpy.init();n=rclpy.create_node('corner_recovery_test')
  try:
   pubs={k:n.create_publisher(PathMsg,'/team_01/camera/'+k,10) for k in ('center_path','left_boundary','right_boundary')}
   meta=n.create_publisher(DiagnosticArray,'/team_01/camera/path_status',10)
   healthy=n.create_publisher(Bool,'/team_01/debug/camera_ready',10);image_age=n.create_publisher(Float32,'/team_01/debug/camera_image_age',10)
   scan=n.create_publisher(LaserScan,'/team_01/scan',qos_profile_sensor_data);imu=n.create_publisher(Imu,'/team_01/imu',qos_profile_sensor_data)
   controls=[];n.create_subscription(String,'/team_01/debug/corner_control',lambda m:controls.append(json.loads(m.data)),10)
   speeds=[];requests=[];contexts=[];debug=[];reasons=[];lidar=[];targets=[]
   n.create_subscription(Float32,'/team_01/speed',lambda m:speeds.append(m.data),10)
   n.create_subscription(Float32,'/team_01/internal/requested_speed',lambda m:requests.append(m.data),10)
   n.create_subscription(String,'/team_01/internal/corner_path_context',lambda m:contexts.append(json.loads(m.data)),10)
   n.create_subscription(String,'/team_01/debug/decision_debug',lambda m:debug.append(json.loads(m.data)),10)
   n.create_subscription(String,'/team_01/debug/control_stop_reason',lambda m:reasons.append(m.data),10)
   n.create_subscription(String,'/team_01/debug/lidar_classification',lambda m:lidar.append(json.loads(m.data)),10)
   x=np.arange(.1,5.01,.1);left=np.column_stack((x,.175*x*x));right=left.copy();right[:,1]*=-1
   def frame(kind='good',obstacle=None,alive=True,laser=True):
    header=Header(frame_id='team_01/base_link',stamp=n.get_clock().now().to_msg())
    bad=kind in ('lost','reverse');center=right.copy() if kind=='jump' else left.copy()
    if kind=='reverse':center[:,0]=-x
    if kind=='lost':center=np.empty((0,2))
    confidence=.68 if kind=='single' else 0. if bad else 1.
    observed=('1','0') if kind=='single' else ('0','0') if bad else ('1','1')
    for key,p in [('center_path',center),('left_boundary',center+[0.,1.2]),('right_boundary',center-[0.,1.2])]:pubs[key].publish(path_message(p,header))
    values=dict(confidence=str(confidence),left_observed=observed[0],right_observed=observed[1],traffic_light='LIGHTS_OUT',path_reason=kind)
    d=DiagnosticArray();d.header=header;d.status=[DiagnosticStatus(name='camera_path',values=[KeyValue(key=k,value=v) for k,v in values.items()])];meta.publish(d)
    healthy.publish(Bool(data=alive));image_age.publish(Float32(data=0. if alive else .5))
    attitude=Imu();attitude.header=Header(frame_id='team_01/imu_link',stamp=header.stamp);attitude.orientation.w=1.;imu.publish(attitude)
    if laser:
     s=LaserScan();s.header=header;s.angle_min=-math.pi/2;s.angle_max=math.pi/2;s.angle_increment=math.pi/180;s.range_min=.04;s.range_max=6.;s.ranges=[math.inf]*181
     if obstacle is not None:
      cy=.175*obstacle*obstacle
      for i in range(181):
       a=s.angle_min+i*s.angle_increment
       if math.cos(a)>.01 and abs(obstacle*math.tan(a)-cy)<.15:s.ranges[i]=obstacle/math.cos(a)
     scan.publish(s)
    end=time.monotonic()+.04
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.001)
   def frames(count,*a,**kw):
    for _ in range(count):frame(*a,**kw)
   end=time.monotonic()+8
   while time.monotonic()<end and pubs['center_path'].get_subscription_count()<3:rclpy.spin_once(n,timeout_sec=.02)
   frames(90)
   self.assertTrue(speeds);self.assertGreater(speeds[-1],2.3)
   self.assertTrue(any(d['corner_state']=='CORNER_COMMIT' for d in debug))
   for count in (1,3):
    start=len(speeds);frames(count,'lost')
    self.assertTrue(any(v.get('holding') for v in contexts[-5:]))
    self.assertAlmostEqual(requests[-1],1.2,places=4)
    self.assertTrue(all(v>0 for v in speeds[start:]),speeds[start:])
    frames(6)
    self.assertTrue(all(v>0 for v in speeds[start:]),speeds[start:])
    self.assertFalse(contexts[-1].get('holding'))
   frames(1,'jump');self.assertTrue(contexts[-1].get('holding'));frames(6)
   frames(4,'single')
   self.assertGreater(speeds[-1],0.)
   self.assertTrue(any(d.get('single_wall_fallback') for d in debug))
   frames(8)
   frames(8,'lost')  # > old .25 geometry timeout, < new .40 hold.
   self.assertAlmostEqual(requests[-1],1.2,places=4)
   self.assertGreater(speeds[-1],0.)
   self.assertTrue(any(d['state']=='CORNER_PATH_HOLD_CORRIDOR' for d in lidar))
   frames(8,'lost')
   self.assertEqual(speeds[-1],0.)
   self.assertIn('decision_stop:path_lost_timeout',reasons)
   frames(10);self.assertGreater(speeds[-1],0.)
   frames(10)  # Resume enough motion to exercise a genuine approaching TTC.
   for distance in (.8,.6,.4):frame('lost',obstacle=distance)
   frames(2,'lost',obstacle=.4)
   self.assertEqual(speeds[-1],0.)
   self.assertTrue(any('FRONT_OBSTACLE_0P5M' in s for s in reasons),reasons[-20:])
   frames(12)
   frames(4,'reverse')
   self.assertEqual(speeds[-1],0.)
   self.assertTrue(any('reverse_risk_persistent' in s for s in reasons))
   frames(12);self.assertGreater(speeds[-1],0.)
   frames(2,'lost',alive=False)
   self.assertEqual(speeds[-1],0.)
   self.assertTrue(any('decision_stop:camera_timeout' in s for s in reasons))
   frames(12);self.assertGreater(speeds[-1],0.)
   report=Path(os.environ.get('TEAM_01_RECOVERY_TEST_OUTPUT','/tmp/team_01-corner-recovery-integration.json'))
   self.assertTrue(controls)
   self.assertTrue(all(c['final_speed']<=c['requested_speed']+1e-6 for c in controls if c['corner_braking'] or c['using_previous_trajectory']))
   self.assertTrue(all(c['stop']==bool(c['stop_reason']) for c in controls))
   report.write_text(json.dumps(dict(controls=controls,debug=debug,reasons=reasons,requested=requests,speeds=speeds,lidar=lidar),indent=2))
  finally:n.destroy_node();rclpy.shutdown()
