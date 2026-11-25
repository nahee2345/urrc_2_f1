"""Foreign global /clock cannot age team_01 data; private clock loss still stops."""
import os
os.environ['ROS_DOMAIN_ID']='159'
import json,time,unittest
from pathlib import Path
import numpy as np
import pytest,launch,launch_ros.actions,launch_testing.actions
import rclpy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from std_msgs.msg import String,Float32

@pytest.mark.launch_test
def generate_test_description():
 p=Path(__file__).resolve().parents[1]
 return launch.LaunchDescription([launch_ros.actions.Node(package='team_01_race',executable='control_adapter_node',name='team_01_control_adapter',
  parameters=[str(p/'config/racing.yaml'),{'primary_steering_source':'lidar','use_sim_time':True,'steering_guard.enabled':False}],
  remappings=[('/clock','/team_01/clock')],output={'both':'log'}),launch_testing.actions.ReadyToTest()])

class TestPrivateClock(unittest.TestCase):
 def test_foreign_clock_ignored_real_clock_watchdog_retained(self):
  rclpy.init();n=rclpy.create_node('clock_isolation_test')
  try:
   good=n.create_publisher(Clock,'/team_01/clock',10);rogue=n.create_publisher(Clock,'/clock',10)
   cmd=n.create_publisher(String,'/team_01/internal/decision_command',10)
   speeds=[];reasons=[];traces=[]
   n.create_subscription(Float32,'/team_01/speed',lambda m:speeds.append(m.data),10)
   n.create_subscription(String,'/team_01/debug/control_stop_reason',lambda m:reasons.append(m.data),10)
   n.create_subscription(String,'/team_01/debug/speed_control',lambda m:traces.append(json.loads(m.data)),10)
   def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.001)
   end=time.monotonic()+8
   while time.monotonic()<end and (good.get_subscription_count()<1 or cmd.get_subscription_count()<1):spin(.02)
   self.assertGreater(good.get_subscription_count(),0)
   x=np.arange(.1,5.01,.1);path=np.column_stack((x,np.zeros(len(x)))).tolist()
   def tick(i,private=True):
    t=190.+i*.02;ns=round(t*1e9);stamp=Time(sec=ns//10**9,nanosec=ns%10**9)
    if private:good.publish(Clock(clock=stamp))
    rogue.publish(Clock(clock=Time(sec=53 if i%2 else 9999)))
    cmd.publish(String(data=json.dumps(dict(stamp=t,path_source='LIDAR_PATH',mode='CENTER',trajectory=path,
      trajectory_header=dict(frame_id='team_01/base_link',sec=stamp.sec,nanosec=stamp.nanosec),
      enabled=True,speed=3.,reason='test_clear',confidence=1.,single_wall=False,corner_braking=False))))
    spin(.025)
   for i in range(180):tick(i)
   self.assertGreater(speeds[-1],2.5)
   self.assertTrue(all(not d['stop'] for d in traces[-25:]),traces[-2:])
   self.assertTrue(all(190.<=d['stamp']<195. for d in traces[-25:]))
   # Foreign clock continues while the selected world actually freezes.
   for i in range(30):tick(180+i,private=False)
   self.assertEqual(speeds[-1],0.)
   self.assertIn('simulation_clock_timeout',reasons)
  finally:n.destroy_node();rclpy.shutdown()
