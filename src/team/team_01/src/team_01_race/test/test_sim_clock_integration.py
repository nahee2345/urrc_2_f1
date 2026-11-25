"""Slow simulation uses simulation seconds; a frozen clock still stops output."""
import os
os.environ['ROS_DOMAIN_ID']='117'
import json,time,unittest
from pathlib import Path
import numpy as np
import pytest,launch,launch_ros.actions,launch_testing.actions
import rclpy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from std_msgs.msg import String,Header,Float32
from nav_msgs.msg import Path as PathMsg
from team_01_race.ros_support import path_message

@pytest.mark.launch_test
def generate_test_description():
 p=Path(__file__).resolve().parents[1]
 return launch.LaunchDescription([launch_ros.actions.Node(package='team_01_race',executable='control_adapter_node',name='team_01_control_adapter',parameters=[str(p/'config/racing.yaml'),{'primary_steering_source':'auto','use_sim_time':True,'steering_guard.enabled':False}]),launch_testing.actions.ReadyToTest()])

class TestSimulationClock(unittest.TestCase):
 def test_slow_clock_acceleration_and_frozen_clock_wall_watchdog(self):
  rclpy.init();n=rclpy.create_node('simulation_clock_test')
  try:
   clock=n.create_publisher(Clock,'/clock',10);path=n.create_publisher(PathMsg,'/team_01/selected_path',10);command=n.create_publisher(String,'/team_01/internal/decision_command',10)
   speeds=[];reasons=[]
   n.create_subscription(Float32,'/team_01/speed',lambda m:speeds.append(m.data),10)
   n.create_subscription(String,'/team_01/debug/control_stop_reason',lambda m:reasons.append(m.data),10)
   def spin(sec):
    end=time.monotonic()+sec
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.002)
   deadline=time.monotonic()+8
   while time.monotonic()<deadline and any(p.get_subscription_count()<1 for p in (clock,path,command)):spin(.02)
   self.assertTrue(all(p.get_subscription_count()>0 for p in (clock,path,command)))
   x=np.arange(.1,5.01,.1);points=np.column_stack((x,np.zeros(len(x))))
   def tick(t):
    stamp=Time(sec=int(t),nanosec=int(round((t-int(t))*1e9)))
    clock.publish(Clock(clock=stamp));path.publish(path_message(points,Header(frame_id='team_01/base_link',stamp=stamp)))
    command.publish(String(data=json.dumps(dict(stamp=t,enabled=True,speed=3.,reason='test_clear',confidence=1.,single_wall=False,corner_braking=False))))
    spin(.055)  # 0.02 simulation seconds per 0.055 wall seconds.
   for i in range(1,22):tick(i*.02)
   self.assertTrue(speeds);self.assertGreater(speeds[-1],.25);self.assertLessEqual(speeds[-1],.45)
   # A newer stop wins over delayed old motion and future motion packets.
   stop=dict(stamp=.43,enabled=False,speed=0.,reason='confirmed_visual_red',confidence=1.,single_wall=False,corner_braking=False)
   command.publish(String(data=json.dumps(stop)));spin(.04)
   for timestamp in (.42,100.):
    command.publish(String(data=json.dumps(dict(stop,stamp=timestamp,enabled=True,speed=3.,reason='replayed_motion'))));spin(.02)
   clock.publish(Clock(clock=Time(sec=0,nanosec=440000000)))
   path.publish(path_message(points,Header(frame_id='team_01/base_link',stamp=Time(sec=0,nanosec=440000000))))
   spin(.08);self.assertEqual(speeds[-1],0.)
   self.assertIn('decision_stop:red_light',reasons)
   spin(.4)
   self.assertEqual(speeds[-1],0.)
   self.assertIn('simulation_clock_timeout',reasons)
   for i in range(22,37):tick(i*.02)
   self.assertGreater(speeds[-1],0.)
  finally:n.destroy_node();rclpy.shutdown()
