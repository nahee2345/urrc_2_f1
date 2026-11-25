"""Synthetic camera/signal safety tests; publishes only test topics."""
import json,subprocess,time,signal,os
import rclpy
from std_msgs.msg import Bool,Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

def run(output):
 rclpy.init();node=rclpy.create_node('control_04_test');out=[];report={}
 sub=node.create_subscription(Twist,'/team04/test/drive',lambda m:out.append(m),10)
 start=node.create_publisher(Bool,'/team04/test/start',10);speed=node.create_publisher(Float32,'/team04/test/speed',10);steer=node.create_publisher(Float32,'/team04/test/steering',10);camera=node.create_publisher(Image,'/team04/test/camera',10)
 def pump(seconds,permission=None,red=None,v=.5,s=0,commands=True):
  out.clear();end=time.monotonic()+seconds;last=0
  while time.monotonic()<end:
   if time.monotonic()-last>.04:
    if permission is not None:start.publish(Bool(data=permission))
    if commands:speed.publish(Float32(data=float(v)));steer.publish(Float32(data=float(s)))
    if red is not None:
     m=Image();m.header.stamp=node.get_clock().now().to_msg();m.width=64;m.height=48;m.step=192;m.encoding='rgb8';m.data=(bytes([0,0,0])*(64*24)+bytes([255,0,0])*(64*24)) if red=='lower' else bytes([255,0,0] if red else [0,0,0])*(64*48);camera.publish(m)
    last=time.monotonic()
   rclpy.spin_once(node,timeout_sec=.01)
  return out[-1] if out else None
 def check(key,predicate):
  report[key]=bool(predicate)
  if not predicate:raise AssertionError(key)
 def launch(mode):
  p=subprocess.Popen(['ros2','run','team04_race','control_04','--ros-args','-p','race_start_mode:='+mode,'-p','race_start_topic:=/team04/test/start','-p','speed_topic:=/team04/test/speed','-p','steering_topic:=/team04/test/steering','-r','/team04/drive:=/team04/test/drive','-r','/team04/camera/image_raw:=/team04/test/camera'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
  pump(1,v=0,commands=False);return p
 def stop(p):
  os.killpg(p.pid,signal.SIGINT);p.wait(timeout=5)
 p=None
 try:
  p=launch('signal')
  check('startup_stopped',pump(.2,commands=False).linear.x==0)
  check('before_permission',pump(.3,v=.5).linear.x==0)
  check('signal_go',pump(.3,True,v=.5,s=.2).linear.x==.5)
  check('positive_is_right',out[-1].angular.z<0)
  check('false_overrides_command',pump(.3,False,v=5).linear.x==0)
  check('reverse',pump(.3,True,v=-1).linear.x==-1)
  check('nan_rejected',pump(.3,True,v=float('nan')).linear.x==0)
  check('invalid_steering_stops',pump(.3,True,v=.5,s=float('inf')).linear.x==0)
  pump(.2,True,v=.5);check('stale_command_stops',pump(.7,commands=False).linear.x==0)
  stop(p);p=launch('camera')
  check('restart_stopped',pump(.2,commands=False).linear.x==0)
  check('initial_dark_not_go',pump(.4,True,False).linear.x==0)
  check('red_barrier_not_signal',pump(.3,True,'lower').linear.x==0)
  check('barrier_then_dark_not_go',pump(.3,True,False).linear.x==0)
  pump(.3,False,True)
  check('red_wait',pump(.2,False,True).linear.x==0)
  check('dark_without_permission',pump(.3,False,False).linear.x==0)
  check('camera_both_conditions',pump(.3,True,False).linear.x==.5)
  check('camera_false_stops',pump(.3,False,False).linear.x==0)
  check('new_cycle_requires_red',pump(.3,True,False).linear.x==0)
  pump(.2,True,True);check('permission_before_dark_waits',out[-1].linear.x==0)
  check('new_red_dark_cycle',pump(.3,True,False).linear.x==.5)
 finally:
  if p:stop(p)
  node.destroy_node();rclpy.shutdown();output.write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2))
