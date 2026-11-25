#!/usr/bin/env python3
"""Synthetic ROS safety tests on test-only topics, isolated from the vehicle."""
import argparse,json,os,signal,subprocess,time,struct,math
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Bool,Float32,String
from geometry_msgs.msg import Twist

def run(output,max_speed=.3,path_mode="centerline",config=None):
 rclpy.init();node=rclpy.create_node('autonomy_04_safety_test');procs=[];messages=[];drives=[];states=[];report={};timing={}
 cloud=node.create_publisher(PointCloud2,'/team04/test_auto/points',qos_profile_sensor_data)
 permit=node.create_publisher(Bool,'/team04/test_auto/start',10)
 subs=[node.create_subscription(Float32,'/team04/test_auto/speed',lambda m:messages.append((time.monotonic(),m.data)),10),node.create_subscription(Twist,'/team04/test_auto/drive',lambda m:drives.append((time.monotonic(),m.linear.x)),10),node.create_subscription(String,'/team04/test_auto/state',lambda m:states.append(json.loads(m.data)),10)]
 def launch(executable,extra):
  p=subprocess.Popen(['ros2','run','team04_race',executable,'--ros-args','-p','speed_topic:=/team04/test_auto/speed','-p','steering_topic:=/team04/test_auto/steering','-p','race_start_topic:=/team04/test_auto/start',*extra],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True);procs.append(p);return p
 controller=launch('control_04',['-p','exclusive_command_source:=true','-r','/team04/drive:=/team04/test_auto/drive'])
 auto_args=(["--params-file",str(config)] if config else [])+['-p','path_mode:='+path_mode,'-p','max_speed:='+str(max_speed),'-r','/team04/points:=/team04/test_auto/points','-r','/team04/debug/state:=/team04/test_auto/state']
 auto=launch('autonomy_04',auto_args)
 def make(kind='valid',old=0):
  p=[]
  for ix in range(-26,27):
   for iy in range(-10,11):
    x=ix*.1;y=iy*.1
    if math.hypot(x,y)>.35:p.append((x,y,-.13))
  for ix in range(-30,140):
   x=ix*.05
   for z in [.04,.10,.20,.30]:
    p.append((x,1.2,z-.13))
    if kind!='one_side':p.append((x,-1.2,z-.13))
  if kind=='blocked':
   for ix in range(16,23):
    for iy in range(-15,16):p.append((ix*.05,iy*.02,-.09))
  if kind=='invalid':p=[(float('nan'),float('inf'),0)]*100
  m=PointCloud2();m.header.frame_id='team04/lidar_link';m.header.stamp=(node.get_clock().now()-rclpy.duration.Duration(seconds=old)).to_msg();m.height=1;m.width=len(p);m.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(['x','y','z'])];m.point_step=12;m.row_step=len(p)*12;m.data=b''.join(struct.pack('<fff',*v) for v in p);return m
 def pump(duration,permission=None,kind=None,old=0):
  end=time.monotonic()+duration;last=0
  while time.monotonic()<end:
   if time.monotonic()-last>.05:
    if permission is not None:permit.publish(Bool(data=permission))
    if kind is not None:cloud.publish(make(kind,old))
    last=time.monotonic()
   rclpy.spin_once(node,timeout_sec=.005)
 def check(name,ok):
  report[name]=bool(ok)
  if not ok:raise AssertionError(name)
 try:
  pump(1.5);check('startup_zero',messages and messages[-1][1]==0)
  pump(.5,False,'valid');check('before_permission_zero',drives and drives[-1][1]==0)
  pump(max(1.4,max_speed/.3+1),True,'valid');check('valid_cloud_drives',drives[-1][1]>min(.8*max_speed,2.4))
  # Measure the FIRST zero after loss, rather than the last periodic zero heartbeat.
  loss=time.monotonic();first_index=len(messages);pump(.30)
  zeros=[stamp-loss for stamp,value in messages[first_index:] if value==0]
  timing['lidar_loss_first_zero_s']=min(zeros) if zeros else None
  timing['lidar_loss_last_message_s']=messages[-1][0]-loss
  check('lidar_loss_stops_under_0_25s',bool(zeros) and min(zeros)<.25 and messages[-1][1]==0)
  pump(.4,True,'valid',old=.4);check('stale_stamp_zero',drives[-1][1]==0)
  pump(.4,True,'blocked');check('low_obstacle_zero',drives[-1][1]==0)
  pump(.4,True,'one_side');check('missing_boundary_zero',drives[-1][1]==0)
  pump(.4,True,'invalid');check('nonfinite_cloud_zero',drives[-1][1]==0)
  pump(1.3,True,'valid');check('fresh_recovery',drives[-1][1]>.1)
  other=node.create_publisher(Float32,'/team04/test_auto/speed',10)
  end=time.monotonic()+1
  while time.monotonic()<end:
   other.publish(Float32(data=1.0));pump(.08,True,'valid')
  check('competing_manual_publisher_blocks_drive',drives[-1][1]==0)
  check('conflict_diagnosed',any(s['state']=='COMMAND_PUBLISHER_CONFLICT' for s in states))
  node.destroy_publisher(other);pump(1.5,True,'valid');check('sole_publisher_recovery',drives[-1][1]>.1)
  pump(.2,False,'valid');check('false_immediate_zero',messages[-1][1]==0 and drives[-1][1]==0)
  os.killpg(auto.pid,signal.SIGINT);auto.wait(timeout=5)
  auto=launch('autonomy_04',auto_args)
  pump(1.2,None,'valid');check('restart_needs_new_permission',messages[-1][1]==0 and drives[-1][1]==0)
 finally:
  output.write_text(json.dumps(report,indent=2)+'\n')
  output.with_suffix('.timing.json').write_text(json.dumps(timing,indent=2)+'\n')
  for p in reversed(procs):
   if p.poll() is None:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
  node.destroy_node();rclpy.shutdown()
 print(json.dumps(report,indent=2))
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--output',type=Path,required=True);a.add_argument('--path-mode',choices=['centerline','racing_line'],default='centerline');a.add_argument('--config',type=Path);a.add_argument('--max-speed',type=float,default=.3);args=a.parse_args();os.environ['ROS_DOMAIN_ID']='97';run(args.output,args.max_speed,args.path_mode,args.config)
