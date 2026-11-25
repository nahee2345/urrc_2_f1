#!/usr/bin/env python3
"""Test-only ROS observer/command source; requires isolated team04_test world.
Never launch against the competition. Publishes test start permission only here.
"""
import argparse,json,time,math,subprocess,signal
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,LaserScan,Imu,JointState,Image
from std_msgs.msg import Float32,Bool
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

def main():
 a=argparse.ArgumentParser();a.add_argument('--output',type=Path,required=True);a.add_argument('--all',action='store_true');a.add_argument('--logic',action='store_true');args=a.parse_args()
 if args.logic:
  from control_checks import run
  return run(args.output)
 fixture=[]
 if args.all:fixture=setup_fixture(args.output.parent)
 bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/model/team04/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],stdout=subprocess.DEVNULL,start_new_session=True)
 rclpy.init();n=rclpy.create_node('team04_runtime_probe');samples={k:[] for k in ['points','scan','imu','joints','camera','drive','poses']}
 def save(k,m):samples[k].append((time.monotonic(),m))
 subs=[]
 for k,typ,topic in [('points',PointCloud2,'/team04/points'),('scan',LaserScan,'/team04/scan'),('imu',Imu,'/team04/imu'),('joints',JointState,'/team04/wheel_joint_states'),('camera',Image,'/team04/camera/image_raw'),('drive',Twist,'/team04/drive'),('poses',TFMessage,'/model/team04/pose')]:
  subs.append(n.create_subscription(typ,topic,lambda m,k=k:save(k,m),qos_profile_sensor_data))
 speed=n.create_publisher(Float32,'/team04/speed',10);steer=n.create_publisher(Float32,'/team04/steering',10);start=n.create_publisher(Bool,'/race/start/team04',10)
 def phase(duration,v=0,s=0,permit=None):
  samples.clear();samples.update({k:[] for k in ['points','scan','imu','joints','camera','drive','poses']})
  deadline=time.monotonic()+duration;last=0
  while time.monotonic()<deadline:
   if time.monotonic()-last>.05:
    if permit is not None:start.publish(Bool(data=permit))
    speed.publish(Float32(data=float(v)));steer.publish(Float32(data=float(s)));last=time.monotonic()
   rclpy.spin_once(n,timeout_sec=.01)
  out={}
  for k in ['points','scan','imu','joints','camera']:
   seq=samples[k];info={'count':len(seq)}
   if len(seq)>2:
    stamps=[m.header.stamp.sec+m.header.stamp.nanosec/1e9 for _,m in seq];span=stamps[-1]-stamps[0]
    info['hz_sim']=(len(seq)-1)/span if span>0 else 0
    info['hz_wall']=(len(seq)-1)/(seq[-1][0]-seq[0][0]);info['frame']=seq[-1][1].header.frame_id
   out[k]=info
  if samples['points']:
   m=samples['points'][-1][1];out['points'].update(width=m.width,height=m.height,fields=[f.name for f in m.fields])
   import struct
   fields={f.name:f.offset for f in m.fields};near=[]
   for row in range(m.height):
    for col in range(m.width):
     p=row*m.row_step+col*m.point_step;xyz=[struct.unpack_from('<f',m.data,p+fields[f])[0] for f in ['x','y','z']]
     if all(math.isfinite(v) for v in xyz) and 0<xyz[0]<.4 and abs(xyz[1])<.002:near.append(xyz)
   out['points']['near_front_returns']=near[:40]
  if samples['camera']:
   m=samples['camera'][-1][1];out['camera'].update(width=m.width,height=m.height,encoding=m.encoding)
  if samples['scan']:
   m=samples['scan'][-1][1];ranges=[v for v in m.ranges if math.isfinite(v)];out['scan'].update(bins=len(m.ranges),min_range=min(ranges) if ranges else None,front=m.ranges[359])
  if samples['joints']:
   m=samples['joints'][-1][1];out['joints'].update(names=list(m.name),position=list(m.position),velocity=list(m.velocity),max_abs_steering=max((abs(v) for _,j in samples['joints'] for v in j.position[2:]),default=0),tail_max_steering_velocity=max((abs(v) for _,j in samples['joints'][-10:] for v in j.velocity[2:]),default=0))
  if samples['imu']:
   seq=[m for _,m in samples['imu']];out['imu'].update(mean_yaw_rate=sum(m.angular_velocity.z for m in seq)/len(seq),last_accel=[seq[-1].linear_acceleration.x,seq[-1].linear_acceleration.y,seq[-1].linear_acceleration.z])
  if samples['drive']:out['drive']={'last_speed':samples['drive'][-1][1].linear.x,'last_steering':samples['drive'][-1][1].angular.z,'max_speed':max(abs(m.linear.x) for _,m in samples['drive']), 'tail_max_speed':max(abs(m.linear.x) for _,m in samples['drive'][-10:])}
  poses=[]
  for _,msg in samples['poses']:
   for t in msg.transforms:
    if t.child_frame_id=='team04':poses.append(t)
  if poses:
   def xyz(t):return [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
   out['pose']={'first':xyz(poses[0]),'last':xyz(poses[-1]),'z_range':[min(t.transform.translation.z for t in poses),max(t.transform.translation.z for t in poses)]}
  elif samples['poses']:out['pose_names']=[t.child_frame_id for t in samples['poses'][-1][1].transforms][:20]
  return out
 report={}
 try:
  phase(2,permit=False)
  report['prestart']=phase(2,.5,0,False)
  report['forward']=phase(2,.5,0,True)
  report['right']=phase(2,.5,.5,True)
  report['left']=phase(2,.5,-.5,True)
  report['reverse']=phase(1,-.4,0,True)
  report['limit']=phase(1,0,2,True)
  report['brake_approach']=phase(1,.5,0,True)
  report['stop']=phase(2,.5,0,False)
  report['invalid']=phase(1,float('nan'),float('inf'),True)
  if fixture:
   import os,re
   phase(1,.4,0,True)
   log=(args.output.parent/'launch_test.log').read_text()
   pid=int(re.search(r'\[control_04-\d+\]: process started with pid \[(\d+)\]',log).group(1))
   os.kill(pid,signal.SIGSTOP)
   try:report['plugin_watchdog']=phase(1,.4,0,None)
   finally:os.kill(pid,signal.SIGCONT)
  start.publish(Bool(data=False));rclpy.spin_once(n,timeout_sec=.1)
 finally:
  args.output.write_text(json.dumps(report,indent=2,allow_nan=True)+'\n');n.destroy_node();rclpy.shutdown();os_kill(bridge)
  for proc in reversed(fixture):os_kill(proc)
 print(json.dumps({k:{'hz':v['points'].get('hz_sim'),'front':v.get('scan',{}).get('front'),'yaw':v.get('imu',{}).get('mean_yaw_rate'),'pose':v.get('pose'),'joints':v.get('joints',{}).get('velocity')} for k,v in report.items()},indent=2))
def os_kill(p):
 import os
 os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=5)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
def setup_fixture(folder):
 import xml.etree.ElementTree as E
 from ament_index_python.packages import get_package_share_directory
 share=Path(get_package_share_directory('team04_race'))
 root=E.parse(share/'models/team04_car/model.sdf');model=root.getroot().find('model')
 plugin=E.SubElement(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
 for key,value in dict(publish_model_pose='true',publish_link_pose='true',publish_nested_model_pose='true',use_pose_vector_msg='true',static_publisher='false',update_frequency='100',topic='/model/team04/pose').items():E.SubElement(plugin,key).text=value
 modelpath=folder/'probe_model.sdf';root.write(modelpath)
 gzlog=(folder/'gazebo_test.log').open('w');launchlog=(folder/'launch_test.log').open('w')
 server=subprocess.Popen(['gz','sim','-s','-r','--headless-rendering',str(share/'config/test_world.sdf')],stdout=gzlog,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  for _ in range(30):
   if server.poll() is not None:raise RuntimeError('Gazebo failed: see gazebo_test.log')
   result=subprocess.run(['gz','service','-l'],capture_output=True,text=True,timeout=5)
   if '/world/team04_test/create' in result.stdout:break
   time.sleep(.3)
  else:raise RuntimeError('Gazebo services unavailable')
  launch=subprocess.Popen(['ros2','launch','team04_race','test_vehicle.launch.py','model_file:='+str(modelpath.resolve())],stdout=launchlog,stderr=subprocess.STDOUT,start_new_session=True)
  time.sleep(3)
  return [server,launch]
 except BaseException:
  os_kill(server);raise
if __name__=='__main__':main()
