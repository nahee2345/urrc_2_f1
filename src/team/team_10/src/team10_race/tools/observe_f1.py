#!/usr/bin/env python3
"""Read-only chain recorder. Ground truth is NEVER published to a driving topic."""
import json,time,subprocess,threading,math,os
from pathlib import Path
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,LaserScan
from std_msgs.msg import Bool,Float32,String
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

class Observe(Node):
 def __init__(self):
  super().__init__('f1_observer')
  self.folder=Path(__file__).resolve().parents[4]/'validation_f1';self.folder.mkdir(exist_ok=True)
  chain=self.folder/'chain.jsonl'
  if chain.exists():chain.rename(self.folder/f'chain_{time.time_ns()}.jsonl')
  self.file=chain.open('w',buffering=1)
  self.latest={};self.count={};self.received={};self.start=time.monotonic();self.image=None
  self.light=os.environ.get('HANLA_SPEED_LIGHT_RECORDING')=='1'
  for topic,kind in [('/clock',Clock),('/team_10/scan_10',LaserScan),('/team_10/start_camera_image_10',Image),('/race/start/team_10',Bool),('/team_10/camera_gate_10',Bool),('/team_10/start_status_10',String),('/team_10/speed_10',Float32),('/team_10/steering_10',Float32),('/team_10/gz_cmd_vel_10',Twist),('/team_10/validation_10/poses',TFMessage)]:
   self.create_subscription(kind,topic,lambda m,t=topic:self.cb(t,m),qos_profile_sensor_data)
  self.gz=None
  if not self.light:
   self.gz=subprocess.Popen(['gz','topic','-e','-t','/world/monaco/pose/info','--json-output'],stdout=subprocess.PIPE,text=True)
   threading.Thread(target=self.poses,daemon=True).start()
  self.create_timer(.2,self.report)
 def poses(self):
  for line in self.gz.stdout:
   try:
    m=json.loads(line)
    x=next(p for p in m['pose'] if p.get('name')=='team_10');p=x['position'];q=x['orientation']
    yaw=math.atan2(2*(q.get('w',0)*q.get('z',0)+q.get('x',0)*q.get('y',0)),1-2*(q.get('y',0)**2+q.get('z',0)**2))
    t='ground_truth_measurement_only'
    self.latest[t]=[p.get('x',0),p.get('y',0),p.get('z',0),yaw]
    self.count[t]=self.count.get(t,0)+1;self.received[t]=time.monotonic()
   except (ValueError,KeyError,StopIteration):pass
 def cb(self,t,m):
  self.count[t]=self.count.get(t,0)+1;self.received[t]=time.monotonic()
  if isinstance(m,Clock):v=m.clock.sec+m.clock.nanosec*1e-9
  elif isinstance(m,Image):
   v={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'encoding':m.encoding,'size':[m.width,m.height]}
   if self.light:
    self.latest[t]=v
    return
   a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
   self.image=cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else a.copy()
   if self.count[t]%10==0:cv2.imwrite(str(self.folder/'camera_latest.png'),self.image)
   if self.count[t]%20==0:cv2.imwrite(str(self.folder/f'frame_{time.time():.3f}.png'),self.image)
  elif isinstance(m,LaserScan):v={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'n':len(m.ranges),'finite':sum(np.isfinite(m.ranges))}
  elif isinstance(m,Twist):v=[m.linear.x,m.angular.z]
  elif isinstance(m,TFMessage):
   poses=[x for x in m.transforms if x.child_frame_id=='team_10']
   if not poses:return
   x=poses[0];q=x.transform.rotation;p=x.transform.translation
   import math
   v=[p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
  else:v=m.data
  self.latest[t]=v
 def report(self):
  row={'wall':time.time(),'elapsed':time.monotonic()-self.start,'values':self.latest,'counts':self.count,'ages':{k:round(time.monotonic()-v,3) for k,v in self.received.items()}}
  self.file.write(json.dumps(row,default=lambda x:int(x))+'\n')
  if int(row['elapsed']*5)%10==0:print(json.dumps(row,default=lambda x:int(x)),flush=True)
rclpy.init();n=Observe()
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:
 if n.gz:n.gz.terminate()
 n.file.close();n.destroy_node();rclpy.try_shutdown()
