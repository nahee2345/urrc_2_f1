"""Isolated physical test actors. Ground truth NEVER goes to the ego algorithm."""
from pathlib import Path
import json,math,xml.etree.ElementTree as ET
import numpy as np
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data

class TrafficFixture:
 def __init__(self,root,out,share,track,scenario):
  self.root,self.out,self.share,self.track,self.scenario=root,out,share,track,scenario
  self.reference=np.genfromtxt(root/f'src/urrc_track_gazebo/tracks/processed/{track}_centerline.csv',delimiter=',',names=True)
  self.xy=np.stack([self.reference['x_m'],self.reference['y_m']],axis=1)
  self.s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(self.xy,axis=0),axis=1))]
  self.actors=[];self.samples=[];self.commands=[];self.last_tick=-1;self.node=None
  self.start_s=30.
  if scenario=='corner':
   k=np.abs(np.gradient(np.unwrap(self.reference['yaw_rad']),self.s));eligible=np.flatnonzero((self.s>30)&(k>.08));self.start_s=max(5.,self.s[eligible[0]]-5) if len(eligible) else 80.
  layouts={'solo':[],'stopped':[(5.,0,0)],'follow':[(5.5,0,.4)],'follow_offset':[(4.,-.45,.4)],'left':[(3.8,-.45,.25)],'left_wide':[(3.8,-.7,.25)],'left_clear':[(5.,-.85,.25)],'left_static':[(5.,-.85,0.)],'right':[(3.8,.45,.25)],'right_wide':[(3.8,.7,.25)],'right_clear':[(5.,.85,.25)],'right_static':[(5.,.85,0.)],'blocked':[(3.8,-.35,.35),(3.8,.35,.35)],'brake':[(4.,-.45,.6)],'corner':[(3.8,-.45,.35)],'multi':[(3.8,-.6,.3),(4.8,.6,.4),(6.,-.2,.3)],'occlusion':[(3.8,-.6,.3),(5.,.6,.3)],'cutin':[(3.8,-.45,.3)]}
  for i,(distance,offset,speed) in enumerate(layouts[scenario]):self.actors.append(dict(name=f'team04_test_actor_{i}',distance=distance,offset=offset,speed=speed,poses=[],last_steering=0))
 def slot(self,s,offset=0):
  i=int(np.argmin(abs(self.s-s)));p=self.reference[i];a=float(p['yaw_rad']);return dict(x=float(p['x_m'])-math.sin(a)*offset,y=float(p['y_m'])+math.cos(a)*offset,road_z=float(p['z_m']),yaw=a)
 def spawn(self,official,env):
  for actor in self.actors:
   tree=ET.parse(self.share/'models/team04_car/model.sdf');m=tree.getroot().find('model')
   # Actor-only sensor removal prevents publishing on ego topics. All links,
   # collisions, masses, joints and force limits remain the same physical model.
   for link in m.findall('link'):
    for sensor in list(link.findall('sensor')):link.remove(sensor)
   for plugin in list(m.findall('plugin')):m.remove(plugin)
   p=ET.SubElement(m,'plugin',filename='libteam04_traffic_actor.so',name='team04_test::TrafficActorSystem')
   actor['topic']=f"/team04/test/{actor['name']}/drive"
   ET.SubElement(p,'drive_topic').text=actor['topic'];ET.SubElement(p,'state_topic').text=f"/team04/test/{actor['name']}/joints"
   path=self.out/(actor['name']+'.sdf');tree.write(path)
   entry=dict(entry_id=actor['name'],_model=str(path),base_link_name='base_link',spawn_z_offset_m=.04)
   official.spawn(self.root,self.track,entry,self.slot(self.start_s+actor['distance'],actor['offset']),env)
   actor['entry']=entry
 def bridge_arguments(self):
  args=[]
  for actor in self.actors:args += [f"/model/{actor['name']}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",actor['topic']+'@geometry_msgs/msg/Twist]gz.msgs.Twist']
  return args
 def attach(self,node):
  self.node=node;self.subs=[]
  for actor in self.actors:
   actor['publisher']=node.create_publisher(Twist,actor['topic'],1)
   def callback(m,actor=actor):
    for t in m.transforms:
     if t.child_frame_id==actor['name']:
      v=t.transform.translation;q=t.transform.rotation;stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
      if not actor['poses'] or stamp-actor['poses'][-1][0]>=.049:actor['poses'].append([stamp,v.x,v.y,v.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
   self.subs.append(node.create_subscription(TFMessage,f"/model/{actor['name']}/pose",callback,qos_profile_sensor_data))
 def tick(self,sim,release):
  if release is None or sim-self.last_tick<.049:return
  dt=.05 if self.last_tick<0 else min(.2,sim-self.last_tick);self.last_tick=sim;elapsed=sim-release
  for i,actor in enumerate(self.actors):
   speed=actor['speed'];offset=actor['offset'];steering=0
   if self.scenario=='brake' and elapsed>12:speed=0
   if self.scenario in ('occlusion','cutin') and i==0 and elapsed>8:offset=.1 if self.scenario=='occlusion' else .45
   if actor['poses']:
    _,x,y,_,yaw=actor['poses'][-1];idx=int(np.argmin(np.sum((self.xy-[x,y])**2,axis=1)));target=self.slot(self.s[idx]+1.4,offset);dx=target['x']-x;dy=target['y']-y;tx=math.cos(yaw)*dx+math.sin(yaw)*dy+.16;ty=-math.sin(yaw)*dx+math.cos(yaw)*dy
    steering=math.atan(.32*2*ty/max(.05,tx*tx+ty*ty));steering=float(np.clip(steering,actor['last_steering']-.65*dt,actor['last_steering']+.65*dt))
   actor['last_steering']=steering;m=Twist();m.linear.x=float(speed);m.angular.z=float(steering);actor['publisher'].publish(m);self.commands.append(dict(stamp=sim,actor=actor['name'],speed=speed,steering=steering,offset=offset))
 def stop(self):
  for a in self.actors:
   if 'publisher' in a:a['publisher'].publish(Twist())
 def save(self):
  (self.out/'actor_poses.json').write_text(json.dumps({a['name']:a['poses'] for a in self.actors}))
  (self.out/'actor_commands.json').write_text(json.dumps(self.commands))
  (self.out/'fixture.json').write_text(json.dumps(dict(scenario=self.scenario,ego_start_s=self.start_s,actors=[{k:v for k,v in a.items() if k in ['name','distance','offset','speed']} for a in self.actors],physical_control='copy of original finite-torque controller; test topics only',ego_receives_ground_truth=False),indent=2)+'\n')
