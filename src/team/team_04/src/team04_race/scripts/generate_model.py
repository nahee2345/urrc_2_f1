#!/usr/bin/env python3
"""Reproducible primitive SDF; all coordinates measured from ground centre."""
from pathlib import Path
import xml.etree.ElementTree as E
import math
P=Path(__file__).resolve().parents[1]
def sub(e,n,t=None,**a):
 s=E.SubElement(e,n,a)
 if t is not None:s.text=str(t)
 return s
def pose(e,v):sub(e,'pose',' '.join(map(str,v)))
def link(m,n,xyz,mass,size):
 l=sub(m,'link',name=n);pose(l,[*xyz,0,0,0]);i=sub(l,'inertial');sub(i,'mass',mass);a=sub(i,'inertia')
 for k,v in zip(['ixx','iyy','izz'],[mass*(size[1]**2+size[2]**2)/12,mass*(size[0]**2+size[2]**2)/12,mass*(size[0]**2+size[1]**2)/12]):sub(a,k,v)
 return l
def shape(l,n,xyz,size,color='0.05 0.25 0.8 1',cylinder=False,rpy=(0,0,0)):
 for kind in ['visual','collision']:
  s=sub(l,kind,name=n+'_'+kind);pose(s,[*xyz,*rpy]);g=sub(s,'geometry')
  if cylinder:
   c=sub(g,'cylinder');sub(c,'radius',size[0]);sub(c,'length',size[1])
  else:sub(sub(g,'box'),'size',' '.join(map(str,size)))
  if kind=='visual':
   mat=sub(s,'material');sub(mat,'ambient',color);sub(mat,'diffuse',color)
  elif cylinder:
   o=sub(sub(sub(s,'surface'),'friction'),'ode');sub(o,'mu',1.0);sub(o,'mu2',1.0)
def joint(m,n,parent,child,axis=None,limits=None):
 j=sub(m,'joint',name=n,type='revolute' if axis else 'fixed');sub(j,'parent',parent);sub(j,'child',child)
 if axis:
  a=sub(j,'axis');sub(a,'xyz',axis);lim=sub(a,'limit')
  for k,v in zip(['lower','upper','effort','velocity'],limits):sub(lim,k,v)
  sub(sub(a,'dynamics'),'damping',0.0001)
 return j
def sensor(l,n,typ,rate,topic,frame):
 s=sub(l,'sensor',name=n,type=typ);sub(s,'always_on','true');sub(s,'update_rate',rate);sub(s,'topic',topic);sub(s,'gz_frame_id',frame)
 return s
def generate(lidar_hz=20):
 root=E.Element('sdf',version='1.9');m=sub(root,'model',name='team04');sub(m,'self_collide','false');sub(m,'static','false')
 b=link(m,'base_link',(0,0,0),2.0,(.46,.20,.07));pose(b.find('inertial'),[0,0,.06,0,0,0])
 shape(b,'centre',(0,0,.06),(.24,.20,.07))
 for x in [-.175,.175]:shape(b,'axle_clearance_body',(x,0,.06),(.11,.09,.07)) if x<0 else shape(b,'nose',(x,0,.06),(.11,.09,.07))
 for x,n in [(-.25,'rear_bumper'),(.25,'front_bumper')]:shape(b,n,(x,0,.035),(.02,.20,.02))
 for front in [True,False]:
  for left in [True,False]:
   tag=('front_' if front else 'rear_')+('left' if left else 'right');x=.16 if front else -.16;y=.092 if left else -.092
   parent='base_link'
   if front:
    kn=link(m,'team04_'+tag+'_knuckle',(x,y,.035),.025,(.012,.012,.016));shape(kn,'knuckle',(0,0,0),(.012,.012,.016))
    joint(m,'team04_'+tag+'_steer','base_link',kn.get('name'),'0 0 1',[-math.radians(35),math.radians(35),.8,4]);parent=kn.get('name')
   w=link(m,'team04_'+tag+'_wheel',(x,y,.035),.06,(.07,.018,.07));shape(w,'tire',(0,0,0),(.035,.018),'0.025 0.025 0.025 1',True,(math.pi/2,0,0))
   # Exact solid-cylinder inertia about the axle Y.
   inertia=w.find('inertial/inertia');inertia.find('iyy').text=str(.06*.035**2/2)
   for k in ['ixx','izz']:inertia.find(k).text=str(.06*(3*.035**2+.018**2)/12)
   joint(m,'team04_'+tag+'_spin',parent,w.get('name'),'0 1 0',[-1e16,1e16,.6,300])
   if not front:shape(b,'encoder_'+tag,(x,y*.78,.035),(.018,.012,.018),'0.3 0.3 0.3 1')
 # Dedicated fixed links preserve sensor measurement coordinates.
 for n,xyz,mass,size in [('lidar',(0,0,.13),.08,(.055,.055,.07)),('camera',(.21,0,.12),.025,(.03,.026,.026)),('imu',(.01,0,.06),.01,(.018,.016,.008))]:
  l=link(m,'team04_'+n+'_link',xyz,mass,size);joint(m,'team04_'+n+'_fixed','base_link',l.get('name'))
  if n=='lidar':
   shape(l,'housing',(0,0,0),(.0275,.07),'0.15 0.15 0.15 1',True);shape(b,'lidar_mount',(0,0,.1025),(.032,.032,.015))
   s=sensor(l,'team04_lidar','gpu_lidar',lidar_hz,'/team04/gz/lidar','team04/lidar_link');li=sub(s,'lidar');sc=sub(li,'scan')
   for name,samples,lo,hi in [('horizontal',720,-math.pi,math.pi),('vertical',16,-math.pi/12,math.pi/12)]:
    a=sub(sc,name)
    for k,v in [('samples',samples),('resolution',1),('min_angle',lo),('max_angle',hi)]:sub(a,k,v)
   ran=sub(li,'range')
   for k,v in [('min',.05),('max',12),('resolution',.01)]:sub(ran,k,v)
  elif n=='camera':
   shape(l,'housing',(-.012,0,0),(.024,.026,.026),'0.2 0.2 0.2 1');shape(b,'camera_mount',(.198,0,.10),(.018,.018,.014))
   s=sensor(l,'team04_camera','camera',30,'/team04/gz/camera','team04/camera_optical_frame');c=sub(s,'camera');sub(c,'horizontal_fov',math.pi/2);im=sub(c,'image')
   for k,v in [('width',640),('height',480),('format','R8G8B8')]:sub(im,k,v)
   cl=sub(c,'clip');sub(cl,'near',.02);sub(cl,'far',100)
  else:
   shape(l,'housing',(0,0,0),size,'0.3 0.3 0.3 1');s=sensor(l,'team04_imu','imu',100,'/team04/gz/imu','team04/imu_link');im=sub(s,'imu')
   for typ,std in [('angular_velocity',.0002),('linear_acceleration',.005)]:
    a=sub(im,typ)
    for ax in ['x','y','z']:
     no=sub(sub(a,ax),'noise',type='gaussian');sub(no,'mean',0);sub(no,'stddev',std)
 sub(m,'plugin',filename='libteam04_vehicle_system.so',name='team04::VehicleSystem')
 sub(m,'plugin',filename='gz-sim-imu-system',name='gz::sim::systems::Imu')
 E.indent(root);return E.tostring(root,encoding='unicode')+'\n'
if __name__=='__main__':
 import argparse
 a=argparse.ArgumentParser();a.add_argument('--lidar-hz',type=float,default=None);args=a.parse_args()
 if args.lidar_hz is None:
  import yaml
  args.lidar_hz=yaml.safe_load((P/'config/sensors.yaml').read_text())['scan_04']['ros__parameters']['lidar_hz']
 if not math.isfinite(args.lidar_hz) or args.lidar_hz<=0: a.error('lidar rate must be positive and finite')
 import yaml
 config_path=P/'config/sensors.yaml'
 config=yaml.safe_load(config_path.read_text())
 if config['scan_04']['ros__parameters']['lidar_hz']!=args.lidar_hz:
  config['scan_04']['ros__parameters']['lidar_hz']=args.lidar_hz
  config_path.write_text(yaml.safe_dump(config,sort_keys=False))
 (P/'models/team04_car/model.sdf').write_text(generate(args.lidar_hz))
