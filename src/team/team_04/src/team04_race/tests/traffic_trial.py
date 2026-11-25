#!/usr/bin/env python3
"""Development-only trial. Map/spawn/pose stay here; never consumed by planner."""
import argparse,json,os,signal,subprocess,sys,time,math,struct,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,LaserScan,JointState
from std_msgs.msg import String,Float32
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts,WorldStatistics
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import MarkerArray
from trial_metrics import TrialMetrics,ExistingStartObserver
from traffic_fixture import TrafficFixture
import shutil,hashlib

def stop(p):
 if p and p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
def main():
 a=argparse.ArgumentParser();a.add_argument('--scenario',choices=['solo','stopped','follow','follow_offset','left','left_wide','left_clear','left_static','right','right_wide','right_clear','right_static','blocked','brake','corner','multi','occlusion','cutin'],default='solo');a.add_argument('--traffic-mode',choices=['solo_racing','traffic_racing'],default='traffic_racing');a.add_argument('--qualifying-distance',action='store_true');a.add_argument('--path-mode',choices=['centerline','racing_line']);a.add_argument('--entry-defaults',action='store_true');a.add_argument('--stop-on-active-failure',action='store_true');a.add_argument('--artifacts-dir',type=Path);a.add_argument('--autonomy-config',type=Path);a.add_argument('--start-sequence',choices=['official','signal'],default='official');a.add_argument('--start-pose',type=Path);a.add_argument('--stop-after-laps',type=int,default=0);a.add_argument('--isolated',action='store_true');a.add_argument('--root',type=Path,required=True);a.add_argument('--track',default='monza',choices=['monza','hungaroring']);a.add_argument('--duration',type=float,default=15);a.add_argument('--label',default='audit');a.add_argument('--autonomous',action='store_true');a.add_argument('--max-speed',type=float,default=.3);args=a.parse_args()
 if args.entry_defaults and (args.path_mode or args.autonomy_config or any(x=='--max-speed' or x.startswith('--max-speed=') for x in sys.argv)):
  a.error('--entry-defaults uses the installed manifest without parameter overrides')
 if args.isolated:
  os.environ['GZ_PARTITION']='team04_development_'+str(os.getpid())
  os.environ['ROS_DOMAIN_ID']='96'
 root=args.root.resolve();out=(args.artifacts_dir.resolve() if args.artifacts_dir else root/'src/team_04/artifacts/stage5')/args.label;out.mkdir(parents=True,exist_ok=False)
 sys.path.insert(0,str(root/'race/scripts'));import race_manager as official
 from ament_index_python.packages import get_package_share_directory
 share=Path(get_package_share_directory('team04_race'));prefix=share.parents[1];runtime_fingerprints={}
 for name,path in {'autonomy_executable':prefix/'lib/team04_race/autonomy_04','observer_library':prefix/'lib/libteam04_trial_observer.so','vehicle_library':prefix/'lib/libteam04_vehicle_system.so','vehicle_model':share/'models/team04_car/model.sdf'}.items():
  if path.is_file():runtime_fingerprints[name]=hashlib.sha256(path.read_bytes()).hexdigest()
 env=os.environ.copy();env['GZ_SIM_RESOURCE_PATH']=str(root/'src/urrc_track_gazebo/models')+os.pathsep+env.get('GZ_SIM_RESOURCE_PATH','')
 fixture=TrafficFixture(root,out,share,args.track,args.scenario);traffic_records=[]
 metrics=TrialMetrics();metrics.qualifying_distance=args.qualifying_distance;center_paths=[];candidate_records=[];speed_profiles=[]
 processes=[];files=[];gate=None;node=None;entry=None;report={};poses=[];states=[];contacts=[];counts={};last={};cloud_saved=False;failure_saved=False;scan_saved=False;encoder=[];latest_cloud=None;wheel_joints=[];progress=0.;last_parameter=None;diagnostic_laps=0;stamps={'points':[],'scan':[]};has_started=False;active_failure=None
 def launch(name,cmd):
  f=(out/(name+'.log')).open('w');files.append(f);p=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);(out/'runtime_processes.json').write_text(json.dumps([dict(pid=q.pid,pgid=os.getpgid(q.pid)) for q in processes if q.poll() is None]));return p
 def cloud(m):
  nonlocal cloud_saved,failure_saved,latest_cloud
  latest_cloud=m
  counts['points']=counts.get('points',0)+1;last['points']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;stamps['points'].append(last['points'])
  failing=bool(has_started and states and states[-1].get('state') not in ['TRACKING','STARTUP','WAITING_FOR_START'])
  if not cloud_saved or (failing and not failure_saved):
   offsets={f.name:f.offset for f in m.fields};rows=[]
   for y in range(m.height):
    for x in range(m.width):
     p=y*m.row_step+x*m.point_step
     xyz=[struct.unpack_from('<f',m.data,p+offsets[f])[0] for f in ['x','y','z']];rows.append(xyz)
   np.savez_compressed(out/('cloud.npz' if not cloud_saved else 'cloud_failure.npz'),xyz=np.array(rows),width=m.width,height=m.height)
   if cloud_saved:failure_saved=True
   cloud_saved=True
 def center_path(m):
  stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if not center_paths or stamp-center_paths[-1]['stamp']>=.199:center_paths.append(dict(stamp=stamp,points=[[p.pose.position.x,p.pose.position.y] for p in m.poses]))
 def candidates(m):
  if not m.markers:return
  stamp=m.markers[0].header.stamp.sec+m.markers[0].header.stamp.nanosec*1e-9
  if not candidate_records or stamp-candidate_records[-1]['stamp']>=.499:candidate_records.append(dict(stamp=stamp,paths=[[[p.x,p.y] for p in mark.points] for mark in m.markers]))
 def profiles(m):
  stamp=last.get('clock',0)
  if not speed_profiles or stamp-speed_profiles[-1]['stamp']>=.499:
   try:speed_profiles.append(dict(stamp=stamp,profile=json.loads(m.data)))
   except ValueError:pass
 def joints(m):
  stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;encoder.append([stamp,*m.velocity[:2]])
  if not wheel_joints or stamp-wheel_joints[-1]['stamp']>=.049:wheel_joints.append(dict(stamp=stamp,names=list(m.name),position=list(m.position),velocity=list(m.velocity)))
 def scan(m):
  nonlocal scan_saved
  counts['scan']=counts.get('scan',0)+1;last['scan']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;stamps['scan'].append(last['scan'])
  if not scan_saved:np.savez_compressed(out/'scan.npz',ranges=np.array(m.ranges),angle_min=m.angle_min,angle_increment=m.angle_increment);scan_saved=True
 centre=np.genfromtxt(root/f'src/urrc_track_gazebo/tracks/processed/{args.track}_centerline.csv',delimiter=',',names=True)
 centre_xy=np.stack([centre['x_m'],centre['y_m']],axis=1);centre_s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(centre_xy,axis=0),axis=1))];course_length=float(centre_s[-1]+np.linalg.norm(centre_xy[-1]-centre_xy[0]))
 def pose(m):
  nonlocal progress,last_parameter,diagnostic_laps
  for t in m.transforms:
   if t.child_frame_id=='team04':
    p=t.transform.translation;q=t.transform.rotation;stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
    if not poses or stamp-poses[-1][0]>.09:
     poses.append([stamp,p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
     index=int(np.argmin(np.sum((centre_xy-[p.x,p.y])**2,axis=1)));parameter=float(centre_s[index])
     if last_parameter is not None:
      delta=(parameter-last_parameter+course_length/2)%course_length-course_length/2
      if abs(delta)<5:progress+=delta
     last_parameter=parameter;diagnostic_laps=max(0,int(progress/course_length))
     metrics.pose(stamp,progress,poses[-1],course_length)
 def state(m):
  nonlocal has_started,active_failure
  try:value=json.loads(m.data)
  except ValueError:value={'state':m.data}
  states.append(value)
  if metrics.release_sim is not None and value.get('state')=='TRACKING' and value.get('stamp',0)>metrics.release_sim+1e-4 and value.get('target_speed',0)>0:has_started=True
  if has_started and value.get('state') not in ('TRACKING','WAITING_FOR_START','STARTUP') and active_failure is None:active_failure=value
 def contact(m):
  counts['contact_messages']=counts.get('contact_messages',0)+1
  counts['physical_contacts']=counts.get('physical_contacts',0)+len(m.contacts)
  for c in m.contacts:
   names=[str(c.collision1),str(c.collision2)]
   if 'contact_sample' not in report:report['contact_sample']=names
   if any('wall' in n or 'gantry' in n or 'ground' in n or 'team04_test_actor' in n for n in names):contacts.append({'time':time.monotonic(),'names':names})
 try:
  server=launch('gazebo',['gz','sim','-s','-r','--headless-rendering',str(root/f'src/urrc_track_gazebo/worlds/{args.track}_race.sdf')])
  for _ in range(40):
   if server.poll() is not None:raise RuntimeError('Gazebo exited')
   s=subprocess.run(['gz','service','-l'],env=env,capture_output=True,text=True,timeout=5)
   if f'/world/{args.track}/create' in s.stdout:break
   time.sleep(.2)
  else:raise RuntimeError('Gazebo create service missing')
  model=ET.parse(share/'models/team04_car/model.sdf');m=model.getroot().find('model')
  # Instrument a COPY, leaving all production model/physics/sensor settings intact.
  ET.SubElement(m,'plugin',filename='libteam04_trial_observer.so',name='team04::TrialObserver')
  contact_topics=['/team04/test/contacts']
  modelpath=out/'instrumented.sdf';model.write(modelpath)
  entry=json.loads((share/'race_entry.json').read_text());entry['_model']=str(modelpath)
  if args.entry_defaults:
   defaults=entry.get('launch_arguments',{})
   args.traffic_mode=defaults.get('autonomy_traffic_mode','solo_racing')
   args.autonomous=defaults.get('driving_mode')=='autonomous'
   args.max_speed=float(defaults['autonomy_max_speed']);args.path_mode=defaults.get('autonomy_path_mode','centerline')
  slot=json.loads((root/f'src/urrc_track_gazebo/tracks/processed/{args.track}.json').read_text())['grid'][0]
  if args.start_pose:
   previous=json.loads(args.start_pose.read_text())[-1];slot=dict(slot,x=previous[1],y=previous[2],road_z=previous[3],yaw=previous[4]);report['diagnostic_start_pose']=previous
  if args.scenario!='solo':slot=fixture.slot(fixture.start_s)
  official.spawn(root,args.track,entry,slot,env)
  fixture.spawn(official,env)
  launch_args=['autonomy_traffic_mode:='+args.traffic_mode]
  if args.path_mode:launch_args.append('autonomy_path_mode:='+args.path_mode)
  if args.autonomy_config:launch_args.append('autonomy_config:='+str(args.autonomy_config.resolve()))
  config_path=args.autonomy_config.resolve() if args.autonomy_config else share/'config/autonomy.yaml'
  shutil.copyfile(config_path,out/'autonomy_used.yaml')
  participant_args=official.launch_args(entry) if args.entry_defaults else ['driving_mode:='+('autonomous' if args.autonomous else 'manual'),'autonomy_max_speed:='+str(args.max_speed),*launch_args]
  report['participant_launch_arguments']=participant_args
  algorithm=launch('participant',['ros2','launch','team04_race','race.launch.py',*participant_args])
  launch('observer_bridge',['ros2','run','ros_gz_bridge','parameter_bridge','/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',f'/world/{args.track}/stats@ros_gz_interfaces/msg/WorldStatistics[gz.msgs.WorldStatistics','/model/team04/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',*[t+'@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts' for t in contact_topics],*fixture.bridge_arguments()])
  rclpy.init();node=rclpy.create_node('team04_trial_observer');fixture.attach(node);subs=[]
  subs.append(node.create_subscription(String,'/team04/debug/traffic_state',lambda m:traffic_records.append(json.loads(m.data)),10))
  for typ,t,cb in [(PointCloud2,'/team04/points',cloud),(LaserScan,'/team04/scan',scan),(TFMessage,'/model/team04/pose',pose),(String,'/team04/debug/state',state),(Clock,'/clock',lambda m:last.update(clock=m.clock.sec+m.clock.nanosec*1e-9)),(WorldStatistics,f'/world/{args.track}/stats',metrics.world_stats),(NavPath,'/team04/debug/path',metrics.path),(NavPath,'/team04/debug/centerline',center_path),(MarkerArray,'/team04/debug/racing_candidates',candidates),(String,'/team04/debug/speed_profile',profiles),(Float32,'/team04/speed',lambda m:metrics.command(last.get('clock',last.get('points',0)),m.data)),(Float32,'/team04/steering',lambda m:metrics.command(last.get('clock',last.get('points',0)),m.data,True)),(JointState,'/team04/wheel_joint_states',joints)]:subs.append(node.create_subscription(typ,t,cb,qos_profile_sensor_data))
  for t in contact_topics:subs.append(node.create_subscription(Contacts,t,contact,qos_profile_sensor_data))
  # Wait for live cloud, then allow a stable, measured prestart interval.
  deadline=time.monotonic()+15
  while not cloud_saved and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
  if not cloud_saved:raise RuntimeError('No PointCloud2 received')
  end=time.monotonic()+2
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
  (out/'gazebo_topics.txt').write_text(subprocess.run(['gz','topic','-l'],env=env,capture_output=True,text=True,timeout=5).stdout)
  end=time.monotonic()+.5
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
  first=dict(last);startwall=time.monotonic();startsim=last.get('points',0);count0=dict(counts)
  if args.autonomous:
   gate=official.StartGate([entry]);assert not gate.ready(['team04'],{'team04':algorithm},timeout=10)
   if args.start_sequence=='official':
    metrics.approach_distance=(course_length-last_parameter)%course_length if args.qualifying_distance and last_parameter is not None else 0.
    report['start_sequence_wall_begin']=time.monotonic()
    monitor=ExistingStartObserver(node,last,metrics,lambda:progress,lambda:poses[-1] if poses else None)
    official.start_sequence(args.track,root,[entry],.2,monitor,gate,args.stop_after_laps or 1,env)
   else:
    metrics.released(last.get('clock',startsim),progress,poses[-1] if poses else None);gate.publish(['team04'],True);metrics.resumed()
   startwall=metrics.release_wall;startsim=metrics.release_sim
  deadline=time.monotonic()+args.duration;last_progress=time.monotonic();failed_since=None
  while time.monotonic()<deadline:
   rclpy.spin_once(node,timeout_sec=.01)
   fixture.tick(last.get('clock',0),metrics.release_sim)
   if time.monotonic()-last_progress>10:
    status=states[-1] if states else {};perf=metrics.checkpoint(last.get('clock',0),progress,processes);summary={'performance':perf,'lap_times':metrics.laps,'elapsed':round(time.monotonic()-startwall,1),'progress_m':progress,'diagnostic_laps':diagnostic_laps,'pose':poses[-1] if poses else [],'planner':status,'traffic':traffic_records[-1] if traffic_records else {}};(out/'progress.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True);last_progress=time.monotonic()
   if states and states[-1].get('state')=='TRACKING' and states[-1].get('stamp',0)>startsim+1e-4 and states[-1].get('target_speed',0)>0:has_started=True
   if args.stop_on_active_failure and active_failure is not None:
    report['termination']='active_safety_stop';report['active_failure_event']=active_failure;break
   if args.stop_after_laps and len(metrics.laps)>=args.stop_after_laps:report['termination']='diagnostic_lap_goal';break
   if args.autonomous and states and states[-1].get('state') not in ['TRACKING','STARTUP','WAITING_FOR_START']:
    if failed_since is None:failed_since=time.monotonic()
    if time.monotonic()-failed_since>5:report['termination']='planner_safety_stop';break
   else:failed_since=None
   if contacts:report['termination']='contact';break
   if algorithm.poll() is not None:raise RuntimeError('Participant launch exited')
  if gate:
   fixture.stop()
   report['goal_wall_elapsed_from_sequence_s']=time.monotonic()-report['start_sequence_wall_begin'] if 'start_sequence_wall_begin' in report else None
   report['stop_command_sim']=last.get('clock',last.get('points',0));report['stop_command_pose']=poses[-1] if poses else None;report['stop_command_encoder_speed']=(encoder[-1][1]+encoder[-1][2])*.035/2 if encoder else None
   gate.publish(['team04'],False)
  end=time.monotonic()+1
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
  report['final_encoder_speed']=(encoder[-1][1]+encoder[-1][2])*.035/2 if encoder else None
  report['end_sim']=last.get('clock',last.get('points',0));report['last_pose']=poses[-1] if poses else None
  simduration=last.get('points',startsim)-startsim
  report.update(course_length_m=course_length,diagnostic_progress_m=progress,diagnostic_completed_laps=diagnostic_laps,track=args.track,autonomous=args.autonomous,sim_duration=simduration,wall_duration=time.monotonic()-startwall,counts=counts,contacts=contacts[:100],contact_monitor_messages=counts.get('contact_messages',0),hz_sim={k:(len(stamps[k])-1)/max(.001,stamps[k][-1]-stamps[k][0]) if len(stamps[k])>1 else 0 for k in ['points','scan']},distance_m=sum(math.dist(a[1:4],b[1:4]) for a,b in zip(poses,poses[1:])),last_state=states[-1] if states else None)
 except BaseException as e:
  report['error']=repr(e);raise
 finally:
  fixture.stop();fixture.save()
  (out/'traffic.json').write_text(json.dumps(traffic_records))
  report['scenario']=args.scenario;report['traffic_mode']=args.traffic_mode
  if latest_cloud is not None:
   fields={f.name:f.offset for f in latest_cloud.fields};m=latest_cloud;rows=[]
   for y in range(m.height):
    for x in range(m.width):
     o=y*m.row_step+x*m.point_step;rows.append([struct.unpack_from('<f',m.data,o+fields[f])[0] for f in ['x','y','z']])
   np.savez_compressed(out/'cloud_last.npz',xyz=np.array(rows),width=m.width,height=m.height)
  (out/'center_paths.json').write_text(json.dumps(center_paths));(out/'candidate_paths.json').write_text(json.dumps(candidate_records));(out/'speed_profiles.json').write_text(json.dumps(speed_profiles))
  (out/'sensor_stamps.json').write_text(json.dumps(stamps))
  (out/'wheel_joints.json').write_text(json.dumps(wheel_joints))
  metrics.save(out,report)
  TrialMetrics.csv(out/'states.csv',states)
  TrialMetrics.csv(out/'poses.csv',poses,['stamp','x','y','z','yaw'])
  TrialMetrics.csv(out/'encoder.csv',encoder,['stamp','left_rad_s','right_rad_s'])
  (out/'run_config.json').write_text(json.dumps({'runtime_fingerprints':runtime_fingerprints,'argv':sys.argv,'package_share':str(share),'max_speed':args.max_speed,'path_mode':args.path_mode,'start_sequence':args.start_sequence,'traffic_mode':args.traffic_mode,'scenario':args.scenario,'autonomy_config':str(args.autonomy_config) if args.autonomy_config else None},indent=2)+'\n')
  (out/'result.json').write_text(json.dumps(report,indent=2)+'\n');(out/'poses.json').write_text(json.dumps(poses));(out/'states.json').write_text(json.dumps(states));(out/'encoder.json').write_text(json.dumps(encoder))
  if gate:gate.close()
  if node:node.destroy_node();rclpy.shutdown()
  for p in reversed(processes):stop(p)
  for f in files:f.close()
  if entry and entry.get('_prepared_model'):Path(entry['_prepared_model']).unlink(missing_ok=True)
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
