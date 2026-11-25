#!/usr/bin/env python3
"""Offline validation metrics. Never connects to ROS or supplies driving input."""
import argparse,json,math,csv
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--chain');a=p.parse_args();folder=Path(a.run)
poses=[json.loads(x) for x in (folder/'poses.jsonl').read_text().splitlines()]
chain=Path(a.chain) if a.chain else folder/'chain.jsonl'
rows=[json.loads(x) for x in chain.read_text().splitlines()]
gates=[json.loads(x) for x in (folder/'crossings.jsonl').read_text().splitlines()]
release=next((json.loads(r['values']['/team_10/start_status_10']) for r in rows if r['values'].get('/team_10/camera_gate_10') and '/team_10/start_status_10' in r['values']),None)
if not release:raise SystemExit('No actual camera release recorded')
t0=release['stamp'];f0=next((e['time'] for e in gates if e['gate']==0),None)
finishes=[e['time'] for e in gates if e['gate']==0];finish=finishes[1] if len(finishes)>1 else None
v=[r for r in poses if r['sim']>=t0 and (finish is None or r['sim']<=finish)]
t=np.array([r['sim'] for r in v]);xyz=np.array([r['xyz'] for r in v]);yaw=np.array([r['yaw'] for r in v]);dt=np.diff(t);delta=np.diff(xyz,axis=0);ds=np.linalg.norm(delta,axis=1);speed=ds/dt
cum=np.r_[0,np.cumsum(ds)];smooth=[];body=[]
for i in range(len(t)):
 j=int(np.searchsorted(t,t[i]-.2))
 if i>j and t[i]-t[j]>.1:
  smooth.append((t[i],(cum[i]-cum[j])/(t[i]-t[j])))
  d=xyz[i]-xyz[j];body.append((t[i],(d[0]*math.cos(yaw[i])+d[1]*math.sin(yaw[i]))/(t[i]-t[j])))
def episodes(series,predicate,min_duration):
 out=[];start=None
 for st,value in series:
  if predicate(value):
   if start is None:start=st
  elif start is not None:
   if st-start>=min_duration:out.append([start,st])
   start=None
 if start is not None and series[-1][0]-start>=min_duration:out.append([start,series[-1][0]])
 return out
# Exclude initial commanded acceleration when classifying sustained stops.
stops=episodes([(s,v) for s,v in smooth if s>t0+2],lambda v:v<.05,2)
reverse=episodes(body,lambda v:v<-.2,.5)
root=Path(__file__).resolve().parents[4]
track=list(csv.DictReader((root/'src/urrc_track_gazebo/tracks/processed/monaco_centerline.csv').open()))
center=np.array([[float(r[k]) for k in ('x_m','y_m','z_m')] for r in track])
track_s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(center,axis=0),axis=1))];route=[];last=-1e30
for st,pos in zip(t,xyz):
 if st-last<.2:continue
 idx=int(np.argmin(np.sum((center-pos)**2,axis=1)));route.append((st,float(track_s[idx])));last=st
progress=np.unwrap(np.array([s for _,s in route])/track_s[-1]*2*math.pi)*track_s[-1]/(2*math.pi)
route_speed=[]
for i,(st,_) in enumerate(route):
 j=int(np.searchsorted([u for u,_ in route],st-1.))
 if i>j and st-route[j][0]>.7:route_speed.append((st,float((progress[i]-progress[j])/(st-route[j][0]))))
route_reverse=episodes(route_speed,lambda value:value<-.3,.5)
brief_stops=episodes([(s,v) for s,v in smooth if s>t0+2],lambda value:value<.05,.3)
pre=[r for r in rows if r['values'].get('/clock',1e30)<t0-.1
     and all(k in r['values'] for k in ['/team_10/speed_10','/team_10/steering_10','/team_10/gz_cmd_vel_10'])]
pre_zero=bool(pre) and all(r['values'].get('/team_10/speed_10',0)==0 and r['values'].get('/team_10/steering_10',0)==0 and r['values'].get('/team_10/gz_cmd_vel_10',[0,0])==[0,0] for r in pre)
pre_pose=[r for r in poses if r['sim']<t0-.1]
pre_xy=max((math.dist(r['xyz'][:2],pre_pose[0]['xyz'][:2]) for r in pre_pose),default=None)
result=dict(result='LAP_VERIFIED' if finish else 'INCOMPLETE',record_type='Preview verification; not official operator race timing',camera_release_sim=t0,first_line=f0,finish_line=finish,ordered_crossings=len(gates),gate_sequence=[e['gate'] for e in gates],lap_seconds=None if finish is None else finish-f0,observed_seconds=float(t[-1]-t[0]),distance_m=float(cum[-1]),average_observed_mps=float(cum[-1]/(t[-1]-t[0])),max_sample_speed_mps=float(max(speed)),max_0_2s_speed_mps=float(max(x[1] for x in smooth)),max_sample_gap_s=float(max(dt)),stops_over_2s=stops,stops_over_0_3s=brief_stops,reverse_body_over_0_5s=reverse,track_reverse_episodes=route_reverse,pre_release_zero=pre_zero,wall_contact='unmeasured: no contact sensor',last_pose=v[-1])
result['pre_release_max_xy_displacement_m']=pre_xy
if stops and finish is None:
 end=stops[0][0];idx=min(len(t)-1,int(np.searchsorted(t,end)))
 result['before_first_sustained_stop']=dict(end_sim=end,seconds=float(t[idx]-t[0]),distance_m=float(cum[idx]),average_mps=float(cum[idx]/(t[idx]-t[0])))
if finish:
 firstidx=int(np.searchsorted(t,f0));result['lap_distance_m']=float(cum[-1]-cum[firstidx]);result['lap_average_mps']=result['lap_distance_m']/(finish-f0)
integrity_path=folder/'integrity.json'
if integrity_path.exists():
 integrity=json.loads(integrity_path.read_text());result['physical_integrity']=integrity
 result['wall_contact']=integrity['wall_contact_episodes']
 passed=(integrity['outside_road_center_samples']==0 and integrity['outside_road_midpoints']==0
         and integrity['max_pose_step_m']<.2 and not integrity['other_collision_pairs']
         and integrity['contact_heartbeat_range'] is not None
         and integrity['max_contact_heartbeat_gap_s']<1.2)
 if finish and not passed:result['result']='GATES_COMPLETE_PHYSICAL_VALIDATION_FAILED'
elif finish:result['result']='GATES_COMPLETE_PHYSICAL_VALIDATION_PENDING'
if (folder/'control_restart.json').exists() or list(folder.glob('initialization*.json')):
 result['result']='SEGMENT_OR_MIXED_CODE_EXPERIMENT_NOT_FINAL_LAP'
print(json.dumps(result,indent=2));(folder/'metrics.json').write_text(json.dumps(result,indent=2))
