#!/usr/bin/env python3
"""Offline actor truth evaluation. Never imported by autonomy or traffic modules."""
from pathlib import Path
import argparse,json,math,collections,os
import numpy as np
from analyze_stage4 import analyze4

def stats(v):
 a=np.asarray(v,float);a=a[np.isfinite(a)]
 return dict(count=len(a),mean=float(a.mean()),p95=float(np.percentile(a,95)),max=float(a.max()),min=float(a.min())) if len(a) else None

def rectangle(x,y,yaw):
 cs,sn=math.cos(yaw),math.sin(yaw)
 return np.array([[x+cs*a-sn*b,y+sn*a+cs*b] for a,b in [(-.26,-.119479),(.26,-.119479),(.26,.119479),(-.26,.119479)]])
def point_distance(p,poly):
 a=poly;v=np.roll(poly,-1,axis=0)-a;d=p-a;t=np.clip(np.sum(d*v,axis=1)/np.maximum(1e-10,np.sum(v*v,axis=1)),0,1)
 return float(np.min(np.linalg.norm(d-t[:,None]*v,axis=1)))
def footprint_gap(a,b):
 for poly in [a,b]:
  for i in range(2):
   v=poly[i+1]-poly[i];axis=np.array([-v[1],v[0]]);pa=a@axis;pb=b@axis
   if pa.max()<pb.min() or pb.max()<pa.min():break
  else:continue
  break
 else:return 0.
 return min([point_distance(p,b) for p in a]+[point_distance(p,a) for p in b])
def interpolate(a,t):return np.array([np.interp(t,a[:,0],a[:,i]) for i in range(1,5)])

def analyze(folder,root):
 folder=folder.resolve()
 if 'stage5' not in folder.parts:raise ValueError('stage5 output only')
 r=json.loads((folder/'result.json').read_text())
 if not (folder/'metrics.json').exists():analyze4(folder,root)
 m=json.loads((folder/'metrics.json').read_text());traffic=json.loads((folder/'traffic.json').read_text());truth={k:np.asarray(v,float) for k,v in json.loads((folder/'actor_poses.json').read_text()).items()};poses=np.asarray(json.loads((folder/'poses.json').read_text()),float)
 begin=r.get('release_sim_time',0);end=r.get('stop_command_sim',r.get('end_sim',begin));ts=[s for s in traffic if begin<=s['stamp']<=end];range_error=[];rate_error=[];ttc_error=[];gaps=[];matched=0;expected=0;false=0;detections=0;switches=0;comparisons=0;last_ids={};last_ranges={};rows=[];initial_front=set();truth_passed=set();hazard_times=[];notice_times=[]
 for record in ts:
  t=record['stamp']
  if len(poses)<2:continue
  ego=interpolate(poses,t);cs,sn=math.cos(ego[3]),math.sin(ego[3]);rotation=np.array([[cs,sn],[-sn,cs]]);actor_data={};a=rectangle(*ego[:2],ego[3])
  for name,p in truth.items():
   if len(p)<2 or t<p[0,0] or t>p[-1,0]:continue
   actor=interpolate(p,t);rel=rotation@(actor[:2]-ego[:2]);b=rectangle(*actor[:2],actor[3]);gap=footprint_gap(a,b);gaps.append(gap)
   shape=(b-ego[:2])@rotation.T;range_truth=point_distance(np.array([0.,0.]),shape)
   rate=None
   if name in last_ranges:
    prev_t,prev_r=last_ranges[name];rate=(prev_r-range_truth)/max(.001,t-prev_t)
   last_ranges[name]=(t,range_truth)
   actor_data[name]=dict(rel=rel,shape=shape,range=range_truth,closing=rate)
  if not initial_front:initial_front={n for n,v in actor_data.items() if v['rel'][0]>1}
  if record['completed']>0:truth_passed.update(n for n,v in actor_data.items() if n in initial_front and max(v['shape'][:,0])<-.26-.16)
  eligible={n:v for n,v in actor_data.items() if .4<np.linalg.norm(v['rel'])<10 and v['rel'][0]>-2};expected+=len(eligible);used=set();detections+=len(record['detections'])
  for detection in record['detections']:
   p=np.array([detection['x'],detection['y']]);candidates=[(point_distance(p,v['shape']),n) for n,v in eligible.items() if n not in used]
   cost,name=min(candidates,default=(1e9,None))
   if cost>.4:false+=1;continue
   used.add(name);matched+=1;range_error.append(abs(np.linalg.norm(p)-eligible[name]['range']))
  for track in record['tracks']:
   if not track['observed'] or track['hits']<3:continue
   p=np.array([track['x'],track['y']]);cost,name=min([(point_distance(p,v['shape']),n) for n,v in actor_data.items()],default=(1e9,None))
   if cost>.4:continue
   if name in last_ids:comparisons+=1;switches+=int(last_ids[name]!=track['id'])
   last_ids[name]=track['id'];rate=actor_data[name]['closing']
   if rate is not None:rate_error.append(abs(track['closing']-rate))
  front=[v for v in actor_data.values() if v['rel'][0]>0 and abs(v['rel'][1])<.4 and v['closing'] is not None and v['closing']>.1]
  if front:
   v=min(front,key=lambda x:x['rel'][0]);reference=max(0.,v['rel'][0]-.52-.16)/v['closing']
   if record.get('ttc_s') is not None:ttc_error.append(record['ttc_s']-reference)
   if reference<=3:hazard_times.append(t)
  if record['risk']!='CLEAR' and record['tracks']:notice_times.append(t)
  rows.append(dict(stamp=t,state=record['state'],risk=record['risk'],minimum_full_footprint_gap_m=min(gaps[-len(actor_data):]) if actor_data else None,attempts=record['attempts'],completed=record['completed'],aborted=record['aborted'],objects=len(record['tracks']),detections=len(record['detections'])))
 contacts=r.get('contacts',[]);vehicle_contacts=sum(any('team04_test_actor' in n for n in c['names']) for c in contacts);wall_contacts=len(contacts)-vehicle_contacts
 offroad=0
 reference=np.genfromtxt(root/f"src/urrc_track_gazebo/tracks/processed/{r['track']}_centerline.csv",delimiter=',',names=True);xy=np.stack([reference['x_m'],reference['y_m']],axis=1)
 for p in poses:
  if p[0]<begin:continue
  index=int(np.argmin(np.sum((xy-p[1:3])**2,axis=1)));dyaw=p[4]-reference['yaw_rad'][index];d=p[1:3]-xy[index];lateral=abs(-math.sin(reference['yaw_rad'][index])*d[0]+math.cos(reference['yaw_rad'][index])*d[1]);support=.26*abs(math.sin(dyaw))+.119479*abs(math.cos(dyaw));offroad+=int(lateral+support>reference['road_width_m'][index]/2+.05)
 completed=max((x['completed'] for x in ts),default=0);clean=not contacts and offroad==0 and r.get('contact_monitor_messages',0)>0
 completion_records=[x for x in ts if x['completed']>0];normal_after=0
 if completion_records:
  start=completion_records[0]['stamp'];run_start=None
  for x in ts:
   if x['stamp']<start:continue
   if x['state']=='RACING':
    if run_start is None:run_start=x['stamp']
    normal_after=max(normal_after,x['stamp']-run_start)
   else:run_start=None
 commands=np.asarray(json.loads((folder/'commands.json').read_text()),float)
 rates=[]
 if len(commands)>1:
  for before,after in zip(commands[:-1],commands[1:]):
   dt=after[0]-before[0]
   if begin<=after[0]<=end and dt>1e-5:rates.append(abs(after[2]-before[2])/dt)
 m.update(observed_command_steering_rate_max_rad_s=max(rates,default=None),maximum_simultaneous_tracks=max((len(x['tracks']) for x in ts),default=0),maximum_simultaneous_detections=max((len(x['detections']) for x in ts),default=0),maximum_track_missed_s=max((t['missed_s'] for x in ts for t in x['tracks']),default=0),traffic_processing_over50ms_records=sum(x['processing_ms']>50 for x in ts),recorded_control_input_topics='own LiDAR, start Bool, clock; internal command history only')
 m.update(scenario=r.get('scenario'),traffic_mode=r.get('traffic_mode'),traffic_state_record_counts=dict(collections.Counter(x['state'] for x in ts)),risk_record_counts=dict(collections.Counter(x['risk'] for x in ts)),traffic_processing_ms=stats([x['processing_ms'] for x in ts]),range_qualified_detection_recall=matched/expected if expected else None,detection_false_fraction=false/detections if detections else None,range_error_to_nearest_physical_footprint_m=stats(range_error),closing_speed_absolute_error_mps=stats(rate_error),constant_velocity_ttc_signed_error_s=stats(ttc_error),track_id_retention=1-switches/comparisons if comparisons else None,track_id_switches=switches,track_id_comparisons=comparisons,minimum_full_footprint_gap_m=min(gaps) if gaps else None,vehicle_contact_records=vehicle_contacts,wall_or_ground_contact_records=wall_contacts,offline_offroad_frames=offroad,overtake_attempts=max((x['attempts'] for x in ts),default=0),overtake_completions=completed,overtake_aborts=max((x['aborted'] for x in ts),default=0),normal_racing_seconds_after_completion=normal_after,ground_truth_passed_actors=sorted(truth_passed),strict_overtake_success=bool(completed>0 and truth_passed and clean and min(gaps,default=0)>=.16 and normal_after>=5 and m.get('front_steering_within_35deg',False)),collision_avoidance_clean=clean,observed_minimum_gap_sampling='2D full 0.520 x 0.238958 m rectangles, interpolated observer poses; contact observer is independent',detection_metric_scope='actor within 10m and x>-2m, including occlusion; visible min-X / median-Y anchor, not full vehicle center; class-unspecific occupied candidates',ttc_metric_scope='constant-velocity truth reference, not a measured eventual collision time',collision_risk_detection_latency_s=max(0.,min(notice_times)-min(hazard_times)) if hazard_times and notice_times else None,latency_metric_scope='diagnostic onset: independent constant-velocity TTC<=3s in +/-0.4m projected corridor; this is not a safety threshold',latency_not_measured_reason=None if hazard_times and notice_times else 'no qualified reference risk onset or no notice',constant_velocity_ttc_absolute_error_s=stats(np.abs(ttc_error)))
 from trial_metrics import TrialMetrics
 TrialMetrics.csv(folder/'traffic_metrics.csv',rows);(folder/'metrics.json').write_text(json.dumps(m,indent=2)+'\n');print(json.dumps({k:m.get(k) for k in ['scenario','range_qualified_detection_recall','minimum_full_footprint_gap_m','overtake_attempts','overtake_completions','overtake_aborts','strict_overtake_success','collision_avoidance_clean','active_safety_stops','traffic_state_record_counts']},indent=2));return m
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--root',type=Path,required=True);a=p.parse_args();os.environ['MPLCONFIGDIR']=str(a.folder/'plot_cache');analyze(a.folder,a.root.resolve())
