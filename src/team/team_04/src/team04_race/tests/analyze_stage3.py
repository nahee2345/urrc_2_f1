#!/usr/bin/env python3
"""Extended observer-only analysis: laps, braking, steering, latency, resources."""
import argparse,csv,json,os,math
from pathlib import Path
import numpy as np
from analyze_trial import analyze
from trial_metrics import TrialMetrics,classify_safety_events

def analyze3(folder,root):
 if 'stage2' in folder.resolve().parts:raise ValueError('Stage2 results are immutable; copy them into stage3 before reanalysis')
 analyze(folder,root)
 result=json.loads((folder/'result.json').read_text());metrics=json.loads((folder/'metrics.json').read_text());states=json.loads((folder/'states.json').read_text());poses=np.asarray(json.loads((folder/'poses.json').read_text()));commands=np.asarray(json.loads((folder/'commands.json').read_text()));enc=np.asarray(json.loads((folder/'encoder.json').read_text()))
 release=result.get('release_sim_time') or (states[0]['stamp'] if states else 0);lap_records=result.get('lap_records',[]);end=lap_records[-1]['crossing_sim'] if lap_records else result.get('end_sim',poses[-1,0] if len(poses) else release)
 active,before=classify_safety_events(states,release)
 metrics.update(completed_laps=len(lap_records),active_safety_stops=len(active),prestart_or_paused_stop_events=len(before),raw_stop_counter=max((s.get('safety_stops',0) for s in states),default=0),lap_times_seconds=result.get('lap_times_seconds',[]),driving_sim_seconds=result.get('driving_sim_seconds'),sim_test_duration=result.get('sim_duration'),wall_duration=result.get('wall_duration'),rtf_mean=result.get('rtf_mean'))
 if (folder/'sensor_stamps.json').exists():
  stamps=json.loads((folder/'sensor_stamps.json').read_text());metrics['active_sensor_hz']={}
  for key,values in stamps.items():
   times=np.array(values);times=times[(times>=release)&(times<=end)]
   if len(times)>1:metrics['active_sensor_hz'][key]=float((len(times)-1)/(times[-1]-times[0]))
 metrics['active_stop_reasons']=[s['state'] for s in active];metrics['prestart_stop_reasons']=[s['state'] for s in before]
 tracking=[s for s in states if s.get('state')=='TRACKING' and release<=s['stamp']<=end]
 if tracking and len(enc)>1:
  timepoints=np.array([s['stamp'] for s in tracking]);target=np.array([s['target_speed'] for s in tracking]);actual=np.interp(timepoints,enc[:,0],enc[:,1:3].sum(axis=1)*.035/2)
  settled=timepoints>release+5
  if settled.any():metrics['command_encoder_absolute_difference_mps']={'mean':float(np.mean(abs(target[settled]-actual[settled]))),'p95':float(np.percentile(abs(target[settled]-actual[settled]),95)),'max':float(max(abs(target[settled]-actual[settled])))}
  lookahead=np.clip(1+.6*target,1,2);metrics['observed_nominal_lookahead_m']={'min':float(min(lookahead)),'max':float(max(lookahead))}
 if len(poses)>1:
  mask=(poses[:,0]>=release)&(poses[:,0]<=end);p=poses[mask]
  if len(p)>1:metrics['actual_average_pose_speed_mps']=float(np.sum(np.linalg.norm(np.diff(p[:,1:4],axis=0),axis=1))/(p[-1,0]-p[0,0]))
 if result.get('stop_command_pose') and result.get('last_pose'):
  metrics['observed_stop_displacement_m']=float(np.linalg.norm(np.array(result['last_pose'][1:4])-np.array(result['stop_command_pose'][1:4])))
  metrics['stop_initial_encoder_speed']=result.get('stop_command_encoder_speed');metrics['stop_final_encoder_speed']=result.get('final_encoder_speed')
  metrics['stop_pose_sampling_period_s']=float(np.median(np.diff(poses[:,0]))) if len(poses)>1 else None
 if len(commands)>1:
  c=commands[(commands[:,0]>=release)&(commands[:,0]<=end)];dt=np.diff(c[:,0]);delta=np.abs(np.diff(c[:,2]));valid=dt>1e-5
  metrics['max_command_steering_rad']=float(max(abs(c[:,2]))) if len(c) else 0
  metrics['steering_delta_max_rad']=float(max(delta)) if len(delta) else 0
  metrics['steering_rate_p95_rad_s']=float(np.percentile(delta[valid]/dt[valid],95)) if valid.any() else 0
  # Sustained speed decreases, grouped without assuming any named corner coordinates.
  brake=[];current=[]
  for i in range(1,len(c)):
   dv=c[i,1]-c[i-1,1];duration=c[i,0]-c[i-1,0]
   decreasing=duration>1e-5 and dv/duration<-.08 and c[i-1,1]>.1
   if decreasing:current.append(i)
   elif current and c[i,0]-c[current[-1],0]>.35:
    if c[current[0]-1,1]-min(c[current,1])>.07:brake.append(current)
    current=[]
  if current and c[current[0]-1,1]-min(c[current,1])>.07:brake.append(current)
  events=[]
  for k,indices in enumerate(brake,1):
   start=c[indices[0]-1];last=c[indices[-1]];pos=[float(np.interp(start[0],poses[:,0],poses[:,j])) for j in [1,2,3]] if len(poses) else [None]*3
   actual=float(np.min((enc[(enc[:,0]>=start[0])&(enc[:,0]<=last[0]+.5),1:3].sum(axis=1))*.035/2)) if len(enc) and ((enc[:,0]>=start[0])&(enc[:,0]<=last[0]+.5)).any() else None
   events.append(dict(event=k,start_sim=float(start[0]),end_sim=float(last[0]),start_speed=float(start[1]),minimum_target_speed=float(min(c[indices,1])),minimum_encoder_speed=actual,observer_x=pos[0],observer_y=pos[1],observer_z=pos[2]))
  TrialMetrics.csv(folder/'braking_events.csv',events);(folder/'braking_events.json').write_text(json.dumps(events,indent=2)+'\n');metrics['braking_event_count']=len(events)
 if (folder/'wheel_joints.json').exists():
  wheels=json.loads((folder/'wheel_joints.json').read_text());angles=[];samples=[]
  for row in wheels:
   if not release<=row['stamp']<=end:continue
   byname=dict(zip(row['names'],row['position']));left=byname.get('team04_front_left_steer');right=byname.get('team04_front_right_steer')
   if left is None or right is None:continue
   angles.extend([abs(left),abs(right)])
   kl=math.tan(left)/(.32+.092*math.tan(left));kr=math.tan(right)/(.32-.092*math.tan(right));samples.append([row['stamp'],-math.atan(.32*(kl+kr)/2)])
  if angles:metrics['actual_front_steering_max_rad']=max(angles);metrics['actual_front_steering_max_deg']=math.degrees(max(angles));metrics['front_steering_within_35deg']=max(angles)<=math.radians(35)+1e-5
  if samples and len(commands)>2:
   measured=np.asarray(samples);ref=np.interp(measured[:,0],commands[:,0],commands[:,2]);mask=(abs(ref)>.02)|(abs(measured[:,1])>.02)
   if mask.any():
    errors=[float(np.mean((measured[mask,1]-np.interp(measured[mask,0]-delay,commands[:,0],commands[:,2]))**2)) for delay in np.arange(0,.501,.025)];index=int(np.argmin(errors));metrics['estimated_steering_response_lag_s']=float(index*.025);metrics['steering_rms_after_lag_rad']=math.sqrt(errors[index])
 if len(poses)>3:
  active_pose=poses[(poses[:,0]>=release)&(poses[:,0]<=end)]
  if len(active_pose)>3:
   velocity=np.linalg.norm(np.gradient(active_pose[:,1:4],active_pose[:,0],axis=0),axis=1);rate=np.gradient(np.unwrap(active_pose[:,4]),active_pose[:,0]);alat=abs(velocity*rate)
   metrics['pose_derived_lateral_acceleration_mps2']={'max':float(max(alat)),'p95':float(np.percentile(alat,95))}
 if root and len(poses):
  ref=np.genfromtxt(root/f"src/urrc_track_gazebo/tracks/processed/{result['track']}_centerline.csv",delimiter=',',names=True);xy=np.stack([ref['x_m'],ref['y_m']],axis=1);errors=[]
  for p in poses:
   index=int(np.argmin(np.sum((xy-p[1:3])**2,axis=1)));values=[]
   for i,j in [((index-1)%len(xy),index),(index,(index+1)%len(xy))]:
    a,b=xy[i],xy[j];v=b-a;t=np.clip(np.dot(p[1:3]-a,v)/max(1e-12,np.dot(v,v)),0,1);values.append(float(np.linalg.norm(p[1:3]-a-t*v)))
   errors.append(min(values))
  errors=np.asarray(errors);after=(poses[:,0]>=release+5)&(poses[:,0]<=end)
  if after.any():metrics['after_alignment_lateral_error_m']={'max':float(max(errors[after])),'p95':float(np.percentile(errors[after],95)),'mean':float(np.mean(errors[after]))}
  rows=[]
  for lap in lap_records:
   begin=release if lap['lap']==1 else lap_records[lap['lap']-2]['crossing_sim'];mask=(poses[:,0]>=begin)&(poses[:,0]<=lap['crossing_sim']);d=poses[mask]
   rows.append(dict(lap=lap['lap'],lap_seconds=lap['lap_seconds'],average_actual_speed=float(np.linalg.norm(np.diff(d[:,1:4],axis=0),axis=1).sum()/max(.001,d[-1,0]-d[0,0])) if len(d)>1 else None,p95_lateral_error_m=float(np.percentile(errors[mask],95)) if mask.any() else None,max_lateral_error_m=float(max(errors[mask])) if mask.any() else None))
  TrialMetrics.csv(folder/'lap_comparison.csv',rows);metrics['per_lap']=rows
  # Fixed offline reference-curvature threshold defines comparable corner zones.
  # The reference and these zones never enter the autonomous controller.
  step=np.linalg.norm(np.diff(xy,axis=0),axis=1);course_s=np.r_[0,np.cumsum(step)];course_length=course_s[-1]+np.linalg.norm(xy[-1]-xy[0]);yaw=np.unwrap(ref['yaw_rad']);kappa=abs(np.gradient(yaw,course_s));indices=np.flatnonzero(kappa>.04)
  groups=[]
  for index in indices:
   if not groups or course_s[index]-course_s[groups[-1][-1]]>5:groups.append([index])
   else:groups[-1].append(index)
  groups=[g for g in groups if course_s[g[-1]]-course_s[g[0]]>=2]
  parameters=np.array([course_s[int(np.argmin(np.sum((xy-p[1:3])**2,axis=1)))] for p in poses]);corner_rows=[]
  for lap in lap_records:
   begin=release if lap['lap']==1 else lap_records[lap['lap']-2]['crossing_sim'];lap_mask=(poses[:,0]>=begin)&(poses[:,0]<=lap['crossing_sim'])
   for number,group in enumerate(groups,1):
    start_s=course_s[group[0]];end_s=course_s[group[-1]]
    mask=lap_mask&(parameters>=start_s)&(parameters<=end_s)
    if mask.sum()<2:continue
    times=poses[mask,0];target=np.interp(times,commands[:,0],commands[:,1]);steering=np.interp(times,commands[:,0],commands[:,2]);actual=np.interp(times,enc[:,0],enc[:,1:3].sum(axis=1)*.035/2) if len(enc) else np.array([float('nan')])
    braking=[event for event in events if times[0]-6<=event['start_sim']<=times[0] and event['start_speed']-event['minimum_target_speed']>.25] if len(commands)>1 else []
    event=braking[-1] if braking else None
    before_distance=None
    if event:
     brake_s=float(np.interp(event['start_sim'],poses[:,0],parameters));before_distance=float((start_s-brake_s+course_length/2)%course_length-course_length/2)
    corner_rows.append(dict(lap=lap['lap'],corner=number,reference_start_m=float(start_s),reference_end_m=float(end_s),entry_sim=float(times[0]),exit_sim=float(times[-1]),minimum_target_speed=float(min(target)),minimum_encoder_speed=float(min(actual)),max_steering_rad=float(max(abs(steering))),max_lateral_error_m=float(max(errors[mask])),p95_lateral_error_m=float(np.percentile(errors[mask],95)),braking_start_sim=event['start_sim'] if event else None,braking_start_observer_x=event['observer_x'] if event else None,braking_start_observer_y=event['observer_y'] if event else None,braking_before_zone_m=before_distance))
  TrialMetrics.csv(folder/'corner_comparison.csv',corner_rows);(folder/'corner_comparison.json').write_text(json.dumps(corner_rows,indent=2)+'\n');metrics['corner_zone_definition']='offline |d(reference_yaw)/ds| > 0.04 1/m, merge gaps <= 5m, span >= 2m; navigation never consumes reference'
 perf=json.loads((folder/'performance.json').read_text()) if (folder/'performance.json').exists() else []
 if perf:
  rss=[r['process_rss_sum_mb'] for r in perf];metrics['process_rss_sum_mb']={'first':rss[0],'last':rss[-1],'max':max(rss)}
  metrics['cpu_percent_one_core_mean']=float(np.mean([r['cpu_percent_one_core'] for r in perf[1:]])) if len(perf)>1 else 0
 stats=json.loads((folder/'world_stats.json').read_text()) if (folder/'world_stats.json').exists() else []
 if stats:
  rtfs=[v[1] for v in stats if release<=v[0]<=end and (len(v)<3 or not v[2])]
  if rtfs:metrics['active_rtf']={'mean':float(np.mean(rtfs)),'p05':float(np.percentile(rtfs,5)),'minimum':float(min(rtfs))}
 for row in metrics.get('per_lap',[]):
  lap=row['lap'];begin=release if lap==1 else lap_records[lap-2]['crossing_sim'];finish=lap_records[lap-1]['crossing_sim']
  ss=[s for s in tracking if begin<=s['stamp']<=finish]
  if ss:row['processing_p95_ms']=float(np.percentile([s['processing_ms'] for s in ss],95));row['sensor_age_p95_s']=float(np.percentile([s['sensor_age_s'] for s in ss],95))
  resources=[v for v in perf if begin<=v['sim']<=finish]
  if resources:row['rss_first_mb']=resources[0]['process_rss_sum_mb'];row['rss_last_mb']=resources[-1]['process_rss_sum_mb'];row['cpu_mean_one_core_percent']=float(np.mean([v['cpu_percent_one_core'] for v in resources]))
  values=[v[1] for v in stats if begin<=v[0]<=finish and not v[2]]
  if values:row['rtf_mean']=float(np.mean(values))
 if metrics.get('per_lap'):TrialMetrics.csv(folder/'lap_comparison.csv',metrics['per_lap'])
 metrics['under_600_clean_one_lap']=bool(lap_records and lap_records[0]['lap_seconds']<600 and not result.get('contacts') and not active)
 (folder/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
 TrialMetrics.csv(folder/'summary.csv',[{'track':result['track'],'max_speed':json.loads((folder/'run_config.json').read_text()).get('max_speed'),'completed_laps':metrics['completed_laps'],'first_lap_seconds':lap_records[0]['lap_seconds'] if lap_records else None,'average_actual_speed':metrics.get('actual_average_pose_speed_mps'),'contacts':metrics['contacts'],'active_safety_stops':len(active),'raw_stop_counter':metrics['raw_stop_counter'],'lidar_hz_sim':metrics.get('hz_sim',{}).get('points'),'processing_mean_ms':metrics.get('processing_ms',{}).get('mean'),'processing_p95_ms':metrics.get('processing_ms',{}).get('p95_abs'),'rtf_mean':metrics.get('active_rtf',{}).get('mean'),'clean_under_600':metrics['under_600_clean_one_lap']}])
 print(json.dumps({k:metrics[k] for k in ['trial','lap_times_seconds','active_safety_stops','raw_stop_counter','under_600_clean_one_lap']},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--root',type=Path,required=True);a=p.parse_args();os.environ.setdefault('MPLCONFIGDIR',str(a.folder/'plot_cache'));analyze3(a.folder,a.root)
