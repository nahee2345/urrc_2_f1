#!/usr/bin/env python3
"""Offline only: measured racing-line error, wall-time qualification and limits."""
from pathlib import Path
import argparse,collections,json,os
import numpy as np
from analyze_stage3 import analyze3
from trial_metrics import TrialMetrics

def analyze4(folder,root):
 if 'stage3' in folder.resolve().parts or 'stage2' in folder.resolve().parts:raise ValueError('Previous raw records are immutable; analyze a stage4 copy')
 analyze3(folder,root)
 m=json.loads((folder/'metrics.json').read_text());r=json.loads((folder/'result.json').read_text());cfg=json.loads((folder/'run_config.json').read_text());states=json.loads((folder/'states.json').read_text());enc=np.asarray(json.loads((folder/'encoder.json').read_text()));poses=np.asarray(json.loads((folder/'poses.json').read_text()));paths=json.loads((folder/'paths.json').read_text())
 begin=r.get('release_sim_time',0);laps=r.get('lap_records',[]);finish=laps[-1]['crossing_sim'] if laps else r.get('end_sim',begin);tracking=[s for s in states if s.get('state')=='TRACKING' and begin<=s['stamp']<=finish]
 def stat(values):
  a=np.asarray(values);return dict(mean=float(np.mean(a)),p95=float(np.percentile(a,95)),max=float(max(a)),min=float(min(a))) if len(a) else None
 modes=collections.Counter(s.get('path_mode','centerline') for s in tracking);limits=collections.Counter(s.get('limiting_factor','not_recorded') for s in tracking);m['path_mode_records']=dict(modes);m['limiting_factor_record_fraction']={k:v/max(1,len(tracking)) for k,v in limits.items()};m['racing_selected_fraction']=sum(bool(s.get('racing_selected')) for s in tracking)/max(1,len(tracking));m['max_candidate_offset_m']=max((s.get('max_offset_m',0) for s in tracking),default=0)
 clearance=[s['min_clearance_m'] for s in tracking if s.get('min_clearance_m',0)>0];m['observed_cross_section_clearance_m']=stat(clearance)
 straight=[s for s in tracking if s['stamp']>begin+5 and abs(s.get('curvature',1))<.03 and abs(s.get('steering',1))<.015]
 if len(enc):
  velocity=enc[:,1:3].sum(axis=1)*.035/2;m['maximum_encoder_speed_mps']=float(max(velocity));m['straight_encoder_speed_mps']=stat(np.interp([s['stamp'] for s in straight],enc[:,0],velocity))
 # Compare actual future motion to a previously committed local path. Unlike
 # distance from the map center, this measures racing-line tracking directly.
 errors=[];yaw=np.unwrap(poses[:,4]) if len(poses) else []
 for path in paths:
  t=path['stamp'];points=np.asarray(path['points'])
  if t<begin+2 or t+.25>finish or len(points)<2 or len(poses)<2:continue
  x=float(np.interp(t,poses[:,0],poses[:,1]));y=float(np.interp(t,poses[:,0],poses[:,2]));angle=float(np.interp(t,poses[:,0],yaw));cs=np.cos(angle);sn=np.sin(angle);xy=np.stack([x+cs*points[:,0]-sn*points[:,1],y+sn*points[:,0]+cs*points[:,1]],axis=1)
  actual=np.array([np.interp(t+.25,poses[:,0],poses[:,1]),np.interp(t+.25,poses[:,0],poses[:,2])]);a=xy[:-1];v=xy[1:]-a;frac=np.clip(np.sum((actual-a)*v,axis=1)/np.maximum(1e-10,np.sum(v*v,axis=1)),0,1);distance=float(min(np.linalg.norm(actual-a-frac[:,None]*v,axis=1)));errors.append(dict(plan_stamp=t,observation_stamp=t+.25,distance_m=distance))
 m['executed_path_error_0_25s_m']=stat([x['distance_m'] for x in errors]);TrialMetrics.csv(folder/'executed_path_error.csv',errors)
 for row in m.get('per_lap',[]):
  lap=int(row['lap']);end=laps[lap-1]['crossing_sim']
  start=laps[lap-2]['crossing_sim'] if lap>1 else (r.get('approach_sim_time') or begin)
  row['executed_path_error_0_25s_m']=stat([x['distance_m'] for x in errors if start<=x['observation_stamp']<=end])
  row['reference_distance_is_not_tracking_error']=True
 m['reference_center_distance_is_intentional_offset_not_tracking_error']=True
 if laps:
  last=laps[-1];m['goal_wall_from_resume_s']=last.get('wall_elapsed_from_resume_s');m['goal_wall_from_arm_s']=last.get('wall_elapsed_from_arm_s');m['goal_wall_including_light_sequence_s']=r.get('goal_wall_elapsed_from_sequence_s');m['goal_sim_from_release_s']=last['elapsed_seconds']
 if len(laps)>=2:
  second=laps[1]
  m['first_two_laps_wall_from_arm_s']=second.get('wall_elapsed_from_arm_s')
  m['first_two_laps_wall_from_resume_s']=second.get('wall_elapsed_from_resume_s')
  m['first_two_laps_sim_from_release_s']=second['elapsed_seconds']
  # The last goal's sequence/arm difference is a constant start-procedure offset.
  # Adding it to the second crossing is conservative by the final goal callback delay.
  if m.get('goal_wall_including_light_sequence_s') is not None and m.get('goal_wall_from_arm_s') is not None:
   m['first_two_laps_wall_including_lights_conservative_s']=second['wall_elapsed_from_arm_s']+m['goal_wall_including_light_sequence_s']-m['goal_wall_from_arm_s']
  m['first_two_laps_under600wall_clean_full_trial']=bool(r.get('qualifying_distance') and second.get('wall_elapsed_from_arm_s',float('inf'))<600 and r.get('termination')=='diagnostic_lap_goal' and not m['contacts'] and not m['active_safety_stops'])
 m['qualifying_distance_includes_grid_approach']=r.get('qualifying_distance',False);m['self_two_lap_under_600wall']=bool(len(laps)>=2 and r.get('qualifying_distance') and m.get('goal_wall_from_arm_s') is not None and m['goal_wall_from_arm_s']<600 and r.get('termination')=='diagnostic_lap_goal' and not m['contacts'] and not m['active_safety_stops']);m['official_lap_certified']=False
 corners=json.loads((folder/'corner_comparison.json').read_text()) if (folder/'corner_comparison.json').exists() else []
 if len(enc):
  velocity=enc[:,1:3].sum(axis=1)*.035/2
  for c in corners:
   c['entry_encoder_speed']=float(np.interp(c['entry_sim'],enc[:,0],velocity));c['exit_encoder_speed']=float(np.interp(c['exit_sim'],enc[:,0],velocity));c['exit_plus_2s_encoder_speed']=float(np.interp(min(finish,c['exit_sim']+2),enc[:,0],velocity))
 (folder/'stage4_corner_comparison.json').write_text(json.dumps(corners,indent=2)+'\n');TrialMetrics.csv(folder/'stage4_corner_comparison.csv',corners)
 (folder/'metrics.json').write_text(json.dumps(m,indent=2)+'\n');row={k:m.get(k) for k in ['trial','completed_laps','lap_times_seconds','goal_sim_from_release_s','goal_wall_from_resume_s','goal_wall_from_arm_s','goal_wall_including_light_sequence_s','contacts','active_safety_stops','raw_stop_counter','maximum_encoder_speed_mps','racing_selected_fraction','self_two_lap_under_600wall']};TrialMetrics.csv(folder/'stage4_summary.csv',[row]);print(json.dumps(row,indent=2))
 return m
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--root',type=Path,required=True);a=p.parse_args();os.environ.setdefault('MPLCONFIGDIR',str(a.folder/'plot_cache'));analyze4(a.folder.resolve(),a.root.resolve())
