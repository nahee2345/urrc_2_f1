#!/usr/bin/env python3
"""Independent-clock frame/target audit. Truth pose is evaluation ONLY."""
import argparse,bisect,json,math
from pathlib import Path
import numpy as np
from speed_metrics import read,external_clock,calibrate,quantiles
from team10_race.delay_alignment import command_motion,rotate

p=argparse.ArgumentParser();p.add_argument('--run',required=True);a=p.parse_args();folder=Path(a.run)
nav=read(folder/'audit/navigation.jsonl');ad=[r for r in read(folder/'audit/adapter.jsonl') if r.get('event')=='output'];clock,clock_source=external_clock(folder);calibrate(nav,clock);calibrate(ad,clock)
plans=[r for r in nav if r.get('plan')];outputs=[r for r in nav if r.get('event')=='output'];om=[r['mono'] for r in outputs];am=[r['mono'] for r in ad]
poses=read(folder/'poses.jsonl');pt=np.array([r['sim'] for r in poses]);xy=np.array([r['xyz'][:2] for r in poses]);yaw=np.unwrap([r['yaw'] for r in poses]);at=[r['sim'] for r in ad]
ct=np.array([r['wall'] for r in clock]);cs=np.array([r['sim'] for r in clock])
window=json.loads((folder/'speed_metrics.json').read_text());begin=window['camera_release_sim'];finish=window['end_sim']
def pose(t):return np.array([np.interp(t,pt,xy[:,0]),np.interp(t,pt,xy[:,1])]),float(np.interp(t,pt,yaw))
def laser(t):
 x,h=pose(t);return x+np.array(rotate(.08,0.,h)),h
def transform_between(target,start,end):
 x0,h0=laser(start);x1,h1=laser(end)
 d=np.array(rotate(*(x1-x0),-h0));dh=h1-h0
 return np.array(rotate(*(np.array(target)-d),-dh)),d,dh
history=[(r['sim'],r['speed'],r['steering']) for r in ad]
lag=json.loads((folder/'speed_response.json').read_text())['best_command_velocity_lag_sim_s'];rows=[]
for r in plans:
 q=r['plan'];start=r['scan_stamp'];end=r['sim']
 if q['current']<6. or not begin<=end<=finish or start<pt[0] or end>pt[-1] or end<=start:continue
 alignment=q.get('alignment',{});frame_time=alignment['control_sim'] if alignment.get('applied') else start
 # A selected aligned target lives in the estimated control frame. Map it
 # back to sensor coordinates using the recorded online estimate, NOT truth.
 oldtarget=np.array(q['target_xy'])
 if alignment.get('applied'):
  dx,dy,dh=alignment['motion'];oldtarget=np.array(rotate(*oldtarget,dh))+[dx,dy]
 target,d,dh=transform_between(oldtarget,start,end)
 predicted=command_motion(history,start,end,lag=lag)
 no_lag=command_motion(history,start,end,lag=0.)
 if predicted is None:continue
 # First downstream adapter output associated with this plan's publication.
 oi=bisect.bisect_left(om,r['mono']);delivery=None
 if oi<len(outputs):
  o=outputs[oi];ai=bisect.bisect_left(am,o['mono'])
  for candidate in ad[ai:ai+8]:
   if candidate['sim']>finish:break
   if np.float32(candidate.get('requested',0.))==np.float32(o['speed']):delivery=candidate;break
 applied_target=None
 if delivery is not None:
  applied_target,_,_=transform_between(oldtarget,start,delivery['sim'])
 speed=float(np.linalg.norm(pose(min(end+.025,pt[-1]))[0]-pose(max(start, end-.025))[0])/(min(end+.025,pt[-1])-max(start,end-.025)))
 tracking=q.get('tracking',{});ego=tracking.get('ego_velocity');icp_error=None
 if ego is not None and tracking.get('rejected_reason') is None:
  icp_error=abs(float(ego[2])*(end-start)-dh)
 receipt_wall=r['wall']-r['processing_wall_s']
 receipt_sim=float(np.interp(receipt_wall,ct,cs))
 row=dict(sim=end,scan_stamp=start,target_frame='estimated control laser' if alignment.get('applied') else 'sensor generation laser',
  target_frame_sim=frame_time,alignment_applied=alignment.get('applied',False),
  sensor_to_callback_sim_s=receipt_sim-start,callback_to_plan_sim_s=end-receipt_sim,age_sim_s=end-start,
  local_clock_lag_sim_s=end-r.get('sim_local',end),command_mps=q['current'],actual_mps=speed,
  simple_speed_times_age_m=speed*(end-start),observed_laser_forward_shift_m=float(d[0]),observed_laser_lateral_shift_m=float(d[1]),observed_yaw_shift_abs_rad=abs(dh),
  commanded_target_distance_m=math.hypot(*q['target_xy']),current_distance_to_same_target_m=float(np.linalg.norm(target)),
  current_forward_distance_to_same_target_m=float(target[0]),current_target_bearing_rad=math.atan2(target[1],target[0]),
  command_model_translation_error_m=float(np.linalg.norm(np.array(predicted[:2])-d)),
  command_model_no_lag_translation_error_m=None if no_lag is None else float(np.linalg.norm(np.array(no_lag[:2])-d)),
  scan_rotation_prediction_error_rad=icp_error,command_model_yaw_error_rad=abs(predicted[2]-dh),
  lookahead_m=q['lookahead'],raw_steering=q['raw_steering'],filtered_steering=q['filtered_steering'],
  effective_target_pursuit_steering=math.atan(2*.340*target[1]/float(target@target)),
  target_speed=q['target'],navigation_final=q['final'],candidates=q['candidates'],curvature=q['curvature'],
  free_distance=q['free_distance'],preview=q['preview'],
  adapter_output_sim=None if delivery is None else delivery['sim'],
  sensor_to_adapter_sim_s=None if delivery is None else delivery['sim']-start,
  adapter_speed=None if delivery is None else delivery['speed'],
  adapter_target_distance_m=None if applied_target is None else float(np.linalg.norm(applied_target)),
  adapter_target_bearing_rad=None if applied_target is None else math.atan2(applied_target[1],applied_target[0]))
 rows.append(row)
keys=[k for k,v in rows[0].items() if type(v) in (int,float) and k not in ('sim','scan_stamp','target_frame_sim','adapter_output_sim')] if rows else []
result=dict(samples=len(rows),window_sim=[begin,finish],clock_source=clock_source,lag_fit_sim_s=lag,
 statistics={k:quantiles([r[k] for r in rows]) for k in keys},
 clocks='Sensor header and /clock are simulator time. Wall/monotonic are host clocks. Completion and publication wall times mapped to independently received /clock; local ROS /clock may lag inside callback. Callback entry reconstructed from processing duration. Subscription execution is not network arrival.',
 frames='Path and ordinary target at primary laser generation time; laser is model x=.08m / rear axle+.25m. SE(2) truth interpolation includes vehicle rotation and laser lever arm. Aligned target uses recorded online estimated frame.',
 limits='Truth only offline. Planar yaw/XY model; pitch/roll were not recorded. Fitted lag and empirical uncertainty are not worst-case guarantees. Adapter matching uses float32 value and timing without sequence IDs. Replay cannot reproduce a changed trajectory.',
 next_minimum='Latest-scan queue candidate is now lap-verified; inspect remaining time-weighted curvature/visibility limits on saved inputs. Rejected command-model path alignment must not be repeated without new evidence.')
(folder/'delay_geometry_analysis.json').write_text(json.dumps(result,indent=2));(folder/'delay_geometry_samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows));print(json.dumps(result,indent=2))
