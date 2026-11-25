#!/usr/bin/env python3
"""Summarize recorded attempts; a failed partial trial never becomes a lap."""
import argparse,csv,json
from pathlib import Path
import yaml
from trial_metrics import classify_safety_events

def summarize(root):
 out=root/'src/team_04/artifacts/stage4';rows=[]
 for folder in sorted(out.glob('stage4-*')):
  if not folder.is_dir():continue
  try:
   result=json.loads((folder/'result.json').read_text()) if (folder/'result.json').exists() else {}
   cfg=json.loads((folder/'run_config.json').read_text()) if (folder/'run_config.json').exists() else {}
   metrics=json.loads((folder/'metrics.json').read_text()) if (folder/'metrics.json').exists() else {}
  except json.JSONDecodeError:continue
  parameters=yaml.safe_load((folder/'autonomy_used.yaml').read_text())['autonomy_04']['ros__parameters'] if (folder/'autonomy_used.yaml').exists() else {}
  times=result.get('lap_times_seconds',[]);states=json.loads((folder/'states.json').read_text()) if (folder/'states.json').exists() else [];active,before=classify_safety_events(states,result.get('release_sim_time',0) or 0)
  complete=bool(times and result.get('termination')=='diagnostic_lap_goal' and not result.get('contacts') and not active and not result.get('error'))
  status='complete_clean' if complete else ('infrastructure_error' if result.get('error') else result.get('termination')) or ('diagnostic_complete' if result else 'running')
  last=(result.get('lap_records') or [{}])[-1]
  rows.append(dict(label=folder.name,track=result.get('track'),status=status,max_speed_effective=cfg.get('max_speed'),path_mode=cfg.get('path_mode') or parameters.get('path_mode','centerline'),planning_horizon=parameters.get('planning_horizon'),lateral_acceleration=parameters.get('max_lateral_acceleration'),planning_braking=parameters.get('braking_deceleration'),acceleration=parameters.get('acceleration_limit'),visibility_buffer=parameters.get('visibility_buffer_m',0),completed_laps=len(times),lap_times_s=times,first_lap_s=times[0] if times else None,total_goal_sim_s=last.get('elapsed_seconds'),goal_wall_from_arm_s=last.get('wall_elapsed_from_arm_s'),goal_wall_from_resume_s=last.get('wall_elapsed_from_resume_s'),goal_wall_including_lights_s=result.get('goal_wall_elapsed_from_sequence_s'),test_wall_s=result.get('wall_duration'),distance_m=result.get('distance_m'),contacts=len(result.get('contacts',[])) if result else None,active_safety_stops=len(active) if result else None,prestart_safety_events=len(before) if result else None,raw_stop_counter=max((s.get('safety_stops',0) for s in states),default=None),average_actual_speed=metrics.get('actual_average_pose_speed_mps'),maximum_encoder_speed=metrics.get('maximum_encoder_speed_mps'),processing_p95_ms=metrics.get('processing_ms',{}).get('p95_abs'),path_error_p95_m=(metrics.get('executed_path_error_0_25s_m') or {}).get('p95'),racing_selected_fraction=metrics.get('racing_selected_fraction'),under_600_two_lap_wall=metrics.get('self_two_lap_under_600wall',False),first_two_wall_from_arm_s=metrics.get('first_two_laps_wall_from_arm_s'),first_two_under600wall_clean=metrics.get('first_two_laps_under600wall_clean_full_trial',False),full_clean=complete))
 (out/'campaign_comparison.json').write_text(json.dumps(rows,indent=2)+'\n')
 if rows:
  with (out/'campaign_comparison.csv').open('w',newline='') as f:
   w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 tested={r['max_speed_effective'] for r in rows if r['status']!='running'}
 summary=dict(attempts=len(rows),clean_completed_attempts=sum(r['full_clean'] for r in rows),not_yet_tested_speed_caps=[v for v in [3.,3.5,4.,4.5,5.] if v not in tested],official_lap_certification=False,self_two_lap_under600wall=any(r['under_600_two_lap_wall'] or r['first_two_under600wall_clean'] for r in rows))
 (out/'campaign_status.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
 return rows
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();summarize(a.root.resolve())
