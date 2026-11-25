#!/usr/bin/env python3
"""Run only missing isolated physical scenarios, serially. Stop on contact/crash."""
from pathlib import Path
import argparse,json,os,subprocess,sys
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();root=a.root.resolve();out=root/'src/team_04/artifacts/stage5';tests=root/'src/team_04/team04_race/tests';env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ROS_LOG_DIR=str(out/'ros_logs'))
cases=[('stage5-s03-stopped','monza','stopped','solo_racing',25),('stage5-f01-follow-center','monza','follow','solo_racing',30),('stage5-f02-follow-offset','monza','follow_offset','solo_racing',35),('stage5-b01-brake','monza','brake','solo_racing',30),('stage5-h02-right-clear','hungaroring','right_clear','traffic_racing',40),('stage5-n01-blocked','monza','blocked','traffic_racing',25),('stage5-c01-corner','monza','corner','traffic_racing',30),('stage5-m01-multi','monza','multi','traffic_racing',30),('stage5-o01-occlusion','monza','occlusion','traffic_racing',30),('stage5-h03-cutin','hungaroring','cutin','traffic_racing',30),('stage5-h04-multi','hungaroring','multi','traffic_racing',30)]
results=[]
for label,track,scenario,mode,duration in cases:
 folder=out/label
 if folder.exists():
  if (folder/'result.json').exists():results.append(dict(label=label,status='existing result retained'));continue
  raise FileExistsError(f'Incomplete folder requires inspection, never overwritten: {folder}')
 (out/'campaign_progress.json').write_text(json.dumps(dict(current=label,completed=results),indent=2)+'\n');print('START '+label,flush=True)
 with (out/(label+'.console.log')).open('w') as f:
  process=subprocess.Popen([sys.executable,'-s',str(tests/'traffic_trial.py'),'--isolated','--root',str(root),'--track',track,'--label',label,'--scenario',scenario,'--traffic-mode',mode,'--autonomous','--max-speed','1.0','--path-mode','racing_line','--duration',str(duration)],env=env,stdout=f,stderr=subprocess.STDOUT)
  code=process.wait()
 if code or not (folder/'result.json').exists():
  subprocess.run([sys.executable,'-s',str(tests/'cleanup_traffic_partition.py'),'team04_development_'+str(process.pid),'--output',str(folder/'cleanup_automatic.json')],env=env,check=True)
  results.append(dict(label=label,status='infrastructure_failure',returncode=code));break
 result=json.loads((folder/'result.json').read_text());entry=dict(label=label,returncode=code,termination=result.get('termination'),contacts=len(result.get('contacts',[])),distance_m=result.get('distance_m'))
 with (out/(label+'-analysis.log')).open('w') as f:entry['analysis_returncode']=subprocess.run([sys.executable,'-s',str(tests/'analyze_stage5.py'),str(folder),'--root',str(root)],env=env,stdout=f,stderr=subprocess.STDOUT).returncode
 if (folder/'metrics.json').exists():
  m=json.loads((folder/'metrics.json').read_text());entry.update(strict_overtake_success=m.get('strict_overtake_success'),attempts=m.get('overtake_attempts'),completed_passes=m.get('overtake_completions'),aborted=m.get('overtake_aborts'),minimum_gap_m=m.get('minimum_full_footprint_gap_m'))
 results.append(entry);print(json.dumps(entry),flush=True)
 if result.get('contacts') or result.get('error'):break
(out/'campaign_progress.json').write_text(json.dumps(dict(current=None,completed=results),indent=2)+'\n')
print('CAMPAIGN FINISHED',flush=True)
