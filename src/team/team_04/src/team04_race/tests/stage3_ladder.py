#!/usr/bin/env python3
"""Sequential, measured speed ladder. Refuses the next speed after a failed run."""
from pathlib import Path
import argparse,json,os,subprocess,sys
from analyze_stage3 import analyze3
from trial_metrics import TrialMetrics

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--prefix',required=True);p.add_argument('--speeds',type=float,nargs='+',default=[1.5,2.,2.5,3.]);p.add_argument('--repeat-laps',type=int,default=5);p.add_argument('--autonomy-config',type=Path);args=p.parse_args()
 root=args.root.resolve();out=root/'src/team_04/artifacts/stage3';out.mkdir(exist_ok=True);script=Path(__file__).with_name('autonomy_trial.py');runs=[];configuration=[]
 if args.autonomy_config:configuration=['--autonomy-config',str(args.autonomy_config.resolve())]
 def execute(speed,laps,tag):
  label=f'{args.prefix}-{tag}';folder=out/label
  command=[sys.executable,'-s',str(script),'--isolated','--root',str(root),'--label',label,'--track','monza','--duration',str(max(1800,laps*900)),'--autonomous','--max-speed',str(speed),'--stop-after-laps',str(laps),'--stop-on-active-failure',*configuration]
  with (out/(label+'.console.log')).open('w') as log:status=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT).returncode
  if status or not (folder/'result.json').is_file():return dict(label=label,max_speed=speed,complete=False,error='trial process failed')
  os.environ['MPLCONFIGDIR']=str(folder/'plot_cache');analyze3(folder,root)
  result=json.loads((folder/'result.json').read_text());m=json.loads((folder/'metrics.json').read_text());times=m.get('lap_times_seconds',[])
  complete=bool(len(times)>=laps and result.get('termination')=='diagnostic_lap_goal' and not result.get('contacts') and not m['active_safety_stops'])
  row=dict(label=label,max_speed=speed,requested_laps=laps,complete=complete,lap_times=times,first_lap_s=times[0] if times else None,average_actual_speed=m.get('actual_average_pose_speed_mps'),contacts=m['contacts'],active_safety_stops=m['active_safety_stops'],raw_stop_counter=m['raw_stop_counter'],after_alignment_error=m.get('after_alignment_lateral_error_m'),processing_ms=m.get('processing_ms'),rtf=m.get('active_rtf'),under_600=bool(complete and all(t<600 for t in times)))
  return row
 def save():
  (out/(args.prefix+'-comparison.json')).write_text(json.dumps(runs,indent=2)+'\n')
  TrialMetrics.csv(out/(args.prefix+'-comparison.csv'),runs,['label','max_speed','requested_laps','complete','first_lap_s','average_actual_speed','contacts','active_safety_stops','raw_stop_counter','under_600','lap_times'])
 for speed in args.speeds:
  row=execute(speed,1,f'monza-{round(speed*100):03d}');runs.append(row);save();print(json.dumps(row),flush=True)
  if not row['complete']:
   print('STOP: resolve this speed before trying the next one.',flush=True);return 1
 candidates=[r for r in runs if r.get('under_600')]
 if candidates and args.repeat_laps:
  best=min(candidates,key=lambda r:r['first_lap_s']);row=execute(best['max_speed'],args.repeat_laps,f"monza-{round(best['max_speed']*100):03d}-continuous-{args.repeat_laps}");runs.append(row);save();print(json.dumps(row),flush=True)
  if not row['complete']:return 1
 return 0
if __name__=='__main__':sys.exit(main())
