#!/usr/bin/env python3
"""Run the three read-only recorders and clean up only their process groups."""
import argparse,os,signal,subprocess,sys,time,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
folder=Path(a.output).resolve()
if (folder/'poses.jsonl').exists():raise SystemExit('Existing lap record: use a new output folder')
folder.mkdir(parents=True,exist_ok=True);here=Path(__file__).resolve().parent
children=[];files=[]
def interrupt(*args):raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupt)
try:
 for name,command in [
  ('clock',[sys.executable,str(here/'record_speed_clock.py'),'--output',str(folder/'clock.jsonl')]),
  ('contacts',[str(here.parents[3]/'build_f1/telemetry/contact_recorder'),str(folder/'contacts.jsonl')]),
  ('observer',[sys.executable,str(here/'observe_f1.py')]),
  ('lap_watch',[sys.executable,str(here/'lap_watch.py'),'--output',str(folder)]),
  ('scans',[sys.executable,str(here/'log_scans.py'),'--output',str(folder/'scans.jsonl.gz')])]:
  f=(folder/(name+'.log')).open('w');files.append(f)
  children.append(subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
 print(f'Recording {folder}; chain: validation_f1/chain.jsonl; Ctrl+C stops recorders only',flush=True)
 while True:
  for c in children:
   if c.poll() is not None:raise RuntimeError(f'recorder exited: pid={c.pid} code={c.returncode}')
  time.sleep(.5)
except KeyboardInterrupt:pass
finally:
 for c in children:
  if c.poll() is None:
   try:os.killpg(c.pid,signal.SIGINT)
   except ProcessLookupError:pass
 for c in children:
  try:c.wait(timeout=5)
  except subprocess.TimeoutExpired:
   try:os.killpg(c.pid,signal.SIGTERM)
   except ProcessLookupError:pass
   c.wait(timeout=3)
 for f in files:f.close()
 chain=here.parents[3]/'validation_f1/chain.jsonl'
 if chain.exists() and not (folder/'chain.jsonl').exists():shutil.copy2(chain,folder/'chain.jsonl')
