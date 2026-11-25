#!/usr/bin/env python3
"""Short start tests: native image timing, ROS chain, measured poses. No publishers."""
import argparse,os,signal,subprocess,time,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=True)
if (out/'poses.jsonl').exists():raise SystemExit('Use a new folder')
here=Path(__file__).resolve().parent;root=here.parents[3];children=[];files=[]
try:
 for name,cmd in [
  ('native_camera',[str(root/'build_f1/telemetry/camera_timing'),str(out/'native_camera.jsonl')]),
  ('observer',['python3',str(here/'observe_f1.py')]),
  ('poses',['python3',str(here/'lap_watch.py'),'--output',str(out)])]:
  f=(out/(name+'.log')).open('w');files.append(f)
  children.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
 print('Recording '+str(out),flush=True)
 while True:
  if any(c.poll() is not None for c in children):raise RuntimeError('Recorder exited')
  time.sleep(.5)
except KeyboardInterrupt:pass
finally:
 for c in children:
  if c.poll() is None:
   try:os.killpg(c.pid,signal.SIGINT)
   except ProcessLookupError:pass
 for c in children:
  try:c.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(c.pid,signal.SIGTERM);c.wait(timeout=3)
 for f in files:f.close()
 shutil.copy2(root/'validation_f1/chain.jsonl',out/'chain.jsonl')
