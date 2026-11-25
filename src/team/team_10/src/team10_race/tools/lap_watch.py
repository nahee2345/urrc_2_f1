#!/usr/bin/env python3
"""Read-only preview lap verification using ordered operator gates.
SceneBroadcaster poses are validation-only, NOT an official timing record.
No ROS publishers, no vehicle control, no pose input to navigation.
"""
import argparse,csv,json,math,subprocess,sys,time
from collections import deque
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(root/'race/scripts'))
from timing import crossing,track_gates
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
if (out/'poses.jsonl').exists():raise SystemExit('Existing lap record: use a new output folder')
gates=track_gates(root,'monaco',10.)
rows=list(csv.DictReader((root/'src/urrc_track_gazebo/tracks/processed/monaco_centerline.csv').open()))
center=np.array([[float(r[k]) for k in ('x_m','y_m','z_m')] for r in rows]);length=np.r_[0,np.cumsum(np.linalg.norm(np.diff(center,axis=0),axis=1))]
raw=(out/'poses.jsonl').open('w',buffering=1);events=(out/'crossings.jsonl').open('w',buffering=1)
proc=subprocess.Popen(['gz','topic','-e','-t','/world/monaco/pose/info','--json-output'],stdout=subprocess.PIPE,text=True)
origin=None;previous=None;first=None;finish=None;next_gate=0;distance=0.;motion_start=None;lastwrite=0.;max_speed=0.;stops=[];stopped=None;reverse=[];reversing=None;window=deque();peak_gap=0.;latest={};first_line_distance=None
try:
 for line in proc.stdout:
  try:
   m=json.loads(line);v=next(x for x in m['pose'] if x.get('name')=='team_10');st=m['header']['stamp'];t=float(st.get('sec',0))+float(st.get('nsec',0))*1e-9
   pos=v['position'];q=v['orientation'];xyz=[pos.get(k,0.) for k in ('x','y','z')];yaw=math.atan2(2*(q.get('w',0)*q.get('z',0)+q.get('x',0)*q.get('y',0)),1-2*(q.get('y',0)**2+q.get('z',0)**2))
   sample=(t,*xyz)
   if previous and t<=previous[0]:continue
   raw.write(json.dumps(dict(wall=time.time(),sim=t,xyz=xyz,yaw=yaw))+'\n')
   if origin is None:origin=list(xyz)
   speed=0.;body_speed=0.
   if previous:
    dt=t-previous[0];peak_gap=max(peak_gap,dt);ds=math.dist(sample[1:],previous[1:]);distance+=ds
    if motion_start is None and math.dist(xyz[:2],origin[:2])>.05 and ds/dt>.15:motion_start=t
    window.append((t,list(xyz),distance))
    while window and t-window[0][0]>.4:window.popleft()
    if window and t-window[0][0]>.15:
     z=window[0];speed=(distance-z[2])/(t-z[0]);body_speed=((xyz[0]-z[1][0])*math.cos(yaw)+(xyz[1]-z[1][1])*math.sin(yaw))/(t-z[0]);max_speed=max(max_speed,speed)
    if motion_start is not None:
     if speed<.05:
      if stopped is None:stopped=t
     elif stopped is not None:
      if t-stopped>=2:stops.append([stopped,t])
      stopped=None
     if body_speed<-.2:
      if reversing is None:reversing=t
     elif reversing is not None:
      if t-reversing>=.5:reverse.append([reversing,t])
      reversing=None
    if finish is None:
     hit=crossing(previous,sample,gates[next_gate])
     if hit:
      hit.update(gate=next_gate,wall=time.time());events.write(json.dumps(hit)+'\n')
      if next_gate==0:
       if first is None:first=hit['time'];first_line_distance=distance
       else:finish=hit['time']
      next_gate=(next_gate+1)%len(gates)
   previous=sample
   if time.monotonic()-lastwrite>=1:
    idx=int(np.argmin(np.sum((center-np.array(xyz))**2,axis=1)))
    latest=dict(measurement='preview verification, not official race record',sim=t,wall=time.time(),xyz=xyz,yaw=yaw,speed_mps=speed,body_forward_mps=body_speed,max_speed_0_15_to_0_4s_mps=max_speed,distance_m=distance,motion_start=motion_start,first_line=first,finish_line=finish,lap_seconds=None if finish is None else finish-first,next_gate=next_gate,total_gates=len(gates),nearest_track_s=float(length[idx]),nearest_track_distance=float(np.linalg.norm(center[idx]-xyz)),max_pose_gap_s=peak_gap,stops=stops,ongoing_stop_s=None if stopped is None else t-stopped,reverse_episodes=reverse,ongoing_reverse_s=None if reversing is None else t-reversing,wall_contact='not instrumented',average_since_motion_mps=None if motion_start is None or t<=motion_start else distance/(t-motion_start))
    (out/'lap_state.json').write_text(json.dumps(latest,indent=2));print(json.dumps(latest),flush=True);lastwrite=time.monotonic()
  except (ValueError,KeyError,StopIteration):continue
except KeyboardInterrupt:pass
finally:
 proc.terminate();raw.close();events.close()
