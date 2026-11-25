#!/usr/bin/env python3
"""Offline pose/contact validation against the original collision road mesh.
No control or ROS connection. Tests centres AND between-sample midpoints so a
shortcut across the hole/outside cannot be called a lap just by hitting gates.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
import matplotlib.tri as tri
p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--from-sim',type=float,default=-float('inf'));p.add_argument('--to-sim',type=float,default=float('inf'));a=p.parse_args();folder=Path(a.run);root=Path(__file__).resolve().parents[4]
state_path=folder/'lap_state.json'
if state_path.exists():
 state=json.loads(state_path.read_text())
 if state.get('finish_line') is not None:
  if math.isinf(a.to_sim):a.to_sim=state['finish_line']
  if math.isinf(a.from_sim):a.from_sim=state['first_line']
vertices=[];faces=[]
for line in (root/'src/urrc_track_gazebo/models/urrc_monaco/meshes/road_collision.obj').open():
 if line.startswith('v '):vertices.append(list(map(float,line.split()[1:])))
 elif line.startswith('f '):faces.append([int(v.split('/')[0])-1 for v in line.split()[1:]])
v=np.array(vertices);f=np.array(faces);n=len(v)//2;top=f[np.all(f<n,axis=1)]
finder=tri.Triangulation(v[:n,0],v[:n,1],top).get_trifinder()
poses=[json.loads(l) for l in (folder/'poses.jsonl').read_text().splitlines()];poses=[v for v in poses if a.from_sim<=v['sim']<=a.to_sim];xyz=np.array([q['xyz'] for q in poses]);t=np.array([q['sim'] for q in poses])
points=np.vstack((xyz[:,:2],.5*(xyz[:-1,:2]+xyz[1:,:2])));outside=finder(points[:,0],points[:,1])<0
contact_path=folder/'contacts.jsonl'
contacts=[json.loads(l) for l in contact_path.read_text().splitlines()] if contact_path.exists() else []
contacts=[r for r in contacts if a.from_sim<=r['sim']<=a.to_sim]
wall=[r for r in contacts if r.get('wall')];other=[r for r in contacts if 'pair' in r and not r.get('wall') and not r.get('road')];heart=[r['sim'] for r in contacts if r.get('heartbeat')]
events=[]
for st in sorted(set(r['sim'] for r in wall)):
 if not events or st-events[-1][-1]>.21:events.append([st,st])
 else:events[-1][-1]=st
out=dict(from_sim=a.from_sim if math.isfinite(a.from_sim) else None,to_sim=a.to_sim if math.isfinite(a.to_sim) else None,road_collision_top_triangles=len(top),poses=len(poses),outside_road_center_samples=int(np.sum(outside[:len(poses)])),outside_road_midpoints=int(np.sum(outside[len(poses):])),max_pose_step_m=float(np.max(np.linalg.norm(np.diff(xyz,axis=0),axis=1))),wall_contact_episodes=events,wall_contact_log_samples=len(wall),wall_max_recorded_penetration_m=max((r['max_depth'] for r in wall),default=0),other_collision_pairs=sorted(set(r['pair'] for r in other)),contact_heartbeat_range=None if not heart else [min(heart),max(heart)],max_contact_heartbeat_gap_s=None if len(heart)<2 else float(max(np.diff(heart))),road_contact_pairs=sorted(set(r['pair'] for r in contacts if r.get('road'))))
(folder/'integrity.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
