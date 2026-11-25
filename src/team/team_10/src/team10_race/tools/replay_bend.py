#!/usr/bin/env python3
"""Offline callback replay in a separate ROS domain. No simulator, no actuator bridge.
Recorded scans only; started is enabled only in this isolated computation harness.
"""
import argparse,gzip,json,math,sys,time,dataclasses
from pathlib import Path
import rclpy
from sensor_msgs.msg import LaserScan
from team10_race.relaxed_navigation_node import NavigationNode
p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True);p.add_argument('--from-sim',type=float,default=205);p.add_argument('--to-sim',type=float,default=212);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
rclpy.init(args=['--ros-args','--params-file','src/team/team_10/config/navigation.yaml','--log-level','error'])
n=NavigationNode();n.started=True
record={};audit=[];fits=[];tick=[0.];real_monotonic=time.monotonic
# Use recorded wall reception times for dt; no rclpy executor is spun.
import team10_race.relaxed_navigation_node as mod
mod.time.monotonic=lambda:tick[0]
def clean(x):
 if dataclasses.is_dataclass(x):return clean(dataclasses.asdict(x))
 if isinstance(x,dict):return {str(k):clean(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [clean(v) for v in x]
 if isinstance(x,float) and not math.isfinite(x):return None
 return x
f=(out/'callbacks.jsonl').open('w')
def trace(frame,event,arg):
 if event!='return':return trace
 name=frame.f_code.co_name;v=frame.f_locals
 if name=='fit_wall' and a.from_sim<=record.get('stamp',0)<=a.to_sim:
  fits.append(dict(side=v.get('side'),usable=len(v.get('usable',[])),bins=len(v.get('samples',[])),filtered=len(v.get('filtered',[])),rms=v.get('rms'),result=clean(arg),maximum_residual=v.get('maximum_residual_m')))
 if name=='_corridor_limit' and a.from_sim<=record.get('stamp',0)<=a.to_sim:
  hits=[]
  for q in v['obstacles']:
   for i,(b,c) in enumerate(zip(v['vertices'],v['vertices'][1:])):
    ratio,lat=n.speed_planner._projection_on_segment(q,b,c);long=v['cumulative'][i]+ratio*b.distance(c)
    if lat<=n.speed_planner.corridor_half_width and .05<long<=n.speed_planner.preview_distance:
     hits.append(dict(long=long,lateral=lat,obstacle=clean(q),segment=i,ratio=ratio,projection=clean(type(b)(b.x+ratio*(c.x-b.x),b.y+ratio*(c.y-b.y)))))
  audit.extend(sorted(hits,key=lambda z:z['long'])[:8])
 if name=='_scan_callback' and frame.f_code.co_filename.endswith('relaxed_navigation_node.py') and a.from_sim<=record.get('stamp',0)<=a.to_sim:
  keys=['segments','observation','polynomial','scan_centerline','centerline','corridor','race_centerline','command','speed_path','speed_obstacles','plan','wall_count','trajectory']
  row=dict(stamp=record['stamp'],wall=record['wall'],sim_age=record['sim_age'],state=n.state,desired_speed=n.desired_speed,desired_steering=n.desired_steering,scan=record,geometry={k:clean(v[k]) for k in keys if k in v},corridor_hits=audit[:],wall_fits=fits[:])
  f.write(json.dumps(row)+'\n');f.flush();audit.clear();fits.clear()
 return trace
try:
 for line in gzip.open(a.input,'rt'):
  record=json.loads(line);tick[0]=record['wall']
  if record['stamp']>a.to_sim:break
  sys.settrace(trace if record['stamp']>=a.from_sim else None)
  q=record;s=LaserScan();sec=int(q['stamp']);ns=round((q['stamp']-sec)*1e9)
  s.header.stamp.sec=sec;s.header.stamp.nanosec=ns;s.angle_min=q['angle_min'];s.angle_increment=q['angle_increment'];s.range_min=q['range_min'];s.range_max=q['range_max'];s.ranges=[x if x is not None else (math.nan if 'positive_infinity_indices' in q else math.inf) for x in q['ranges']]
  for i in q.get('positive_infinity_indices',[]):s.ranges[i]=math.inf
  for i in q.get('negative_infinity_indices',[]):s.ranges[i]=-math.inf
  if q['topic']=='/team_10/scan_10':n._scan_callback(s)
  elif q['topic']=='/team_10/opponent_scan_10':n._opponent_scan_callback(s)
  else:n._rear_opponent_scan_callback(s)
finally:
 sys.settrace(None);mod.time.monotonic=real_monotonic;f.close();n.destroy_node();rclpy.try_shutdown()
