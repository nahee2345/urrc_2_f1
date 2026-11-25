#!/usr/bin/env python3
"""Saved-scan computation only, isolated ROS domain, no executor/actuator bridge."""
import argparse
import gzip
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace
import rclpy
from sensor_msgs.msg import LaserScan
from team10_race.relaxed_navigation_node import NavigationNode

p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True)
p.add_argument('--from-sim',type=float,required=True);p.add_argument('--to-sim',type=float,required=True)
p.add_argument('--max-speed',type=float,help='Isolated replay only; does not edit runtime settings');p.add_argument('--delay-history',help='Original command/independent-clock record for fixed-input estimator replay');p.add_argument('--step',type=float,default=.099);p.add_argument('--legacy',action='store_true');a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
rclpy.init(args=['--ros-args','--params-file','src/team/team_10/config/navigation.yaml','--log-level','error']+([] if a.max_speed is None else ['-p','speed.max_mps:='+str(a.max_speed)]))
n=NavigationNode();n.started=True
if a.legacy:n.speed_planner.continuous_limits=False
n.speed_audit=SimpleNamespace(enabled=True,close=lambda:None)
from team10_race.path_safety import verify_path
scan_holder=[None];body_holder=[None];compute=n.controller.compute
def checked_compute(*args,**kwargs):
    command=compute(*args,**kwargs)
    if command is not None:body_holder[0]=vars(verify_path(scan_holder[0],command.soft_path))
    return command
n.controller.compute=checked_compute
recorded=[];commands=[]
if a.delay_history:
    from speed_metrics import read,external_clock,calibrate
    import bisect
    folder=Path(a.delay_history);c,_=external_clock(folder)
    recorded=[r for r in read(folder/'audit/navigation.jsonl') if r.get('plan')];calibrate(recorded,c)
    commands=[r for r in read(folder/'audit/adapter.jsonl') if r.get('event')=='output'];calibrate(commands,c)
    stamps=[r['scan_stamp'] for r in recorded];ct=[r['sim'] for r in commands]
last={};rows=[];clock=[0.];original=time.monotonic;time.monotonic=lambda:clock[0]
try:
    with gzip.open(a.input,'rt') as f:
        for line in f:
            q=json.loads(line);st=q['stamp']
            if st<a.from_sim:continue
            if st>a.to_sim:break
            if st-last.get(q['topic'],-math.inf)<a.step:continue
            last[q['topic']]=st;clock[0]=q['wall'];s=LaserScan()
            sec=int(st);ns=round((st-sec)*1e9);s.header.stamp.sec=sec;s.header.stamp.nanosec=ns
            for k in ['angle_min','angle_increment','range_min','range_max']:setattr(s,k,q[k])
            ranges=[v if v is not None else math.nan for v in q['ranges']]
            for i in q.get('positive_infinity_indices',[]):ranges[i]=math.inf
            for i in q.get('negative_infinity_indices',[]):ranges[i]=-math.inf
            s.ranges=ranges
            if q['topic']=='/team_10/scan_10':
                scan_holder[0]=s;body_holder[0]=None
                if recorded:
                    ri=min(len(recorded)-1,bisect.bisect_left(stamps,st));done=recorded[ri]['sim']
                    ci=bisect.bisect_right(ct,done)
                    n._control_clock=(done,clock[0])
                    n._command_history.clear()
                    n._command_history.extend((r['sim'],r['speed'],r['steering']) for r in commands[max(0,ci-200):ci])
                n._audit_plan=None;n._process_scan(s)
                rows.append(dict(sim=st,state=n.state,speed=n.desired_speed,plan=n._audit_plan,body=body_holder[0]))
            elif q['topic']=='/team_10/opponent_scan_10':n._opponent_scan_callback(s)
            else:n._rear_opponent_scan_callback(s)
finally:
    time.monotonic=original;n.destroy_node();rclpy.try_shutdown()
(out/'plans.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
from collections import Counter
summary=dict(mode='isolated fixed-input replay; no closed-loop performance claim',frames=len(rows),
    states=dict(Counter(r['state'] for r in rows)),zero_frames=sum(r['speed']==0 for r in rows),
    max_command=max((r['speed'] for r in rows),default=0))
(out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
