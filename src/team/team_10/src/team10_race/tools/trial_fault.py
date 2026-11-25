#!/usr/bin/env python3
"""Validation only: inject STOP or interrupt only the image bridge after real start.
Never publishes speed, steering, camera data, or lights-out permission; no pose input.
"""
import json,os,signal,sys,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,Float32
rclpy.init();n=Node('f1_fault_trial');speed=0.;gate=False;moving=None;done=None;pid=None
mode=sys.argv[1];assert mode in ('false','camera_drop')
pub=n.create_publisher(Bool,'/race/start/team_10',10)
def on_speed(m):
 global speed
 speed=m.data
def on_gate(m):
 global gate
 gate=m.data
n.create_subscription(Float32,'/team_10/speed_10',on_speed,10)
n.create_subscription(Bool,'/team_10/camera_gate_10',on_gate,10)
folder=Path(__file__).resolve().parents[4]/'validation_f1'
def event(kind):
 row=dict(wall=time.time(),mode=mode,event=kind,speed=speed,gate=gate,pid=pid)
 with (folder/'fault_events.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
 print(row,flush=True)
event('waiting_for_real_camera_start')
end=time.monotonic()+180
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.02);now=time.monotonic()
 if moving is None and gate and speed>.3:moving=now;event('moving')
 if moving is not None and done is None and now-moving>=3:
  if mode=='false':
   needed={'navigation_10','race_adapter_10','start_camera_10'}
   assert needed<={e.node_name for e in n.get_subscriptions_info_by_topic('/race/start/team_10')}
   for _ in range(5):pub.publish(Bool(data=False));rclpy.spin_once(n,timeout_sec=.05)
  else:
   pids=[]
   for p in Path('/proc').glob('[0-9]*'):
    try:
     cmd=(p/'cmdline').read_bytes()
     if b'parameter_bridge' in cmd and b'__node:=image_bridge_10' in cmd:pids.append(int(p.name))
    except OSError:pass
   assert len(pids)==1,pids
   pid=pids[0];os.kill(pid,signal.SIGSTOP)
  done=now;event('fault_injected')
 if done is not None and now-done>=3:
  event('after_3s')
  if mode=='camera_drop':os.kill(pid,signal.SIGCONT);event('image_bridge_resumed')
  break
else:raise SystemExit('No real camera-triggered movement observed')
n.destroy_node();rclpy.try_shutdown()
