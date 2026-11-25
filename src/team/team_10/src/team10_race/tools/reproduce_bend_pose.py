#!/usr/bin/env python3
"""ONE validation initialisation from a measured pose. Never used for a lap run.
Must first complete real camera start at grid 1. No velocity/steering publication.
Recorded poses only contain yaw: roll/pitch are initialised to zero, so this is a
pose reproduction, not an exact replay of joint/velocity state.
"""
import argparse,json,math,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
p=argparse.ArgumentParser();p.add_argument('--poses',required=True);p.add_argument('--sim',type=float,required=True);p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init();n=Node('f1_validation_initialisation');gate=[False]
n.create_subscription(Bool,'/team_10/camera_gate_10',lambda v:gate.__setitem__(0,v.data),qos_profile_sensor_data)
end=time.monotonic()+3
while not gate[0] and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
assert gate[0],'First complete actual camera start; never bypass the gate'
rows=[json.loads(x) for x in Path(a.poses).read_text().splitlines()];r=min(rows,key=lambda x:abs(x['sim']-a.sim));x,y,z=r['xyz'];yaw=r['yaw']
req=f'name: "team_10" position {{ x: {x} y: {y} z: {z} }} orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }}'
event=dict(wall=time.time(),source=a.poses,source_pose=r,kind='single validation pose initialization, NOT lap',roll_pitch='zero; unrecorded in source',velocity_and_joints='not set; current simulated values retained or reset by SetPose implementation')
result=subprocess.run(['gz','service','-s','/world/monaco/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','3000','--req',req],capture_output=True,text=True);event['response']=result.stdout;Path(a.output).write_text(json.dumps(event,indent=2));print(json.dumps(event,indent=2));assert 'data: true' in result.stdout
n.destroy_node();rclpy.try_shutdown()
