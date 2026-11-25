#!/usr/bin/env python3
"""Read-only ROS endpoint/QoS and GZ actuator connection evidence."""
import json,os,subprocess,time
import rclpy
from rclpy.node import Node
rclpy.init();n=Node('f1_graph_probe')
topics={'/clock':('clock_bridge_10',),'/team_10/start_camera_image_10':('image_bridge_10','start_camera_10'),'/team_10/scan_10':('vehicle_bridge_10','navigation_10'),'/team_10/camera_gate_10':('start_camera_10','navigation_10','race_adapter_10'),'/team_10/gz_cmd_vel_10':('vehicle_bridge_10','race_adapter_10')}
end=time.monotonic()+20
while True:
 rclpy.spin_once(n,timeout_sec=.1)
 result={};ok=True
 for t,names in topics.items():
  endpoints=n.get_publishers_info_by_topic(t)+n.get_subscriptions_info_by_topic(t)
  result[t]=[dict(node=e.node_name,endpoint=e.endpoint_type.name,reliability=e.qos_profile.reliability.name,durability=e.qos_profile.durability.name,depth=e.qos_profile.depth) for e in endpoints]
  ok &= set(names)<={e.node_name for e in endpoints}
 if ok or time.monotonic()>end:break
print(json.dumps(dict(complete=ok,GZ_PARTITION=os.environ.get('GZ_PARTITION'),ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID'),endpoints=result),indent=2))
print(subprocess.run(['gz','topic','-i','-t','/model/team_10/cmd_vel'],capture_output=True,text=True,timeout=5).stdout)
n.destroy_node();rclpy.try_shutdown()
if not ok:raise SystemExit(1)
