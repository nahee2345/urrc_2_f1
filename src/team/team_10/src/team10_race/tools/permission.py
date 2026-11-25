#!/usr/bin/env python3
"""Deliver preparation/stop to all three safety nodes, after discovery."""
import sys,time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
rclpy.init();n=Node('f1_prepare_permission')
pub=n.create_publisher(Bool,'/race/start/team_10',10)
required={'navigation_10','race_adapter_10','start_camera_10'}
end=time.monotonic()+20
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 found={e.node_name for e in n.get_subscriptions_info_by_topic('/race/start/team_10')}
 if required<=found:break
else:raise SystemExit(f'Missing safety subscribers: {required-found}')
value={'true':True,'false':False}[sys.argv[1]]
for _ in range(10):
 pub.publish(Bool(data=value));rclpy.spin_once(n,timeout_sec=.1)
print(f'prepare_permission={value} delivered with subscribers={sorted(found)}')
n.destroy_node();rclpy.try_shutdown()
