#!/usr/bin/env python3
"""Independent /clock receive trace for offline age measurement, no publishers."""
import argparse,json,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init();n=Node('speed_clock_recorder');f=open(a.output,'x',buffering=262144);last=[0.];flush=[0.]
def cb(m):
    tm=time.monotonic()
    if tm-last[0]<.02:return
    last[0]=tm
    f.write(json.dumps(dict(wall=time.time(),mono=tm,sim=m.clock.sec+m.clock.nanosec*1e-9))+'\n')
    if tm-flush[0]>.5:f.flush();flush[0]=tm
n.create_subscription(Clock,'/clock',cb,qos_profile_sensor_data)
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:f.close();n.destroy_node();rclpy.try_shutdown()
