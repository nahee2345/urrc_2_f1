#!/usr/bin/env python3
"""Validation-only acquisition-time LiDAR recorder, no publishers."""
import argparse,gzip,json,math,time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import LaserScan
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init();n=Node('f1_scan_recorder',parameter_overrides=[Parameter('use_sim_time',value=True)])
f=gzip.open(a.output,'wt',compresslevel=1);last={}
def cb(t,m):
 st=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if t!='/team_10/scan_10' and st-last.get(t,-1)<.09:return
 last[t]=st
 f.write(json.dumps(dict(wall=time.time(),stamp=st,sim_age=n.get_clock().now().nanoseconds*1e-9-st,topic=t,positive_infinity_indices=[i for i,v in enumerate(m.ranges) if math.isinf(v) and v>0],nan_indices=[i for i,v in enumerate(m.ranges) if math.isnan(v)],negative_infinity_indices=[i for i,v in enumerate(m.ranges) if math.isinf(v) and v<0],angle_min=m.angle_min,angle_increment=m.angle_increment,range_min=m.range_min,range_max=m.range_max,ranges=[x if math.isfinite(x) else None for x in m.ranges]))+'\n');f.flush()
for t in ['/team_10/scan_10','/team_10/opponent_scan_10','/team_10/rear_opponent_scan_10']:
 n.create_subscription(LaserScan,t,lambda m,t=t:cb(t,m),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:f.close();n.destroy_node();rclpy.try_shutdown()
