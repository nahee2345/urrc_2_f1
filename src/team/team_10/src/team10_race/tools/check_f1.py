#!/usr/bin/env python3
"""Read-only readiness check before READY. Run after spawn, before permission."""
import json,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,LaserScan
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String,Float32
rclpy.init();n=Node('f1_readiness');data={};counts={};first={};received={}
stable_since=None;stable_sim=None;reference=None;image=None;pixel_change=None
def cb(t,m):
 global stable_since,image
 now=time.monotonic()
 if t in ('/team_10/start_camera_image_10','/team_10/scan_10','/clock') and now-received.get(t,now)>.25:stable_since=None
 counts[t]=counts.get(t,0)+1;data[t]=m;received[t]=time.monotonic();first.setdefault(t,m)
 if t=='/team_10/start_camera_image_10':
  image=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)[::4,::4].copy()
for t,kind in [('/clock',Clock),('/team_10/start_camera_image_10',Image),('/team_10/scan_10',LaserScan),('/team_10/start_status_10',String),('/team_10/speed_10',Float32),('/team_10/steering_10',Float32)]:
 n.create_subscription(kind,t,lambda m,t=t:cb(t,m),qos_profile_sensor_data)
end=time.monotonic()+20
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.05)
 if len(data)<6 or min(counts.values())<3:continue
 now=data['/clock'].clock;sim=now.sec+now.nanosec*1e-9
 prev=first['/clock'].clock
 status=json.loads(data['/team_10/start_status_10'].data)
 ages={t:sim-(data[t].header.stamp.sec+data[t].header.stamp.nanosec*1e-9) for t in ('/team_10/start_camera_image_10','/team_10/scan_10')}
 required={'navigation_10','race_adapter_10','start_camera_10'}
 found={e.node_name for e in n.get_subscriptions_info_by_topic('/race/start/team_10')}
 fresh=(all(time.monotonic()-received[t]<.25 for t in ('/clock','/team_10/start_camera_image_10','/team_10/scan_10')))
 valid=(sim>prev.sec+prev.nanosec*1e-9 and all(-.1<=a<=.25 for a in ages.values()) and fresh
     and not status['allowed'] and required<=found and data['/team_10/speed_10'].data==0 and data['/team_10/steering_10'].data==0)
 if not valid:stable_since=None;continue
 if stable_since is None:
  stable_since=time.monotonic();stable_sim=sim;reference=image.copy()
 difference=np.abs(image.astype(np.int16)-reference.astype(np.int16))
 pixel_change=float(np.mean(difference))
 if pixel_change>.75 or np.mean(difference>15)>.005:
  stable_since=None;continue
 # Observed fresh, stationary camera view, not a fixed delay that permits GO.
 # Existing sensor age/lease limits and 20s readiness wait remain unchanged.
 if time.monotonic()-stable_since>=2 and sim-stable_sim>=.5:
  print(json.dumps(dict(result='PASS',clock=sim,sensor_sim_age=ages,counts=counts,start=status,ready_subscribers=sorted(found),stable_wall_s=time.monotonic()-stable_since,stable_sim_s=sim-stable_sim,pixel_change=pixel_change),indent=2));break
else:raise SystemExit(f'NOT READY: counts={counts}; inspect /clock, sensors and node logs')
n.destroy_node();rclpy.try_shutdown()
