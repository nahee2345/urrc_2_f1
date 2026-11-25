"""Real production launch + stamped sensor fixtures; deliberately unread console.
No Gazebo/user process is joined. Private ROS domain and exact cleanup marker.
"""
import os,sys,time,json,math,fcntl,termios,array,threading,subprocess
from pathlib import Path
import numpy as np
out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=True)
os.environ['ROS_DOMAIN_ID']='156'
import rclpy
from std_msgs.msg import String,Header,Bool,Float32
from sensor_msgs.msg import LaserScan,Imu
from rclpy.qos import qos_profile_sensor_data
from team_01_race.camera_bundle import encode_bundle
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_process_cleanup import stop_partition_processes
marker='team_01_output_probe_'+str(os.getpid())
env=dict(os.environ,GZ_PARTITION=marker,ROS_LOG_DIR=str(out/'ros'))
proc=subprocess.Popen(['ros2','launch','team_01_race','race.launch.py','use_sim_time:=false','bridge_sensors:=false'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env,start_new_session=True)
fcntl.fcntl(proc.stdout,fcntl.F_SETPIPE_SZ,4096)
rclpy.init();n=rclpy.create_node('launch_output_observer');rows=[];reader=None;console=[]
camera=n.create_publisher(String,'/team_01/camera/geometry_bundle',10)
scan=n.create_publisher(LaserScan,'/team_01/scan',qos_profile_sensor_data)
imu=n.create_publisher(Imu,'/team_01/imu',qos_profile_sensor_data)
start=n.create_publisher(Bool,'/race/start/team_01',10)
speeds=[];traces=[]
n.create_subscription(Float32,'/team_01/speed',lambda m:speeds.append((time.monotonic(),m.data)),10)
n.create_subscription(String,'/team_01/debug/speed_control',lambda m:traces.append((time.monotonic(),json.loads(m.data))),10)
x=np.arange(.1,5.01,.1);p=np.column_stack((x,np.zeros(len(x))))
def frames(duration,red=False):
 end=time.monotonic()+duration
 while time.monotonic()<end:
  h=Header(frame_id='team_01/base_link',stamp=n.get_clock().now().to_msg())
  camera.publish(String(data=encode_bundle(dict(path=p,left=p+[0,1.2],right=p-[0,1.2],confidence=1.,observed=(True,True),light='RED' if red else 'LIGHTS_OUT',reason='fixture'),h)))
  start.publish(Bool(data=not red))
  a=Imu(header=Header(frame_id='team_01/imu_link',stamp=h.stamp));a.orientation.w=1.;imu.publish(a)
  s=LaserScan(header=h,angle_min=-math.pi/2,angle_max=math.pi/2,angle_increment=math.pi/360,range_min=.04,range_max=6.)
  angles=s.angle_min+np.arange(361)*s.angle_increment;r=1.2/np.maximum(abs(np.sin(angles)),1e-9);r[r>6]=math.inf;s.ranges=r.tolist();scan.publish(s)
  next_frame=time.monotonic()+.04
  while time.monotonic()<next_frame:rclpy.spin_once(n,timeout_sec=.001)
def snapshot():
 now=time.monotonic();recent=[v for t,v in speeds if t>now-1.]
 b=array.array('i',[0]);fcntl.ioctl(proc.stdout,termios.FIONREAD,b)
 return dict(speed_messages_last_sec=len(recent),max_speed_last_sec=max(recent,default=None),min_speed_last_sec=min(recent,default=None),
             last_speed_age_sec=now-speeds[-1][0] if speeds else None,console_unread_bytes=b[0],
             last_trace=traces[-1][1] if traces else None,launch_exit=proc.poll())
def drain():
 for line in iter(proc.stdout.readline,b''):console.append(line.decode(errors='replace'))
try:
 deadline=time.monotonic()+8
 while time.monotonic()<deadline and (scan.get_subscription_count()<2 or camera.get_subscription_count()<2):rclpy.spin_once(n,timeout_sec=.01)
 frames(.8,red=True);red_phase=snapshot();frames(6.)
 unread=snapshot()
 reader=threading.Thread(target=drain,daemon=True);reader.start()
 frames(4.);drained=snapshot()
 (out/'result.json').write_text(json.dumps(dict(red_phase=red_phase,unread=unread,drained=drained),indent=2))
 print(json.dumps(dict(red_phase=red_phase,unread=unread,drained=drained),indent=2),flush=True)
finally:
 if reader is None:reader=threading.Thread(target=drain,daemon=True);reader.start()
 stop_partition_processes(marker)
 try:proc.wait(timeout=5)
 except subprocess.TimeoutExpired:proc.kill();proc.wait()
 reader.join(timeout=2);(out/'console.log').write_text(''.join(console))
 n.destroy_node();rclpy.shutdown()
