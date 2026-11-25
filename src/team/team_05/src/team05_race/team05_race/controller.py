import math
import time
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,Empty,String,Float64

from .planner import plan,clamp

class Driver(Node):
    def __init__(self):
        super().__init__('team05_controller')

        defaults={
            'race_start_topic':'/race/start/team_05',
            'race_start_mode':'signal',
            'speed_topic':'/team_05/speed',
            'steering_topic':'/team_05/steering',
            'lidar_topic':'/team_05/scan',
            'max_speed':.85
        }

        for k,v in defaults.items():
            self.declare_parameter(k,v)

        get=lambda k:self.get_parameter(k).value

        self.started=get('race_start_mode')=='immediate'
        self.gate_topic=get('race_start_topic')
        self.gate_type=Bool
        self.gate_sub=self.create_subscription(
            Bool,self.gate_topic,self.gate,10)

        self.cmd=self.create_publisher(
            Twist,'/model/team_05/cmd_vel',10)
        self.speed_pub=self.create_publisher(
            Float64,get('speed_topic'),10)
        self.steer_pub=self.create_publisher(
            Float64,get('steering_topic'),10)

        self.create_subscription(
            LaserScan,get('lidar_topic'),
            self.scan,qos_profile_sensor_data)

        self.create_subscription(
            Image,'/team_05/camera/image_raw',
            self.camera,qos_profile_sensor_data)

        self.create_subscription(
            Odometry,'/model/team_05/odometry',
            self.odom,qos_profile_sensor_data)

        self.points=None
        self.last_scan=0.
        self.last_odom=0.
        self.front=None
        self.closing=0.
        self.v=0.
        self.k=0.
        self.scans=0
        self.images=0
        self.last_report=0.
        self.last_gate=0.
        self.last_loop=time.monotonic()
        self.max_speed=clamp(float(get('max_speed')),.1,1.2)

        self.timer=self.create_timer(
            .10,self.step,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info('TEAM05 READY')

    def gate(self,msg):
        if isinstance(msg,Bool):
            self.started=msg.data
        elif isinstance(msg,Empty):
            self.started=True
        elif isinstance(msg,String):
            s=msg.data.strip().lower()
            if s in ('go','start','true','1','green'):
                self.started=True
            elif s in ('stop','false','0','red'):
                self.started=False

        if not self.started:
            self.send(0.,0.)

    def check_gate(self):
        types={
            'std_msgs/msg/Bool':Bool,
            'std_msgs/msg/Empty':Empty,
            'std_msgs/msg/String':String
        }
        found={
            x.topic_type for x in
            self.get_publishers_info_by_topic(self.gate_topic)
        }
        if len(found)==1:
            t=types.get(next(iter(found)))
            if t and t is not self.gate_type:
                self.destroy_subscription(self.gate_sub)
                self.gate_type=t
                self.gate_sub=self.create_subscription(
                    t,self.gate_topic,self.gate,10)

    def camera(self,msg):
        self.images+=1

    def odom(self,msg):
        p=msg.pose.pose.position
        if all(math.isfinite(x) for x in (p.x,p.y,p.z)):
            self.last_odom=time.monotonic()

    def scan(self,msg):
        now=time.monotonic()
        r=np.asarray(msg.ranges,dtype=float)

        if len(r)<30 or not math.isfinite(
                msg.angle_increment) or msg.angle_increment<=0:
            self.points=None
            return

        a=msg.angle_min+np.arange(len(r))*msg.angle_increment
        valid=(np.isfinite(r)
               &(r>=msg.range_min)
               &(r<=msg.range_max))

        healthy=valid|np.isposinf(r)

        if np.mean(healthy)<.8:
            self.points=None
            return

        p=np.column_stack((
            r[valid]*np.cos(a[valid]),
            r[valid]*np.sin(a[valid])
        ))

        if len(p)<3:
            self.points=None
            return

        front=p[(p[:,0]>.05)&(np.abs(p[:,1])<.18),0]
        d=float(min(front)) if len(front) else 3.
        dt=now-self.last_scan

        if self.front is not None and .015<dt<.7:
            raw=clamp((self.front-d)/dt,0.,2.)
            self.closing=.65*self.closing+.35*raw

        self.front=d
        self.points=p
        self.last_scan=now
        self.scans+=1

    def send(self,v,k):
        self.v=float(v)
        self.k=float(k)

        m=Twist()
        m.linear.x=self.v
        m.angular.z=self.v*self.k
        self.cmd.publish(m)

        self.speed_pub.publish(Float64(data=self.v))
        self.steer_pub.publish(
            Float64(data=math.atan(.28*self.k)))

    def report(self,now,text):
        if now-self.last_report>=3.:
            self.get_logger().info(text)
            self.last_report=now

    def step(self):
        now=time.monotonic()
        dt=clamp(now-self.last_loop,.02,.20)
        self.last_loop=now

        if now-self.last_gate>1.:
            self.check_gate()
            self.last_gate=now

        if not self.started:
            self.send(0.,0.)
            self.report(now,
                f'WAIT START | scan={self.scans} camera={self.images}')
            return

        if (self.points is None
                or now-self.last_scan>.6
                or now-self.last_odom>.8):
            self.send(0.,0.)
            self.report(now,'SAFE STOP: SENSOR NOT READY')
            return

        v,k,free=plan(
            self.points,self.k,
            self.closing,self.max_speed)

        v=min(v,self.v+.65*dt)
        self.send(v,k if v>0 else 0.)

        state='DRIVE' if v>0 else 'BLOCKED'
        self.report(now,
            f'{state} speed={v:.2f} '
            f'clearance={free:.2f} '
            f'scan={self.scans} camera={self.images}')

def main():
    rclpy.init()
    node=Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send(0.,0.)
            node.destroy_node()
            rclpy.shutdown()

if __name__=='__main__':
    main()
