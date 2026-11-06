"""Hard independent race gate + command timeout: Float32 -> Gazebo Twist."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist


class Adapter(Node):
    def __init__(self):
        super().__init__('team01_adapter')
        for k,v in [('race_start_topic','/race/start/team01'),
                    ('speed_topic','/team01/speed'),('steering_topic','/team01/steering'),
                    ('command_topic','/team01/cmd_vel'),('use_sim_time',True)]:
            if not self.has_parameter(k):self.declare_parameter(k,v)
        self.allowed=False
        self.race_allowed=False
        self.fallback_allowed=False
        self.speed=0.
        self.steer=0.
        self.ts=0.
        self.tt=0.
        self.create_subscription(Bool,self.get_parameter('race_start_topic').value,
                                 self.on_gate,10)
        self.create_subscription(Bool,'/team01/start_fallback',self.on_fallback,10)
        self.create_subscription(Float32,self.get_parameter('speed_topic').value,
                                 self.on_speed,10)
        self.create_subscription(Float32,self.get_parameter('steering_topic').value,
                                 self.on_steer,10)
        self.pub=self.create_publisher(Twist,self.get_parameter('command_topic').value,10)
        self.pub_debug=self.create_publisher(String,'/team01/debug/adapter',10)
        self.create_timer(.04,self.tick)

    def on_gate(self,m):
        self.race_allowed=bool(m.data)
        self.allowed=self.race_allowed or self.fallback_allowed
        if not self.allowed:
            self.speed=self.steer=0.
            self.pub.publish(Twist())

    def on_fallback(self,m):
        self.fallback_allowed=bool(m.data)
        self.allowed=self.race_allowed or self.fallback_allowed
        if not self.allowed:
            self.speed=self.steer=0.
            self.pub.publish(Twist())

    def on_speed(self,m):
        self.speed=max(0.,min(12.,float(m.data))) if math.isfinite(m.data) else 0.
        self.ts=time.monotonic()

    def on_steer(self,m):
        # Race rule steering: +RIGHT, -LEFT. Max centre steer deliberately
        # below 35 degrees because Ackermann INNER tyre can be sharper.
        self.steer=max(-.52,min(.52,float(m.data))) if math.isfinite(m.data) else 0.
        self.tt=time.monotonic()

    def tick(self):
        now=time.monotonic()
        safe=(self.allowed and now-self.ts<.3 and now-self.tt<.3)
        m=Twist()
        if safe:
            m.linear.x=self.speed
            m.angular.z=-self.speed*math.tan(self.steer)/.34
        self.pub.publish(m)
        # ROS command-side evidence: distinguishes camera gate from missing
        # adapter permission, command timeout and Gazebo drivetrain issues.
        self.pub_debug.publish(String(data=json.dumps({
            'allowed':self.allowed,'race_allowed':self.race_allowed,
            'fallback_allowed':self.fallback_allowed,
            'fresh_speed':now-self.ts<.3,
            'fresh_steer':now-self.tt<.3,'send_enabled':safe,
            'received_speed_mps':round(self.speed,3),
            'received_steer_rad':round(self.steer,4),
            'sent_linear_mps':round(float(m.linear.x),3),
            'sent_yaw_rad_s':round(float(m.angular.z),4),
        },separators=(',',':'))))


def main():
    rclpy.init()
    n=Adapter()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.pub.publish(Twist())
        n.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
