"""Test-only ROS to Gazebo Ackermann Twist bridge; fail-closed input watchdog."""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from .stage5_core import convert

class Adapter(Node):
    def __init__(self):
        super().__init__("team11_stage5_adapter")
        self.declare_parameter("test_bypass_gate", False)
        self.bypass=bool(self.get_parameter("test_bypass_gate").value)
        self.gate=False
        self.speed=self.steer=0.
        self.ts=self.tt=-1.e10
        self.pub=self.create_publisher(Twist,"/model/team_11/cmd_vel_11",10)
        self.create_subscription(Float32,"/speed_11",self.on_speed,10)
        self.create_subscription(Float32,"/steering_11",self.on_steer,10)
        self.create_subscription(Bool,"/start_allowed_11",self.on_gate,10)
        self.create_timer(.05,self.tick)
    def on_speed(self,msg):
        self.speed=float(msg.data); self.ts=time.monotonic()
    def on_steer(self,msg):
        self.steer=float(msg.data); self.tt=time.monotonic()
    def on_gate(self,msg):
        self.gate=bool(msg.data)
    def tick(self):
        now=time.monotonic()
        speed,yaw=convert(self.speed,self.steer,
            self.gate or self.bypass,now-self.ts<.4,now-self.tt<.4)
        cmd=Twist();cmd.linear.x=speed;cmd.angular.z=yaw
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    n=Adapter()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
