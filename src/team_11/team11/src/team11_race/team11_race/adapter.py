"""Independent race-safety gate and controller-to-Ackermann Twist adapter."""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,Float32
from geometry_msgs.msg import Twist


class Team11Adapter(Node):
    def __init__(self):
        super().__init__("team11_adapter")
        for key,value in (
            ("race_start_topic","/race/start/team11"),
            ("speed_topic","/team11/speed"),
            ("steering_topic","/team11/steering"),
            ("use_sim_time",True),
        ):
            self.declare_parameter(key,value)
        self.operator_gate=False
        self.controller_gate=False
        self.speed=0.
        self.steer=0.
        self.speed_time=0.
        self.steer_time=0.
        self.auth_time=0.
        self.pub=self.create_publisher(Twist,"/team11/cmd_vel",10)
        self.create_subscription(
            Bool,self.get_parameter("race_start_topic").value,
            self.on_operator_gate,10)
        self.create_subscription(
            Bool,"/team11/start_authorized",self.on_controller_gate,10)
        self.create_subscription(
            Float32,self.get_parameter("speed_topic").value,
            self.on_speed,10)
        self.create_subscription(
            Float32,self.get_parameter("steering_topic").value,
            self.on_steer,10)
        self.create_timer(.04,self.tick)

    def on_operator_gate(self,msg):
        self.operator_gate=bool(msg.data)
        if not self.operator_gate:
            self.pub.publish(Twist())
            self.controller_gate=False
            self.speed=self.steer=0.

    def on_controller_gate(self,msg):
        self.controller_gate=bool(msg.data)
        self.auth_time=time.monotonic()
        if not self.controller_gate:self.pub.publish(Twist())

    def on_speed(self,msg):
        self.speed=max(0.,min(15.,float(msg.data))) if math.isfinite(msg.data) else 0.
        self.speed_time=time.monotonic()

    def on_steer(self,msg):
        # URRC contract: positive = right, Gazebo yaw: positive = left.
        self.steer=max(-.52,min(.52,float(msg.data))) if math.isfinite(msg.data) else 0.
        self.steer_time=time.monotonic()

    def tick(self):
        now=time.monotonic()
        fresh=(self.speed_time>0. and self.steer_time>0. and
               0<=now-self.speed_time<=.30 and
               0<=now-self.steer_time<=.30 and
               abs(self.speed_time-self.steer_time)<=.12 and
               self.auth_time>0 and 0<=now-self.auth_time<=.30)
        msg=Twist()
        if fresh and self.operator_gate and self.controller_gate:
            msg.linear.x=self.speed
            msg.angular.z=-self.speed*math.tan(self.steer)/.34
        self.pub.publish(msg)


def main():
    rclpy.init()
    node=Team11Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
