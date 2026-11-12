"""Separate race-gate safety bridge from TEAM11 Float32 to Gazebo Twist."""
from __future__ import annotations
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

WHEELBASE=.34
LIMIT=.52
WATCHDOG=.55

class Team11Adapter(Node):
    def __init__(self):
        super().__init__("team11_adapter_11")
        for name,default in (
            ("race_start_topic","/race/start/team_11"),
            ("speed_topic","/team11/speed_11"),
            ("steering_topic","/team11/steering_11")):
            self.declare_parameter(name,default)
        self.enabled=False
        self.speed=0.
        self.steer=0.
        self.ts=0.
        self.tt=0.
        self.out=self.create_publisher(Twist,"/team11/cmd_vel_11",10)
        self.create_subscription(
            Bool,str(self.get_parameter("race_start_topic").value),self.gate,10)
        self.create_subscription(
            Float32,str(self.get_parameter("speed_topic").value),self.on_speed,10)
        self.create_subscription(
            Float32,str(self.get_parameter("steering_topic").value),self.on_steer,10)
        self.create_timer(.04,self.tick)

    def gate(self,msg):
        self.enabled=bool(msg.data)
        if not self.enabled:
            self.out.publish(Twist())
            self.speed=0.
            self.steer=0.

    def on_speed(self,msg):
        self.speed=max(0.,min(15.,float(msg.data))) if math.isfinite(msg.data) else 0.
        self.ts=time.monotonic()

    def on_steer(self,msg):
        self.steer=max(-LIMIT,min(LIMIT,float(msg.data))) if math.isfinite(msg.data) else 0.
        self.tt=time.monotonic()

    def tick(self):
        now=time.monotonic()
        pair_fresh=self.ts>0 and self.tt>0 and abs(self.ts-self.tt)<=.20
        is_fresh=now-self.ts<=WATCHDOG and now-self.tt<=WATCHDOG
        msg=Twist()
        if self.enabled and pair_fresh and is_fresh:
            msg.linear.x=self.speed
            # Organizer: + steering right; Gazebo yaw +left.
            msg.angular.z=-self.speed*math.tan(self.steer)/WHEELBASE
        self.out.publish(msg)


def main():
    rclpy.init()
    node=Team11Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C can invalidate the ROS context before finally executes.
        if rclpy.ok():
            try:
                node.out.publish(Twist())
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()
