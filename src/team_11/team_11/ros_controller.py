"""ROS2 command interface for TEAM 11 planner.

Input: /odom_11 (nav_msgs/Odometry), /start_allowed_11 (Bool),
and individual opponent odometry topics set with opponent_topics parameter.
Output: /cmd_vel_11 (Twist: linear.x speed m/s, angular.z desired steering
angle *in radians*, NOT yaw rate). Connect output to the correct vehicle
actuator adapter only after checking the simulated car's interface.
"""
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from strategy import Planner, Track, Vehicle

def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y),
                      1 - 2*(q.y*q.y + q.z*q.z))

class Team11Controller(Node):
    def __init__(self):
        super().__init__("team11_controller")
        self.declare_parameter("track", "monza")
        self.declare_parameter("routes_root", "")
        self.declare_parameter("opponent_topics", [])
        self.declare_parameter("spawn_grid", 1)
        name=self.get_parameter("track").value
        if name not in ("monza", "spa", "monaco", "cota", "hungaroring", "baku"):
            raise ValueError("Unknown official track")
        root=self.get_parameter("routes_root").value
        if not root:
            root=str(Path(__file__).resolve().parents[2] / "generated_routes")
        self.route_root=Path(root)
        grid=int(self.get_parameter("spawn_grid").value)
        if not 1 <= grid <= 11:
            raise ValueError("spawn_grid must be 1..11")
        self.planner=Planner(Track(self.route_root / name),
                             spawn_lane=1 if grid % 2 else 3)
        self.ego=None
        self.others={}
        self.enabled=False
        self.pub=self.create_publisher(Twist, "cmd_vel_11", 10)
        self.create_subscription(Odometry, "odom_11", self.on_ego, 10)
        self.create_subscription(Bool, "start_allowed_11", self.on_start, 10)
        for topic in self.get_parameter("opponent_topics").value:
            if not topic.endswith("_11"):
                raise ValueError("opponent topic must end in _11")
            self.create_subscription(Odometry, topic,
                lambda msg, source=topic: self.on_other(source,msg),10)
        self.create_timer(.05,self.tick)

    def sim_stamp(self, msg):
        # ROS time from message; running the node with use_sim_time=true is expected.
        return float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9

    def vehicle(self,msg,name):
        p=msg.pose.pose.position
        q=msg.pose.pose.orientation
        speed=msg.twist.twist.linear.x
        stamp=self.sim_stamp(msg)
        return Vehicle(p.x,p.y,yaw_of(q),speed,stamp,name)

    def on_ego(self,msg):
        self.ego=self.vehicle(msg,"ego")

    def on_other(self,name,msg):
        self.others[name]=self.vehicle(msg,name)

    def on_start(self,msg):
        self.enabled=bool(msg.data)

    def tick(self):
        output=Twist()
        now=self.get_clock().now().nanoseconds*1e-9
        # Odometry must be fresh even while start is enabled.
        if self.ego is None or abs(now-self.ego.stamp) > .5:
            self.pub.publish(output)
            return
        others=[o for o in self.others.values() if abs(now-o.stamp)<.6]
        command=self.planner.step(self.ego,others,now,enabled=self.enabled)
        output.linear.x=command.speed
        output.angular.z=command.steer
        self.pub.publish(output)

def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=Team11Controller()
        rclpy.spin(node)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
