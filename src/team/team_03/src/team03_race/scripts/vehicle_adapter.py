#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class VehicleAdapter(Node):
    def __init__(self):
        super().__init__("vehicle_adapter_03")
        self.target_speed = 0.0
        self.target_steering = 0.0

        self.create_subscription(Float32, "/team03/speed_03", self.speed_callback, 10)
        self.create_subscription(Float32, "/team03/steering_03", self.steer_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/team03/cmd_vel_03", 10)
        self.timer = self.create_timer(0.02, self.publish_cmd) # 50Hz

    def speed_callback(self, msg):
        # 5.0 m/s 제한 적용
        self.target_speed = max(-1.0, min(11.0, float(msg.data))) if math.isfinite(msg.data) else 0.0

    def steer_callback(self, msg):
        self.target_steering = float(msg.data)

    def publish_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.target_speed
        cmd.angular.z = self.target_steering
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = VehicleAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()