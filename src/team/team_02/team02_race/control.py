"""TEAM02 signal-gate-only start: no camera input or timeout fallback.

The race operator releases /race/start/team_02 only after five lights go out.
This bridge never starts the Gazebo direct-pose controller before that signal.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class Start(Node):
    def __init__(self):
        super().__init__("team02_signal_start_02")
        self.declare_parameter("race_start_topic", "/race/start/team_02")
        # use_sim_time is owned by the ROS 2 time source; do not redeclare it.
        self.gate = False
        self.reported = False
        self.publisher = self.create_publisher(Bool, "/team02/replay_go_02", 10)
        self.create_subscription(
            Bool,
            str(self.get_parameter("race_start_topic").value),
            self.on_gate, 10)
        self.create_timer(0.05, self.tick)
        self.publisher.publish(Bool(data=False))

    def on_gate(self, msg):
        self.gate = bool(msg.data)
        self.publisher.publish(Bool(data=self.gate))
        if not self.gate:
            self.reported = False
        elif not self.reported:
            self.reported = True
            self.get_logger().info(
                "TEAM02 START: official signal gate released after lights-out")

    def tick(self):
        self.publisher.publish(Bool(data=self.gate))


def main():
    rclpy.init()
    node = Start()
    try:
        rclpy.spin(node)
    finally:
        node.publisher.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
