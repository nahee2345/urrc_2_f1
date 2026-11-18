"""TEAM08 camera/gate-only start. Motion is native Gazebo SetWorldPoseCmd."""
from __future__ import annotations
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from team08_race.camera_start import LightTransition, red_evidence

class Start(Node):
    def __init__(self):
        super().__init__("team08_camera_start_08")
        self.declare_parameter("race_start_topic", "/race/start/team_08")
        self.declare_parameter("use_sim_time", True)
        self.gate = False
        self.gate_at = 0.0
        self.frames = 0
        self.transition = LightTransition()
        self.publisher = self.create_publisher(Bool, "/team08/replay_go_08", 10)
        self.create_subscription(
            Bool, str(self.get_parameter("race_start_topic").value), self.on_gate, 10)
        self.create_subscription(
            Image, "/team08/start_image_08", self.on_image, 10)
        self.create_timer(.05, self.tick)
        self.reported = False
    def on_gate(self, msg):
        flag = bool(msg.data)
        if not flag:
            self.gate_at = 0.0
            self.reported = False
        elif not self.gate:
            self.gate_at = time.monotonic()
        self.gate = flag
        if not flag:
            self.publisher.publish(Bool(data=False))
    def on_image(self, msg):
        self.frames += 1
        if self.frames % 2:
            return
        amount = red_evidence(
            msg.data, int(msg.width), int(msg.height),
            int(msg.step), str(msg.encoding))
        self.transition.update(amount)
    def tick(self):
        # Camera red-to-dark is primary. The official gate is authoritative
        # after lights-out, so solo test can start even when the camera missed
        # the red pixels; never move before the organizer releases its gate.
        fallback = self.gate_at > 0 and time.monotonic()-self.gate_at >= .75
        start = self.gate and (self.transition.started or fallback)
        self.publisher.publish(Bool(data=start))
        if start and not self.reported:
            self.reported = True
            reason = "CAMERA_LIGHTS_OUT" if self.transition.started else "OFFICIAL_GATE_SOLO"
            self.get_logger().info("TEAM08 direct-pose START source="+reason)

def main():
    rclpy.init()
    node=Start()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.publisher.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()
