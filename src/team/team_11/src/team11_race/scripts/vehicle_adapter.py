#!/usr/bin/env python3
"""Float32 speed and +right steering to Gazebo Twist."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32


class VehicleAdapter(Node):
    def __init__(self):
        super().__init__("team11_vehicle_adapter")

        self.declare_parameter("race_start_topic", "/race/start/team_11")
        self.declare_parameter("race_start_relay_topic", "/team_11/internal/race_start")
        self.declare_parameter("require_race_start", True)
        self.declare_parameter("speed_topic", "/team_11/speed")
        self.declare_parameter("steering_topic", "/team_11/steering")

        self.race_start_topic = str(
            self.get_parameter("race_start_topic").value
        )
        self.require_race_start = bool(
            self.get_parameter("require_race_start").value
        )
        self.race_allowed = not self.require_race_start
        self.start_sub = None
        self._arm_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._arm_timer = None
        self._pub_seen_at = None
        self.speed = 0.0
        self.steering = 0.0

        self.speed_time = 0.0
        self.steer_time = 0.0

        self.wheel_base = 0.32

        # Virtual center angle capped so the inner wheel stays below 30 deg.
        self.limit = math.atan(
            0.32 / (0.32 / math.tan(math.pi / 6) + 0.085)
        )

        self.pub = self.create_publisher(
            Twist,
            "/team_11/cmd_vel",
            10,
        )

        relay_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.relay_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("race_start_relay_topic").value),
            relay_qos,
        )
        self.relay_pub.publish(Bool(data=self.race_allowed))

        self.create_subscription(
            Float32,
            self.get_parameter("speed_topic").value,
            self.on_speed,
            10,
        )

        self.create_subscription(
            Float32,
            self.get_parameter("steering_topic").value,
            self.on_steering,
            10,
        )

        self.create_timer(0.05, self.send)

    def arm_start_subscription(self):
        # FastDDS writer wait_for_all_acked() can miss the first sample if the
        # subscriber announces itself while StartGate SEDP matching is incomplete.
        if self.start_sub is not None or self._arm_timer is not None:
            return
        self._pub_seen_at = None
        # Must use steady wall time so discovery works when use_sim_time:=true before /clock ticks.
        self._arm_timer = self.create_timer(
            0.02,
            self._poll_start_publisher,
            clock=self._arm_clock,
        )

    def _poll_start_publisher(self):
        if not rclpy.ok() or self.start_sub is not None:
            self._cancel_arm_timer()
            return

        if self.count_publishers(self.race_start_topic) <= 0:
            self._pub_seen_at = None
            return

        now = time.monotonic()
        if self._pub_seen_at is None:
            # Wait until StartGate has published its SEDP announcement and settled
            # before creating the subscription so wait_for_all_acked() sees ACKs.
            self._pub_seen_at = now
            return

        if now - self._pub_seen_at < 0.6:
            return

        self._cancel_arm_timer()
        if rclpy.ok() and self.start_sub is None:
            self.start_sub = self.create_subscription(
                Bool,
                self.race_start_topic,
                self.on_start,
                10,
            )

    def _cancel_arm_timer(self):
        if self._arm_timer is not None:
            self._arm_timer.cancel()
            self.destroy_timer(self._arm_timer)
            self._arm_timer = None

    def on_start(self, msg):
        if not self.require_race_start:
            self.race_allowed = True
            self.relay_pub.publish(Bool(data=True))
            return

        self.race_allowed = bool(msg.data)
        self.relay_pub.publish(Bool(data=self.race_allowed))
        if not self.race_allowed:
            self.speed = 0.0
            self.pub.publish(Twist())

    def on_speed(self, msg):
        # 전진 최고속도: 8.0 m/s.
        self.speed = (
            max(-1.0, min(8.0, msg.data))
            if math.isfinite(msg.data)
            else 0.0
        )

        self.speed_time = time.monotonic()
        self.send()

    def on_steering(self, msg):
        self.steering = (
            max(-self.limit, min(self.limit, msg.data))
            if math.isfinite(msg.data)
            else 0.0
        )

        self.steer_time = time.monotonic()
        self.send()

    def send(self):
        msg = Twist()

        now = time.monotonic()

        if (
            self.race_allowed
            and now - self.speed_time < 1.0
            and now - self.steer_time < 1.0
        ):
            msg.linear.x = self.speed

            msg.angular.z = (
                -self.speed
                * math.tan(self.steering)
                / self.wheel_base
            )

        self.pub.publish(msg)


def main():
    rclpy.init()

    node = VehicleAdapter()

    try:
        node.arm_start_subscription()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node.pub.publish(Twist())

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()