"""Fail-safe conversion from the URRC Float32 contract to Gazebo Ackermann Twist."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32
from .start_interlock import StartInterlock
from .speed_audit import SpeedAudit


def competition_to_plugin(
    speed_mps: float,
    steering_rad: float,
    wheelbase_m: float,
) -> tuple[float, float]:
    """Convert +forward/+right contract to the verified legacy plugin axes.

    The copied wheel joints require negative plugin linear velocity for chassis
    +X motion. The competition steering sign is opposite to conventional yaw,
    and the legacy vehicle requires both conventional Twist components to be
    negated. These two steering sign changes cancel, producing positive
    plugin angular.z for a competition-contract right turn.
    """

    return -speed_mps, speed_mps * math.tan(steering_rad) / wheelbase_m


@dataclass
class CommandGate:
    maximum_speed_mps: float
    maximum_steering_rad: float
    timeout_s: float
    started: bool = False
    speed: float = 0.0
    steering: float = 0.0
    speed_time: float | None = None
    steering_time: float | None = None

    def set_started(self, value: bool) -> None:
        self.started = bool(value)
        if not self.started:
            self.speed = 0.0
            self.steering = 0.0
            self.speed_time = None
            self.steering_time = None

    def set_speed(self, value: float, now: float) -> None:
        if math.isfinite(value):
            self.speed = max(-self.maximum_speed_mps, min(self.maximum_speed_mps, value))
            self.speed_time = now

    def set_steering(self, value: float, now: float) -> None:
        if math.isfinite(value):
            self.steering = max(
                -self.maximum_steering_rad,
                min(self.maximum_steering_rad, value),
            )
            self.steering_time = now

    def output(self, now: float) -> tuple[float, float]:
        if not self.started or self.speed_time is None or self.steering_time is None:
            return 0.0, 0.0
        if now - self.speed_time > self.timeout_s or now - self.steering_time > self.timeout_s:
            return 0.0, 0.0
        return self.speed, self.steering


class RaceAdapter(Node):
    def __init__(self) -> None:
        super().__init__("race_adapter_10")
        self.speed_audit = SpeedAudit('adapter')
        self.requested_speed = 0.
        defaults = {
            "race_start_topic": "/race/start/team_10",
            "speed_topic": "/team_10/speed_10",
            "steering_topic": "/team_10/steering_10",
            "actuator_topic": "/team_10/gz_cmd_vel_10",
            "wheelbase_m": 0.340,
            "maximum_speed_mps": 8.0,
            "maximum_steering_rad": 0.488692,
            "command_timeout_s": 0.35,
            "command_rate_hz": 50.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.wheelbase = float(value("wheelbase_m"))
        self.gate = CommandGate(
            float(value("maximum_speed_mps")),
            float(value("maximum_steering_rad")),
            float(value("command_timeout_s")),
        )
        self.control_callbacks = MutuallyExclusiveCallbackGroup()
        start_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.start_interlock = StartInterlock(
            self, str(value("race_start_topic")), self._start_callback, self.control_callbacks
        )
        self.create_subscription(
            Float32, str(value("speed_topic")), self._speed_callback, 10, callback_group=self.control_callbacks
        )
        self.create_subscription(
            Float32, str(value("steering_topic")), self._steering_callback, 10, callback_group=self.control_callbacks
        )
        self.publisher = self.create_publisher(
            Twist, str(value("actuator_topic")), 10
        )
        rate = max(10.0, float(value("command_rate_hz")))
        self.create_timer(1.0 / rate, self._publish, callback_group=self.control_callbacks)

    def _start_callback(self, message: Bool) -> None:
        self.gate.set_started(message.data)
        if not message.data:
            self._publish()

    def _speed_callback(self, message: Float32) -> None:
        self.requested_speed = float(message.data)
        self.gate.set_speed(float(message.data), time.monotonic())

    def _steering_callback(self, message: Float32) -> None:
        self.gate.set_steering(float(message.data), time.monotonic())

    def _publish(self) -> None:
        speed, steering = self.gate.output(time.monotonic())
        linear, angular = competition_to_plugin(speed, steering, self.wheelbase)
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        now = time.monotonic()
        if now-getattr(self, '_gate_log_time', 0.0)>1.0:
            self.get_logger().info(f'adapter started={self.gate.started} speed={speed:.3f} steer={steering:.3f} twist=({linear:.3f},{angular:.3f}) command_age=({None if self.gate.speed_time is None else now-self.gate.speed_time},{None if self.gate.steering_time is None else now-self.gate.steering_time})')
            self._gate_log_time=now
        self.publisher.publish(message)
        if self.speed_audit.enabled:
            if not self.gate.started:
                reason='STOP_GATE'
            elif self.gate.speed_time is None or self.gate.steering_time is None:
                reason='NO_COMMAND'
            elif max(now-self.gate.speed_time,now-self.gate.steering_time)>self.gate.timeout_s:
                reason='COMMAND_TIMEOUT'
            elif abs(self.requested_speed)>self.gate.maximum_speed_mps:
                reason='SPEED_CLAMP'
            else:
                reason='COMMAND'
            self.speed_audit.emit(event='output', sim=self.get_clock().now().nanoseconds*1e-9,
                requested=self.requested_speed, speed=speed, steering=steering,
                twist=(linear,angular), started=self.gate.started,
                limited_request_mps=self.gate.speed,maximum_speed_mps=self.gate.maximum_speed_mps,
                output_reason=reason,command_timeout_wall_s=self.gate.timeout_s,
                steering_age_wall=None if self.gate.steering_time is None else now-self.gate.steering_time,
                command_age_wall=None if self.gate.speed_time is None else now-self.gate.speed_time)

    def destroy_node(self):
        self.speed_audit.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaceAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
