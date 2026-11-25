"""Pure Pursuit steering and curvature-dependent low-speed control."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .geometry import Point2, finite_point


@dataclass(frozen=True)
class ControlCommand:
    speed: float
    yaw_rate: float
    steering_angle: float
    target: Point2
    raw_steering_angle: float = 0.0


def actuator_twist(command: ControlCommand, command_sign: float) -> tuple[float, float]:
    """Map logical forward/yaw-rate commands to the existing vehicle interface.

    The current vehicle's wheel joint axes make negative plugin wheel velocity
    move the chassis in its +X (nose-forward) direction. Both Twist components
    must use the same sign so ``linear/angular`` preserves turning radius.
    """

    sign = -1.0 if command_sign < 0.0 else 1.0
    return sign * command.speed, sign * command.yaw_rate


class PurePursuitController:
    def __init__(
        self,
        wheelbase: float,
        lookahead: float,
        maximum_steering: float,
        cruise_speed: float,
        medium_speed: float,
        corner_speed: float,
        medium_steering: float,
        corner_steering: float,
        steering_filter_alpha: float,
        steering_rate_limit: float,
    ) -> None:
        self.wheelbase = wheelbase
        self.lookahead = lookahead
        self.maximum_steering = min(maximum_steering, math.radians(28.0))
        self.cruise_speed = cruise_speed
        self.medium_speed = medium_speed
        self.corner_speed = corner_speed
        self.medium_steering = medium_steering
        self.corner_steering = corner_steering
        self.steering_filter_alpha = steering_filter_alpha
        self.steering_rate_limit = steering_rate_limit
        self._steering = 0.0

    def reset(self) -> None:
        self._steering = 0.0

    def _target(self, centerline: Sequence[Point2]) -> Point2 | None:
        # Preserve the boundary-chain order.  Choosing a Euclidean-nearest
        # point can jump to a different part of a tight curve even when that
        # point happens to lie on the lookahead circle.
        forward = [point for point in centerline if point.x > 0.05 and finite_point(point)]
        if not forward:
            return None

        previous = Point2(0.0, 0.0)
        travelled = 0.0
        for point in forward:
            segment = previous.distance(point)
            if segment < 1e-9:
                previous = point
                continue
            if travelled + segment >= self.lookahead > 0.0:
                ratio = (self.lookahead - travelled) / segment
                return Point2(
                    previous.x + ratio * (point.x - previous.x),
                    previous.y + ratio * (point.y - previous.y),
                )
            travelled += segment
            previous = point
        return forward[-1]

    def _speed_for_steering(self, steering: float) -> float:
        magnitude = abs(steering)
        if magnitude <= self.medium_steering:
            if self.medium_steering <= 0.0:
                return self.medium_speed
            fraction = magnitude / self.medium_steering
            return self.cruise_speed + fraction * (self.medium_speed - self.cruise_speed)
        if magnitude >= self.corner_steering:
            return self.corner_speed
        fraction = ((magnitude - self.medium_steering) /
                    (self.corner_steering - self.medium_steering))
        return self.medium_speed + fraction * (self.corner_speed - self.medium_speed)

    def compute(self, centerline: Sequence[Point2], dt: float) -> ControlCommand | None:
        target = self._target(centerline)
        if target is None or not math.isfinite(dt) or dt <= 0.0:
            return None
        distance_squared = target.x * target.x + target.y * target.y
        if distance_squared < 1e-8:
            return None
        raw = math.atan(2.0 * self.wheelbase * target.y / distance_squared)
        raw = max(-self.maximum_steering, min(self.maximum_steering, raw))

        alpha = max(0.0, min(1.0, self.steering_filter_alpha))
        filtered = self._steering + alpha * (raw - self._steering)
        maximum_step = max(0.0, self.steering_rate_limit) * dt
        filtered = max(self._steering - maximum_step, min(self._steering + maximum_step, filtered))
        self._steering = max(-self.maximum_steering, min(self.maximum_steering, filtered))

        # An abrupt target change can leave the rate-limited steering close to
        # zero while the raw request already points hard into the next bend.
        # Do not interpret that actuator lag as a straight and accelerate; use
        # the more conservative of the requested and applied steering angles.
        speed = self._speed_for_steering(
            raw if abs(raw) > abs(self._steering) else self._steering
        )
        yaw_rate = speed * math.tan(self._steering) / self.wheelbase
        values = (speed, yaw_rate, self._steering, target.x, target.y)
        if not all(math.isfinite(value) for value in values):
            return None
        return ControlCommand(speed, yaw_rate, self._steering, target, raw)
