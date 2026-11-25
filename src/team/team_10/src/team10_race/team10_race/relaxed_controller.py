"""Pure-pursuit controller over a relaxed, wall-safe local trajectory."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .geometry import Point2, finite_point
from .soft_centering import CorridorMeasurement, SoftCenteringPlanner


@dataclass(frozen=True)
class RelaxedControlCommand:
    speed: float
    yaw_rate: float
    steering_angle: float
    target: Point2
    raw_steering_angle: float
    original_target: Point2
    soft_target_y: float
    heading_steering_angle: float
    lateral_steering_angle: float
    center_deadband_state: str
    recenter_offset: float
    recenter_length: float
    wall_safety_state: str
    soft_path: tuple[Point2, ...]


class RelaxedPurePursuitController:
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
        soft_centering_planner: SoftCenteringPlanner,
        target_lateral_rate_limit: float,
        lookahead_speed_gain: float = 0.,
        maximum_lookahead: float = 2.6,
    ) -> None:
        self.wheelbase = wheelbase
        self.lookahead = lookahead
        self.lookahead_speed_gain = max(0.,lookahead_speed_gain)
        self.maximum_lookahead = max(lookahead,maximum_lookahead)
        self.current_lookahead = lookahead
        self.maximum_steering = min(maximum_steering, math.radians(28.0))
        self.cruise_speed = cruise_speed
        self.medium_speed = medium_speed
        self.corner_speed = corner_speed
        self.medium_steering = medium_steering
        self.corner_steering = corner_steering
        self.steering_filter_alpha = steering_filter_alpha
        self.steering_rate_limit = steering_rate_limit
        self.soft_centering_planner = soft_centering_planner
        self.target_lateral_rate_limit = max(0.0, target_lateral_rate_limit)
        self._steering = 0.0
        self._target_lateral_correction = 0.0

    def reset(self) -> None:
        self._steering = 0.0
        self._target_lateral_correction = 0.0
        self.soft_centering_planner.reset()

    def _target(self, path: Sequence[Point2]) -> Point2 | None:
        forward = [point for point in path if point.x > 0.05 and finite_point(point)]
        if not forward:
            return None
        previous = Point2(0.0, 0.0)
        travelled = 0.0
        for point in forward:
            segment = previous.distance(point)
            if segment < 1e-9:
                previous = point
                continue
            if travelled + segment >= self.current_lookahead > 0.0:
                ratio = (self.current_lookahead - travelled) / segment
                return Point2(
                    previous.x + ratio * (point.x - previous.x),
                    previous.y + ratio * (point.y - previous.y),
                )
            travelled += segment
            previous = point
        return forward[-1]

    def _steering_for_target(self, target: Point2) -> float | None:
        distance_squared = target.x * target.x + target.y * target.y
        if distance_squared < 1e-8:
            return None
        value = math.atan(2.0 * self.wheelbase * target.y / distance_squared)
        return max(-self.maximum_steering, min(self.maximum_steering, value))

    def _speed_for_steering(self, steering: float) -> float:
        magnitude = abs(steering)
        if magnitude <= self.medium_steering:
            fraction = magnitude / self.medium_steering if self.medium_steering > 0.0 else 1.0
            return self.cruise_speed + fraction * (self.medium_speed - self.cruise_speed)
        if magnitude >= self.corner_steering:
            return self.corner_speed
        fraction = (
            (magnitude - self.medium_steering)
            / (self.corner_steering - self.medium_steering)
        )
        return self.medium_speed + fraction * (self.corner_speed - self.medium_speed)

    def compute(
        self,
        centerline: Sequence[Point2],
        corridor: CorridorMeasurement,
        dt: float,
        speed_mps: float,
        follow_observed_path: bool = False,
    ) -> RelaxedControlCommand | None:
        if not math.isfinite(dt) or dt <= 0.0:
            return None
        self.current_lookahead=min(self.maximum_lookahead,self.lookahead+
            self.lookahead_speed_gain*max(0.,speed_mps-3.))
        original_target = self._target(centerline)
        plan = self.soft_centering_planner.build(
            centerline, corridor, speed_mps, preserve_observed_origin=follow_observed_path)
        heading_target = self._target(plan.heading_points)
        desired_target = self._target(plan.points)
        if original_target is None or heading_target is None or desired_target is None:
            return None

        desired_correction = desired_target.y - heading_target.y
        if plan.wall_safety_state == "CRITICAL":
            limited_correction = desired_correction
        else:
            maximum_step = self.target_lateral_rate_limit * dt
            limited_correction = max(
                self._target_lateral_correction - maximum_step,
                min(self._target_lateral_correction + maximum_step, desired_correction),
            )
        self._target_lateral_correction = limited_correction
        target = Point2(desired_target.x, heading_target.y + limited_correction)

        raw = self._steering_for_target(target)
        heading_raw = self._steering_for_target(heading_target)
        if raw is None or heading_raw is None:
            return None
        lateral_raw = raw - heading_raw
        alpha = max(0.0, min(1.0, self.steering_filter_alpha))
        filtered = self._steering + alpha * (raw - self._steering)
        maximum_steering_step = max(0.0, self.steering_rate_limit) * dt
        filtered = max(
            self._steering - maximum_steering_step,
            min(self._steering + maximum_steering_step, filtered),
        )
        self._steering = max(-self.maximum_steering, min(self.maximum_steering, filtered))
        speed = self._speed_for_steering(
            raw if abs(raw) > abs(self._steering) else self._steering
        )
        yaw_rate = speed * math.tan(self._steering) / self.wheelbase
        values = (speed, yaw_rate, self._steering, target.x, target.y)
        if not all(math.isfinite(value) for value in values):
            return None
        return RelaxedControlCommand(
            speed=speed,
            yaw_rate=yaw_rate,
            steering_angle=self._steering,
            target=target,
            raw_steering_angle=raw,
            original_target=original_target,
            soft_target_y=desired_target.y,
            heading_steering_angle=heading_raw,
            lateral_steering_angle=lateral_raw,
            center_deadband_state=plan.center_deadband_state,
            recenter_offset=plan.recenter_offset,
            recenter_length=plan.recenter_length,
            wall_safety_state=plan.wall_safety_state,
            soft_path=plan.points,
        )

    def follow_trajectory(self, trajectory, corridor, dt):
        """Track a footprint-checked Ackermann rollout without deforming its geometry.

        Its near-wall safety is the swept complete vehicle envelope, not a y(x)
        intercept applied a second time. Preserve steering filtering and slew limits.
        """
        if not math.isfinite(dt) or dt <= 0 or len(trajectory.centerline.points)<5:
            return None
        self.current_lookahead=self.lookahead
        raw=trajectory.steering
        alpha=max(0.,min(1.,self.steering_filter_alpha))
        change=max(-self.steering_rate_limit*dt,min(self.steering_rate_limit*dt,
                                                 alpha*(raw-self._steering)))
        self._steering=max(-self.maximum_steering,min(self.maximum_steering,self._steering+change))
        self._target_lateral_correction=0.
        path=trajectory.centerline.points;target=self._target(path)
        if target is None:return None
        speed=self._speed_for_steering(raw)
        return RelaxedControlCommand(speed,speed*math.tan(self._steering)/self.wheelbase,
            self._steering,target,raw,target,target.y,raw,0.,'FOOTPRINT',0.,0.,
            corridor.wall_safety_state,path)
