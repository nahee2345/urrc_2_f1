"""Track-independent preview speed planning from the local LiDAR centerline."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .geometry import Point2, finite_point


@dataclass(frozen=True)
class SpeedPlan:
    current_speed: float
    target_speed: float
    final_speed: float
    preview_distance: float
    preview_max_curvature: float
    distance_to_curve: float
    speed_limit_from_curvature: float
    speed_limit_from_steering: float
    speed_limit_from_corridor: float
    applied_speed_delta: float
    acceleration_command: float
    state: str
    curvature_bands: tuple[float, float, float, float]
    required_braking_distance: float
    wall_heading_delta: float
    straight_hold_time: float
    boost_eligible: bool
    valid: bool = True
    state_ceiling: float = math.inf
    speed_limit_from_visibility: float = math.inf
    speed_limit_from_brake_commit: float = math.inf


class PreviewSpeedPlanner:
    """Choose speed from future curvature, steering demand and free corridor."""

    def __init__(
        self,
        max_speed: float,
        medium_speed: float,
        corner_speed: float,
        preview_distance: float,
        curvature_low: float,
        curvature_medium: float,
        curvature_high: float,
        max_accel: float,
        max_decel: float,
        braking_margin: float,
        lateral_accel_limit: float,
        corridor_half_width: float,
        emergency_clearance: float,
        medium_steering: float,
        corner_steering: float,
        boost_entry_time: float = 0.80,
        boost_max_steering: float = 0.035,
        boost_max_wall_heading_delta: float = 0.10,
        boost_min_wall_clearance: float = 0.55,
        boost_min_preview_distance: float = 10.0,
        boost_min_retention_preview_distance: float = 5.0,
        boost_curvature_exit_persistence: float = 0.10,
        continuous_limits: bool = False,
        reaction_time: float = 0.30,
        wheelbase: float = .340,
    ) -> None:
        if not 0.0 <= corner_speed <= medium_speed <= max_speed:
            raise ValueError("speed limits must satisfy 0 <= corner <= medium <= max")
        if not 0.0 <= curvature_low < curvature_medium < curvature_high:
            raise ValueError("curvature thresholds must be strictly increasing")
        if preview_distance <= 0.0 or max_accel <= 0.0 or max_decel <= 0.0:
            raise ValueError("preview distance and acceleration limits must be positive")
        if lateral_accel_limit <= 0.0:
            raise ValueError("lateral acceleration limit must be positive")
        if corridor_half_width <= 0.0 or emergency_clearance < 0.0:
            raise ValueError("corridor dimensions must be non-negative")
        if not 0.0 < medium_steering < corner_steering:
            raise ValueError("steering thresholds must be positive and increasing")
        if boost_entry_time < 0.0 or boost_max_steering <= 0.0:
            raise ValueError("boost hold time and steering threshold must be valid")
        if boost_max_wall_heading_delta <= 0.0 or boost_min_wall_clearance < 0.0:
            raise ValueError("boost wall gates must be valid")
        if boost_min_preview_distance <= 0.0:
            raise ValueError("boost preview distance must be positive")
        if not 0.0 < boost_min_retention_preview_distance <= boost_min_preview_distance:
            raise ValueError("boost retention preview must not exceed entry preview")
        if boost_curvature_exit_persistence < 0.0:
            raise ValueError("boost curvature exit persistence must be non-negative")

        self.max_speed = max_speed
        self.medium_speed = medium_speed
        self.corner_speed = corner_speed
        self.preview_distance = preview_distance
        self.curvature_low = curvature_low
        self.curvature_medium = curvature_medium
        self.curvature_high = curvature_high
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.braking_margin = max(0.0, braking_margin)
        self.lateral_accel_limit = lateral_accel_limit
        self.corridor_half_width = corridor_half_width
        self.emergency_clearance = emergency_clearance
        self.medium_steering = medium_steering
        self.corner_steering = corner_steering
        self.boost_entry_time = boost_entry_time
        self.boost_max_steering = boost_max_steering
        self.boost_max_wall_heading_delta = boost_max_wall_heading_delta
        self.boost_min_wall_clearance = boost_min_wall_clearance
        self.boost_min_preview_distance = boost_min_preview_distance
        self.boost_min_retention_preview_distance = boost_min_retention_preview_distance
        self.boost_curvature_exit_persistence = boost_curvature_exit_persistence
        self.continuous_limits = continuous_limits
        self.reaction_time = max(0., reaction_time)
        self.wheelbase = wheelbase
        self._speed = 0.0
        self._straight_hold_time = 0.0
        self._boost_active = False
        self._boost_curvature_hold_time = 0.0
        self._visibility_brake_cap = math.inf

    @property
    def speed(self) -> float:
        return self._speed

    def reset(self, speed: float = 0.0) -> None:
        self._speed = max(0.0, min(self.max_speed, speed))
        self._straight_hold_time = 0.0
        self._boost_active = False
        self._boost_curvature_hold_time = 0.0
        self._visibility_brake_cap = math.inf

    @staticmethod
    def _interpolate(value: float, low: float, high: float, first: float, second: float) -> float:
        if high <= low:
            return second
        fraction = max(0.0, min(1.0, (value - low) / (high - low)))
        return first + fraction * (second - first)

    def _speed_for_curvature(self, curvature: float) -> float:
        """Combine configured corner tiers with the lateral-acceleration bound."""
        magnitude = abs(curvature)
        if self.continuous_limits:
            # The physical bound applies to gentle bends too. Preserve the
            # established conservative profile for the sharper corner tiers.
            physical = math.sqrt(self.lateral_accel_limit / max(magnitude, 1e-9))
            if magnitude <= self.curvature_low:
                return min(self.max_speed, physical)
        if magnitude <= self.curvature_low:
            return self.max_speed
        physical_limit = math.sqrt(self.lateral_accel_limit / magnitude)
        if magnitude < self.curvature_medium:
            profile_limit = self._interpolate(
                magnitude,
                self.curvature_low,
                self.curvature_medium,
                self.max_speed,
                self.medium_speed,
            )
        elif magnitude < self.curvature_high:
            profile_limit = self._interpolate(
                magnitude,
                self.curvature_medium,
                self.curvature_high,
                self.medium_speed,
                self.corner_speed,
            )
        else:
            profile_limit = self.corner_speed
        return max(self.corner_speed, min(profile_limit, physical_limit))

    def _speed_for_steering(self, raw: float, filtered: float) -> float:
        magnitude = max(abs(raw), abs(filtered))
        if self.continuous_limits and magnitude<=self.boost_max_steering:
            # Steering demand is curvature tan(delta)/L. A linear reduction
            # from max at exactly zero penalises tiny corrections yet can
            # exceed the lateral bound at moderate angles.
            return min(self.max_speed,math.sqrt(self.lateral_accel_limit*self.wheelbase/
                max(math.tan(magnitude),1e-9)))
        if magnitude <= self.medium_steering:
            profile = self._interpolate(
                magnitude, 0.0, self.medium_steering, self.max_speed, self.medium_speed
            )
            if self.continuous_limits:
                profile=min(profile,math.sqrt(self.lateral_accel_limit*self.wheelbase/max(math.tan(magnitude),1e-9)))
            return profile
        if magnitude < self.corner_steering:
            return self._interpolate(
                magnitude,
                self.medium_steering,
                self.corner_steering,
                self.medium_speed,
                self.corner_speed,
            )
        return self.corner_speed

    def _preview(self, centerline: Sequence[Point2]) -> tuple[list[Point2], list[float]]:
        points: list[Point2] = []
        distances: list[float] = []
        previous = Point2(0.0, 0.0)
        travelled = 0.0
        for point in centerline:
            if not finite_point(point):
                continue
            segment = previous.distance(point)
            if segment < 1e-6:
                previous = point
                continue
            remaining = self.preview_distance - travelled
            if remaining <= 0.0:
                break
            if segment > remaining:
                break
            travelled += segment
            points.append(point)
            distances.append(travelled)
            previous = point
        return points, distances

    @staticmethod
    def _curvature(first: Point2, middle: Point2, last: Point2) -> float:
        first_leg = first.distance(middle)
        second_leg = middle.distance(last)
        chord = first.distance(last)
        denominator = first_leg * second_leg * chord
        if first_leg < 0.15 or second_leg < 0.15 or denominator < 1e-8:
            return 0.0
        cross = abs(
            (middle.x - first.x) * (last.y - first.y)
            - (middle.y - first.y) * (last.x - first.x)
        )
        return 2.0 * cross / denominator

    def _curvature_limit(
        self, points: Sequence[Point2], distances: Sequence[float]
    ) -> tuple[float, float, float, tuple[float, float, float, float], float]:
        samples: list[tuple[float, float, float]] = []
        for index in range(1, len(points) - 1):
            curvature = self._curvature(points[index - 1], points[index], points[index + 1])
            onset = 0.0 if index == 1 else distances[index - 2]
            samples.append((distances[index], onset, curvature))
        if not samples:
            return self.max_speed, 0.0, math.inf, (0.0, 0.0, 0.0, 0.0), self.max_speed

        maximum = max(curvature for _, _, curvature in samples)
        distance_to_curve = min(
            (onset for _, onset, value in samples if value > self.curvature_low),
            default=math.inf,
        )
        bands = tuple(
            max(
                (value for distance, _, value in samples if start <= distance < stop),
                default=0.0,
            )
            for start, stop in ((0.0, 5.0), (5.0, 10.0), (10.0, 15.0), (15.0, 20.01))
        )

        limit = self.max_speed
        next_curve_speed = self.max_speed
        for _, onset, curvature in samples:
            curve_speed = self._speed_for_curvature(curvature)
            usable_distance = max(0.0, onset - self.braking_margin)
            braking_limit = math.sqrt(
                max(0.0, curve_speed * curve_speed + 2.0 * self.max_decel * usable_distance)
            )
            limit = min(limit, curve_speed if usable_distance <= 0.0 else braking_limit)
            if curvature > self.curvature_low:
                next_curve_speed = min(next_curve_speed, curve_speed)
        return (
            max(0.0, min(self.max_speed, limit)),
            maximum,
            distance_to_curve,
            bands,
            next_curve_speed,
        )

    @staticmethod
    def _projection_on_segment(
        point: Point2, first: Point2, second: Point2
    ) -> tuple[float, float]:
        dx, dy = second.x - first.x, second.y - first.y
        length_squared = dx * dx + dy * dy
        if length_squared < 1e-12:
            return 0.0, point.distance(first)
        ratio = ((point.x - first.x) * dx + (point.y - first.y) * dy) / length_squared
        ratio = max(0.0, min(1.0, ratio))
        projection = Point2(first.x + ratio * dx, first.y + ratio * dy)
        return ratio, point.distance(projection)

    def _corridor_limit(
        self,
        path: Sequence[Point2],
        path_distances: Sequence[float],
        obstacles: Sequence[Point2],
    ) -> float:
        if not path:
            return 0.0
        vertices = [Point2(0.0, 0.0), *path]
        cumulative = [0.0, *path_distances]
        closest = math.inf
        for obstacle in obstacles:
            if not finite_point(obstacle):
                continue
            for index, (first, second) in enumerate(zip(vertices, vertices[1:])):
                ratio, lateral = self._projection_on_segment(obstacle, first, second)
                if lateral > self.corridor_half_width:
                    continue
                longitudinal = cumulative[index] + ratio * first.distance(second)
                if 0.05 < longitudinal <= self.preview_distance:
                    closest = min(closest, longitudinal)
        if not math.isfinite(closest):
            return self.max_speed
        stopping_distance = max(0.0, closest - self.emergency_clearance)
        return min(self.max_speed, math.sqrt(2.0 * self.max_decel * stopping_distance))

    def plan(
        self,
        centerline: Sequence[Point2],
        obstacles: Sequence[Point2],
        raw_steering: float,
        filtered_steering: float,
        dt: float,
        *,
        centerline_source: str = "UNKNOWN",
        wall_heading_delta: float = math.inf,
        minimum_wall_clearance: float = 0.0,
        swept_free_distance_m: float | None = None,
    ) -> SpeedPlan:
        current = self._speed
        points, distances = self._preview(centerline)
        if len(points) < 2 or not math.isfinite(dt) or dt <= 0.0:
            self.reset()
            return SpeedPlan(
                current_speed=current,
                target_speed=0.0,
                final_speed=0.0,
                preview_distance=distances[-1] if distances else 0.0,
                preview_max_curvature=0.0,
                distance_to_curve=math.inf,
                speed_limit_from_curvature=0.0,
                speed_limit_from_steering=0.0,
                speed_limit_from_corridor=0.0,
                applied_speed_delta=-current,
                acceleration_command=-current / dt if math.isfinite(dt) and dt > 0.0 else 0.0,
                state="INVALID",
                curvature_bands=(0.0, 0.0, 0.0, 0.0),
                required_braking_distance=0.0,
                wall_heading_delta=wall_heading_delta,
                straight_hold_time=0.0,
                boost_eligible=False,
                valid=False,
            )

        (
            curvature_limit,
            maximum_curvature,
            distance_to_curve,
            curvature_bands,
            next_curve_speed,
        ) = self._curvature_limit(points, distances)
        steering_limit = self._speed_for_steering(raw_steering, filtered_steering)
        if swept_free_distance_m is None:
            corridor_limit = self._corridor_limit(points, distances, obstacles)
        else:
            # A current-scan kinematic rollout checks the complete F1 envelope.
            # Keep the same emergency reserve and braking bound, including zero.
            free = swept_free_distance_m if math.isfinite(swept_free_distance_m) else 0.0
            corridor_limit = min(self.max_speed, math.sqrt(2.0*self.max_decel*
                max(0.0,free-self.emergency_clearance)))

        required_braking_distance = 0.0
        if next_curve_speed < current:
            required_braking_distance = (
                (current * current - next_curve_speed * next_curve_speed)
                / (2.0 * self.max_decel)
            )

        steering_magnitude = max(abs(raw_steering), abs(filtered_steering))
        source_reliable = centerline_source in {"SCAN_MIDPOINT", "SCAN_PLUS_WALL", "POLY_BOTH"}
        curvature_clear = maximum_curvature <= self.curvature_low
        common_boost_safe = (
            steering_magnitude <= self.boost_max_steering
            and math.isfinite(wall_heading_delta)
            and wall_heading_delta <= self.boost_max_wall_heading_delta
            and minimum_wall_clearance >= self.boost_min_wall_clearance
            and source_reliable
        )
        boost_eligible = (
            curvature_clear
            and common_boost_safe
            and distances[-1] >= self.boost_min_preview_distance
        )
        if self._boost_active:
            retention_geometry_safe = (
                common_boost_safe
                and distances[-1] >= self.boost_min_retention_preview_distance
            )
            braking_window = required_braking_distance + self.braking_margin + 0.25
            immediate_curve = (
                maximum_curvature >= self.curvature_medium
                or distance_to_curve <= braking_window
            )
            if not retention_geometry_safe or immediate_curve:
                self._straight_hold_time = 0.0
                self._boost_active = False
                self._boost_curvature_hold_time = 0.0
            elif curvature_clear:
                self._boost_curvature_hold_time = 0.0
            else:
                self._boost_curvature_hold_time += dt
                if (
                    self._boost_curvature_hold_time + 1e-9
                    >= self.boost_curvature_exit_persistence
                ):
                    self._straight_hold_time = 0.0
                    self._boost_active = False
                    self._boost_curvature_hold_time = 0.0
        elif boost_eligible:
            self._straight_hold_time += dt
            if self._straight_hold_time >= self.boost_entry_time:
                self._boost_active = True
        else:
            self._straight_hold_time = 0.0

        if self._boost_active:
            state = "STRAIGHT_BOOST"
            state_ceiling = self.max_speed
        elif curvature_bands[0] >= self.curvature_medium or steering_magnitude >= self.medium_steering:
            state = "CORNER"
            state_ceiling = self.medium_speed
        elif math.isfinite(distance_to_curve):
            state = "BRAKE"
            state_ceiling = self.medium_speed
        elif boost_eligible:
            state = "STRAIGHT_CONFIRM"
            state_ceiling = self.medium_speed
        else:
            state = "TRANSITION"
            state_ceiling = self.medium_speed

        visibility_limit = self.max_speed
        if self.continuous_limits:
            # Boost is a diagnostic confidence state, not an independent
            # low-speed ceiling duplicating curvature/steering/braking checks.
            state_ceiling = self.max_speed
            free = distances[-1] if swept_free_distance_m is None else min(distances[-1], swept_free_distance_m)
            budget = max(0., free-self.emergency_clearance)
            reaction = self.max_decel*self.reaction_time
            visibility_limit = min(self.max_speed, math.sqrt(reaction*reaction+2*self.max_decel*budget)-reaction)
        if self.continuous_limits:
            # A flickering long/short horizon must not reverse necessary braking
            # before reaching the previously observed stopping-speed bound.
            if current<=self._visibility_brake_cap+1e-6:
                self._visibility_brake_cap=math.inf
            if visibility_limit<current-1e-6:
                self._visibility_brake_cap=min(self._visibility_brake_cap,visibility_limit)
        target = min(curvature_limit, steering_limit, corridor_limit, state_ceiling,
                     visibility_limit,self._visibility_brake_cap)
        requested_delta = target - current
        if requested_delta >= 0.0:
            applied_delta = min(requested_delta, self.max_accel * dt)
        else:
            applied_delta = max(requested_delta, -self.max_decel * dt)
        self._speed = max(0.0, min(self.max_speed, current + applied_delta))
        return SpeedPlan(
            current_speed=current,
            target_speed=target,
            final_speed=self._speed,
            preview_distance=distances[-1],
            preview_max_curvature=maximum_curvature,
            distance_to_curve=distance_to_curve,
            speed_limit_from_curvature=curvature_limit,
            speed_limit_from_steering=steering_limit,
            speed_limit_from_corridor=corridor_limit,
            applied_speed_delta=applied_delta,
            acceleration_command=applied_delta / dt,
            state=state,
            curvature_bands=curvature_bands,
            required_braking_distance=required_braking_distance,
            wall_heading_delta=wall_heading_delta,
            straight_hold_time=self._straight_hold_time,
            boost_eligible=boost_eligible,
            state_ceiling=state_ceiling,
            speed_limit_from_visibility=visibility_limit,
            speed_limit_from_brake_commit=self._visibility_brake_cap,
        )
