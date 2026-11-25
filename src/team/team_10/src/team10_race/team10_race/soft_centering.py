"""Relaxed local-centerline tracking with explicit wall-safety priority.

All geometry in this module is expressed in the LiDAR/vehicle frame.  No
world pose, map, waypoint, or track coordinate is used.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .centerline import CenterlineEstimate
from .geometry import Point2, finite_point
from .wall_tracker import WallObservation


@dataclass(frozen=True)
class CorridorMeasurement:
    left_wall_y: float | None
    right_wall_y: float | None
    left_clearance: float
    right_clearance: float
    lower_center_bound_y: float | None
    upper_center_bound_y: float | None
    corridor_center_y: float
    vehicle_lateral_error: float
    wall_safety_state: str
    has_two_walls: bool


@dataclass(frozen=True)
class SoftPathPlan:
    points: tuple[Point2, ...]
    heading_points: tuple[Point2, ...]
    center_deadband_state: str
    recenter_offset: float
    recenter_length: float
    wall_safety_state: str


def _linear_intercept(samples: Sequence[tuple[float, float]]) -> float | None:
    """Estimate wall y at the vehicle origin from the nearest scan bins."""

    usable = sorted(
        ((x, y) for x, y in samples if math.isfinite(x) and math.isfinite(y)),
        key=lambda item: item[0],
    )[:4]
    if not usable:
        return None
    if len(usable) == 1:
        return usable[0][1]
    mean_x = sum(item[0] for item in usable) / len(usable)
    mean_y = sum(item[1] for item in usable) / len(usable)
    denominator = sum((item[0] - mean_x) ** 2 for item in usable)
    if denominator < 1e-8:
        return mean_y
    slope = sum(
        (item[0] - mean_x) * (item[1] - mean_y) for item in usable
    ) / denominator
    return mean_y - slope * mean_x


def measure_corridor(
    observation: WallObservation,
    scan_centerline: CenterlineEstimate,
    vehicle_half_width_m: float,
    wall_clearance_margin_m: float,
    warning_clearance_m: float,
    critical_clearance_m: float,
) -> CorridorMeasurement:
    """Measure the usable lateral corridor from current LiDAR wall geometry.

    Near scan-bin boundaries are preferred because they are the same raw wall
    geometry used by SCAN_MIDPOINT.  The fitted walls are a fallback when a
    side has no accepted near bin.  Track-width memory is intentionally not
    used to invent a missing boundary.
    """

    accepted = [item for item in scan_centerline.bins if item.accepted and item.x <= 2.0]
    left_y = _linear_intercept([
        (item.x, item.left_y) for item in accepted if item.left_y is not None
    ])
    right_y = _linear_intercept([
        (item.x, item.right_y) for item in accepted if item.right_y is not None
    ])
    if left_y is None and observation.left is not None:
        left_y = observation.left.y(0.0)
    if right_y is None and observation.right is not None:
        right_y = observation.right.y(0.0)

    half_width = max(0.0, vehicle_half_width_m)
    margin = max(0.0, wall_clearance_margin_m)
    left_clearance = left_y - half_width if left_y is not None else math.inf
    right_clearance = -right_y - half_width if right_y is not None else math.inf
    lower = right_y + half_width + margin if right_y is not None else None
    upper = left_y - half_width - margin if left_y is not None else None
    has_two = lower is not None and upper is not None and lower <= upper
    center = 0.5 * (lower + upper) if has_two else 0.0

    known_clearances = [
        value for value in (left_clearance, right_clearance) if math.isfinite(value)
    ]
    closest = min(known_clearances, default=math.inf)
    if closest <= critical_clearance_m:
        safety = "CRITICAL"
    elif closest <= warning_clearance_m:
        safety = "WARNING"
    else:
        safety = "NORMAL"
    return CorridorMeasurement(
        left_wall_y=left_y,
        right_wall_y=right_y,
        left_clearance=left_clearance,
        right_clearance=right_clearance,
        lower_center_bound_y=lower,
        upper_center_bound_y=upper,
        corridor_center_y=center,
        vehicle_lateral_error=center,
        wall_safety_state=safety,
        has_two_walls=has_two,
    )


def _smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _path_origin_y(points: Sequence[Point2]) -> float:
    usable = [point for point in points if point.x > 0.05 and finite_point(point)][:4]
    return _linear_intercept([(point.x, point.y) for point in usable]) or 0.0


class SoftCenteringPlanner:
    """Build a parallel near-field path that only slowly returns to center."""

    def __init__(
        self,
        deadband_enter_m: float,
        deadband_exit_m: float,
        full_correction_error_m: float,
        recenter_length_m: float,
        recenter_speed_gain_s: float,
        warning_recenter_length_m: float,
        critical_recenter_length_m: float,
        warning_clearance_m: float,
        critical_clearance_m: float,
    ) -> None:
        if not 0.0 <= deadband_exit_m < deadband_enter_m:
            raise ValueError("deadband must satisfy 0 <= exit < enter")
        if full_correction_error_m <= deadband_enter_m:
            raise ValueError("full correction error must exceed deadband entry")
        if min(recenter_length_m, warning_recenter_length_m, critical_recenter_length_m) <= 0.0:
            raise ValueError("recenter lengths must be positive")
        if not 0.0 <= critical_clearance_m < warning_clearance_m:
            raise ValueError("wall clearances must satisfy 0 <= critical < warning")
        self.deadband_enter = deadband_enter_m
        self.deadband_exit = deadband_exit_m
        self.full_correction_error = full_correction_error_m
        self.recenter_length = recenter_length_m
        self.recenter_speed_gain = max(0.0, recenter_speed_gain_s)
        self.warning_recenter_length = warning_recenter_length_m
        self.critical_recenter_length = critical_recenter_length_m
        self.warning_clearance = warning_clearance_m
        self.critical_clearance = critical_clearance_m
        self._correcting = False

    def reset(self) -> None:
        self._correcting = False

    def _normal_correction(self, error: float) -> float:
        magnitude = abs(error)
        if not self._correcting:
            if magnitude <= self.deadband_enter:
                return 0.0
            self._correcting = True
        elif magnitude < self.deadband_exit:
            self._correcting = False
            return 0.0

        ramp = _smoothstep(
            (magnitude - self.deadband_exit)
            / (self.deadband_enter - self.deadband_exit)
        )
        return error * ramp

    def _wall_avoidance_error(self, corridor: CorridorMeasurement) -> float:
        if corridor.has_two_walls:
            return corridor.corridor_center_y
        left_shortage = (
            self.warning_clearance - corridor.left_clearance
            if math.isfinite(corridor.left_clearance) else 0.0
        )
        right_shortage = (
            self.warning_clearance - corridor.right_clearance
            if math.isfinite(corridor.right_clearance) else 0.0
        )
        # +Y is vehicle-left: a close left wall demands a rightward (-Y)
        # correction, while a close right wall demands a leftward one.
        return max(0.0, right_shortage) - max(0.0, left_shortage)

    def build(
        self,
        centerline: Sequence[Point2],
        corridor: CorridorMeasurement,
        speed_mps: float,
        preserve_observed_origin: bool = False,
    ) -> SoftPathPlan:
        usable = tuple(point for point in centerline if point.x > 0.05 and finite_point(point))
        if not usable:
            return SoftPathPlan((), (), "CENTER", 0.0, self.recenter_length, corridor.wall_safety_state)

        error = corridor.vehicle_lateral_error
        normal_correction = (
            self._normal_correction(error)
            if corridor.has_two_walls and not preserve_observed_origin else 0.0
        )
        safety = corridor.wall_safety_state
        length = self.recenter_length + self.recenter_speed_gain * max(0.0, speed_mps)
        correction = normal_correction
        if safety in ("WARNING", "CRITICAL"):
            wall_error = self._wall_avoidance_error(corridor)
            if preserve_observed_origin:
                # The raw wall-normal path already follows the measured bend.
                # A y(x) intercept across that bend is not its lateral centre:
                # applying that centre again can push every target behind us.
                # Still correct the full measured near-wall clearance deficit.
                left = (max(0.0, self.warning_clearance - corridor.left_clearance)
                        if math.isfinite(corridor.left_clearance) else 0.0)
                right = (max(0.0, self.warning_clearance - corridor.right_clearance)
                         if math.isfinite(corridor.right_clearance) else 0.0)
                wall_error = right - left
            closest = min(corridor.left_clearance, corridor.right_clearance)
            risk = _smoothstep(
                (self.warning_clearance - closest)
                / max(1e-6, self.warning_clearance - self.critical_clearance)
            )
            warning_length = self.warning_recenter_length
            length = warning_length + risk * (self.critical_recenter_length - warning_length)
            correction = normal_correction + risk * (wall_error - normal_correction)
            if safety == "CRITICAL":
                correction = wall_error
                length = self.critical_recenter_length

        # A directly observed wall-normal path was already checked against
        # obstacles. Translating it through the vehicle origin can cut a bend.
        # Keep all wall-warning/critical corrections and their limits above.
        origin_offset = 0.0 if preserve_observed_origin else -_path_origin_y(usable)
        soft_points: list[Point2] = []
        heading_points: list[Point2] = []
        previous = Point2(0.0, 0.0)
        travelled = 0.0
        for index, point in enumerate(usable):
            travelled += previous.distance(point)
            previous = point
            before = usable[index - 1] if index else Point2(0.0, _path_origin_y(usable))
            after = usable[index + 1] if index + 1 < len(usable) else point
            dx, dy = after.x - before.x, after.y - before.y
            norm = math.hypot(dx, dy)
            normal_x, normal_y = (0.0, 1.0) if norm < 1e-8 else (-dy / norm, dx / norm)
            heading_points.append(Point2(
                point.x + normal_x * origin_offset,
                point.y + normal_y * origin_offset,
            ))
            decay = math.exp(-travelled / max(0.1, length))
            # On an observed bend the origin clearance describes the near wall,
            # not a parallel wall continuing through the far curve. Resolve that
            # deficit near the vehicle and return to the obstacle-checked arc.
            lateral_return = correction * (decay if preserve_observed_origin else 1.0 - decay)
            offset = origin_offset + lateral_return
            soft_points.append(Point2(
                point.x + normal_x * offset,
                point.y + normal_y * offset,
            ))

        state = "CORRECT" if self._correcting else "CENTER"
        return SoftPathPlan(
            tuple(soft_points),
            tuple(heading_points),
            state,
            origin_offset,
            length,
            safety,
        )
