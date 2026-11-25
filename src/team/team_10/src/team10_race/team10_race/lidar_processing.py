"""LaserScan filtering and continuity segmentation for solid walls."""

from dataclasses import dataclass
import math
from typing import Sequence

from .geometry import Point2


@dataclass(frozen=True)
class ScanPoint:
    beam_index: int
    range: float
    point: Point2


def scan_to_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    sensor_range_min: float,
    sensor_range_max: float,
    fov_deg: float,
    configured_range_min: float,
    configured_range_max: float,
) -> list[ScanPoint]:
    if not math.isfinite(angle_min) or not math.isfinite(angle_increment):
        return []
    if angle_increment == 0.0 or not 0.0 < fov_deg <= 360.0:
        return []
    minimum = max(sensor_range_min, configured_range_min)
    maximum = min(sensor_range_max, configured_range_max)
    if not math.isfinite(minimum) or not math.isfinite(maximum) or minimum >= maximum:
        return []

    half_fov = math.radians(fov_deg) * 0.5
    result = []
    for index, distance in enumerate(ranges):
        angle = angle_min + index * angle_increment
        wrapped = math.atan2(math.sin(angle), math.cos(angle))
        if abs(wrapped) > half_fov:
            continue
        if not math.isfinite(distance) or not minimum <= distance <= maximum:
            continue
        point = Point2(distance * math.cos(wrapped), distance * math.sin(wrapped))
        # Reject rear returns and the conservative vehicle self-return box.
        if point.x < -0.20:
            continue
        if -0.18 <= point.x <= 0.30 and abs(point.y) <= 0.18:
            continue
        result.append(ScanPoint(index, distance, point))
    return result


def continuous_segments(
    points: Sequence[ScanPoint],
    maximum_gap_m: float,
    maximum_missing_beams: int,
    minimum_points: int,
) -> list[list[ScanPoint]]:
    """Split ordered beams at geometric or angular discontinuities."""

    if not points:
        return []
    segments: list[list[ScanPoint]] = [[points[0]]]
    for item in points[1:]:
        previous = segments[-1][-1]
        beam_gap = item.beam_index - previous.beam_index
        geometric_gap = item.point.distance(previous.point)
        # Permit a slightly larger chord at long range without merging objects.
        allowed = maximum_gap_m + 0.012 * min(item.range, previous.range)
        if beam_gap <= maximum_missing_beams + 1 and geometric_gap <= allowed:
            segments[-1].append(item)
        else:
            segments.append([item])
    return [segment for segment in segments if len(segment) >= minimum_points]
