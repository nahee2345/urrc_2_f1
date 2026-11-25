"""Create a local centerline from raw LiDAR bins or fitted walls."""

from collections import Counter
from dataclasses import dataclass
import math
from typing import Sequence

from .geometry import Point2, median
from .lidar_processing import ScanPoint
from .wall_tracker import WallObservation


@dataclass(frozen=True)
class ScanBinDiagnostic:
    x: float
    left_y: float | None
    right_y: float | None
    width_m: float | None
    left_count: int
    right_count: int
    accepted: bool
    rejection_reason: str = ""


@dataclass(frozen=True)
class CenterlineEstimate:
    points: tuple[Point2, ...]
    source: str
    track_width_m: float
    bins: tuple[ScanBinDiagnostic, ...] = ()
    rejection_counts: tuple[tuple[str, int], ...] = ()
    forward_span_m: float = 0.0
    raw_point_count: int = 0


def _offset_from_wall(x: float, y: float, slope: float, distance: float, side: str) -> Point2:
    norm = math.sqrt(1.0 + slope * slope)
    if side == "left":
        # Right-hand normal of the left wall.
        return Point2(x + distance * slope / norm, y - distance / norm)
    # Left-hand normal of the right wall.
    return Point2(x - distance * slope / norm, y + distance / norm)


def _inner_boundary(
    points: Sequence[Point2], minimum_points: int, inner_fraction: float
) -> tuple[float, float] | None:
    """Return a robust representative of the points nearest the corridor."""

    if len(points) < minimum_points:
        return None
    ordered = sorted(points, key=lambda point: abs(point.y))
    count = max(minimum_points, int(math.ceil(len(ordered) * inner_fraction)))
    inner = ordered[:min(len(ordered), count)]
    return median(point.x for point in inner), median(point.y for point in inner)


def _angle_difference(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


def build_scan_centerline(
    scan_points: Sequence[ScanPoint],
    preview_distance_m: float,
    bin_width_m: float,
    track_width_m: float,
    minimum_points_per_side: int = 2,
    inner_fraction: float = 0.50,
    minimum_width_factor: float = 0.55,
    maximum_width_factor: float = 2.25,
    maximum_lateral_jump_m: float = 0.55,
    maximum_heading_change_rad: float = 0.85,
    maximum_curvature_inv_m: float = 1.50,
) -> CenterlineEstimate:
    """Estimate corridor midpoints directly from robust forward-x bins.

    Only bins containing both boundaries may contribute a midpoint. Missing
    sides are deliberately left to the fitted-wall fallbacks; inventing a
    per-bin midpoint from one side is prone to target sign flips at openings.
    """

    step = max(0.15, bin_width_m)
    minimum_points_per_side = max(1, minimum_points_per_side)
    inner_fraction = max(0.20, min(1.0, inner_fraction))
    buckets: dict[int, list[Point2]] = {}
    eligible_count = 0
    for item in scan_points:
        point = item.point
        if not 0.20 <= point.x <= preview_distance_m:
            continue
        if not 0.25 <= abs(point.y) <= 3.50:
            continue
        index = math.floor((point.x - 0.20) / step)
        buckets.setdefault(index, []).append(point)
        eligible_count += 1

    centers: list[Point2] = []
    diagnostics: list[ScanBinDiagnostic] = []
    rejected: Counter[str] = Counter()
    for index in sorted(buckets):
        values = buckets[index]
        left_points = [point for point in values if point.y > 0.25]
        right_points = [point for point in values if point.y < -0.25]
        left = _inner_boundary(left_points, minimum_points_per_side, inner_fraction)
        right = _inner_boundary(right_points, minimum_points_per_side, inner_fraction)
        bin_x = 0.20 + (index + 0.5) * step
        left_y = left[1] if left is not None else None
        right_y = right[1] if right is not None else None
        width = left_y - right_y if left_y is not None and right_y is not None else None
        reason = ""
        candidate: Point2 | None = None

        if left is None and right is None:
            reason = "missing_both"
        elif left is None:
            reason = "missing_left"
        elif right is None:
            reason = "missing_right"
        elif width is None or not math.isfinite(width):
            reason = "invalid_width"
        elif width < minimum_width_factor * track_width_m:
            reason = "width_narrow"
        elif width > maximum_width_factor * track_width_m:
            reason = "width_wide"
        else:
            candidate = Point2(0.5 * (left[0] + right[0]), 0.5 * (left_y + right_y))
            if centers:
                dx = candidate.x - centers[-1].x
                lateral_limit = maximum_lateral_jump_m + math.tan(
                    maximum_heading_change_rad
                ) * max(0.0, dx)
                if dx <= 0.02:
                    reason = "non_forward"
                elif abs(candidate.y - centers[-1].y) > lateral_limit:
                    reason = "lateral_jump"
                elif len(centers) >= 2:
                    previous_heading = math.atan2(
                        centers[-1].y - centers[-2].y,
                        centers[-1].x - centers[-2].x,
                    )
                    heading = math.atan2(
                        candidate.y - centers[-1].y,
                        candidate.x - centers[-1].x,
                    )
                    heading_change = abs(_angle_difference(heading, previous_heading))
                    arc_step = max(0.05, centers[-1].distance(candidate))
                    if heading_change > maximum_heading_change_rad:
                        reason = "heading_jump"
                    elif heading_change / arc_step > maximum_curvature_inv_m:
                        reason = "curvature_jump"

        accepted = candidate is not None and not reason
        if accepted:
            centers.append(candidate)
        else:
            rejected[reason or "invalid"] += 1
        diagnostics.append(ScanBinDiagnostic(
            x=bin_x,
            left_y=left_y,
            right_y=right_y,
            width_m=width,
            left_count=len(left_points),
            right_count=len(right_points),
            accepted=accepted,
            rejection_reason=reason,
        ))

    if len(centers) >= 3:
        centers = [
            Point2(
                point.x,
                median(item.y for item in centers[max(0, position - 1):position + 2]),
            )
            for position, point in enumerate(centers)
        ]
    span = centers[-1].x - centers[0].x if len(centers) >= 2 else 0.0
    return CenterlineEstimate(
        tuple(centers),
        "SCAN_MIDPOINT",
        track_width_m,
        tuple(diagnostics),
        tuple(sorted(rejected.items())),
        span,
        eligible_count,
    )


def choose_centerline(
    scan: CenterlineEstimate,
    polynomial: CenterlineEstimate,
    minimum_scan_points: int,
    minimum_scan_span_m: float,
) -> CenterlineEstimate:
    """Select scan midpoints first, then the fitted two/one-wall estimate."""

    if (
        len(scan.points) >= minimum_scan_points
        and scan.forward_span_m >= minimum_scan_span_m
        and scan.points[0].x <= 1.50
    ):
        return scan
    return polynomial


def build_centerline(
    observation: WallObservation,
    preview_distance_m: float,
    sample_step_m: float,
) -> CenterlineEstimate:
    left, right = observation.left, observation.right
    if left is None and right is None:
        return CenterlineEstimate((), "NONE", observation.track_width_m)

    start = 0.20
    stop = preview_distance_m
    if left is not None:
        start = max(start, left.x_min)
        stop = min(stop, left.x_max)
    if right is not None:
        start = max(start, right.x_min)
        stop = min(stop, right.x_max)
    # With one wall, use all of the current fitted wall. No stale wall geometry
    # is retained; only the slowly filtered scalar track width is remembered.
    if left is None and right is not None:
        start, stop = max(0.20, right.x_min), min(preview_distance_m, right.x_max)
    elif right is None and left is not None:
        start, stop = max(0.20, left.x_min), min(preview_distance_m, left.x_max)

    points = []
    x = start
    while x <= stop + 1e-9:
        if left is not None and right is not None:
            gap = left.y(x) - right.y(x)
            if 0.65 * observation.track_width_m <= gap <= 2.25 * observation.track_width_m:
                points.append(Point2(x, 0.5 * (left.y(x) + right.y(x))))
        elif left is not None:
            points.append(_offset_from_wall(
                x, left.y(x), left.slope(x), 0.5 * observation.track_width_m, "left"
            ))
        elif right is not None:
            points.append(_offset_from_wall(
                x, right.y(x), right.slope(x), 0.5 * observation.track_width_m, "right"
            ))
        x += max(0.10, sample_step_m)

    ordered = tuple(sorted((point for point in points if point.x > 0.05), key=lambda p: p.x))
    source = "POLY_BOTH" if left is not None and right is not None else (
        "POLY_LEFT_FALLBACK" if left is not None else "POLY_RIGHT_FALLBACK"
    )
    span = ordered[-1].x - ordered[0].x if len(ordered) >= 2 else 0.0
    return CenterlineEstimate(ordered, source, observation.track_width_m, forward_span_m=span)
