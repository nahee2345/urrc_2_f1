"""Small fail-safe predicates used by navigation and tests."""

import math


def scan_is_fresh(last_scan: float | None, now: float, timeout: float) -> bool:
    if last_scan is None:
        return False
    return (
        all(math.isfinite(value) for value in (last_scan, now, timeout))
        and timeout > 0.0
        and 0.0 <= now - last_scan <= timeout
    )


def perception_is_valid(
    segment_count: int,
    wall_count: int,
    centerline_count: int,
    minimum_centerline_points: int,
    primary_scan_valid: bool = False,
) -> bool:
    return (
        (primary_scan_valid or (segment_count > 0 and wall_count > 0))
        and centerline_count >= minimum_centerline_points
    )
