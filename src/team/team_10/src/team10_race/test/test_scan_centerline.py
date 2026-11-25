import math

import pytest

from team10_race.centerline import (
    CenterlineEstimate,
    build_scan_centerline,
    choose_centerline,
)
from team10_race.controller import PurePursuitController
from team10_race.geometry import Point2
from team10_race.lidar_processing import ScanPoint


def raw_point(x: float, y: float, beam: int) -> ScanPoint:
    point = Point2(x, y)
    return ScanPoint(beam, point.norm, point)


def corridor(
    center,
    half_width: float = 1.10,
    left_step: float = 0.04,
    right_step: float = 0.04,
    stop: float = 6.0,
) -> list[ScanPoint]:
    points = []
    beam = 0
    x = 0.20
    while x <= stop + 1e-9:
        points.append(raw_point(x, center(x) + half_width, beam))
        beam += 1
        x += left_step
    x = 0.20
    while x <= stop + 1e-9:
        points.append(raw_point(x, center(x) - half_width, beam))
        beam += 1
        x += right_step
    return points


def estimate(points: list[ScanPoint], width: float = 2.20):
    return build_scan_centerline(points, 6.0, 0.40, width)


def make_controller() -> PurePursuitController:
    return PurePursuitController(
        wheelbase=0.340,
        lookahead=1.40,
        maximum_steering=0.488692,
        cruise_speed=2.50,
        medium_speed=1.50,
        corner_speed=0.70,
        medium_steering=0.14,
        corner_steering=0.32,
        steering_filter_alpha=1.0,
        steering_rate_limit=10.0,
    )


def test_scan_midpoint_straight_corridor_is_centered():
    result = estimate(corridor(lambda _x: 0.0))
    assert result.source == "SCAN_MIDPOINT"
    assert len(result.points) >= 12
    assert result.forward_span_m >= 5.0
    assert max(abs(point.y) for point in result.points) < 0.01


def test_scan_midpoint_handles_unequal_side_density():
    result = estimate(corridor(lambda _x: 0.08, left_step=0.02, right_step=0.08))
    assert len(result.points) >= 12
    assert max(abs(point.y - 0.08) for point in result.points) < 0.02


def test_scan_midpoint_selects_inner_boundaries_with_outer_walls():
    points = corridor(lambda _x: 0.0)
    points.extend(corridor(lambda _x: 0.0, half_width=3.0))
    result = estimate(points)
    assert len(result.points) >= 12
    assert max(abs(point.y) for point in result.points) < 0.02
    accepted = [item for item in result.bins if item.accepted]
    assert accepted
    assert max(item.width_m for item in accepted if item.width_m is not None) < 2.30


def test_scan_midpoint_rejects_missing_side_bins_without_inventing_offsets():
    points = corridor(lambda _x: 0.0)
    points = [
        item for item in points
        if not (2.0 <= item.point.x <= 2.8 and item.point.y < 0.0)
    ]
    result = estimate(points)
    rejected = dict(result.rejection_counts)
    assert rejected.get("missing_right", 0) >= 1
    assert len(result.points) >= 8
    assert max(abs(point.y) for point in result.points) < 0.02


def test_scan_midpoint_accepts_wide_chicane_opening():
    result = estimate(corridor(lambda _x: 0.0, half_width=2.10))
    assert len(result.points) >= 12
    assert max(abs(point.y) for point in result.points) < 0.02
    assert all(
        item.width_m == pytest.approx(4.20, abs=0.02)
        for item in result.bins if item.accepted
    )


@pytest.mark.parametrize("direction", [1.0, -1.0])
def test_scan_midpoint_tracks_gentle_curve(direction):
    center = lambda x: direction * 0.018 * x * x
    result = estimate(corridor(center))
    assert len(result.points) >= 10
    assert math.copysign(1.0, result.points[-1].y) == direction
    command = make_controller().compute(result.points, 0.10)
    assert command is not None
    assert math.copysign(1.0, command.steering_angle) == direction


def test_scan_midpoint_single_inner_outlier_does_not_flip_sign():
    points = corridor(lambda _x: 0.10)
    points.append(raw_point(1.0, 0.30, 9999))
    result = estimate(points)
    near_outlier = min(result.points, key=lambda point: abs(point.x - 1.0))
    assert near_outlier.y > 0.05
    assert min(point.y for point in result.points) > 0.05


def test_insufficient_scan_span_falls_back_to_polynomial():
    short_scan = estimate(corridor(lambda _x: 0.0, stop=1.0))
    polynomial = CenterlineEstimate(
        tuple(Point2(0.2 + index * 0.25, 0.15) for index in range(12)),
        "POLY_BOTH",
        2.20,
        forward_span_m=2.75,
    )
    selected = choose_centerline(short_scan, polynomial, 5, 1.60)
    assert selected is polynomial
    assert selected.source == "POLY_BOTH"


def test_multiple_frames_keep_target_sign_stable():
    controller = make_controller()
    signs = []
    for frame in range(12):
        center_y = 0.08 + 0.005 * math.sin(frame)
        points = corridor(
            lambda _x, offset=center_y: offset,
            left_step=0.02 if frame % 2 else 0.04,
            right_step=0.08 if frame % 2 else 0.04,
        )
        if frame % 3 == 0:
            points.append(raw_point(0.8 + 0.4 * (frame % 5), 0.30, 10000 + frame))
        result = estimate(points)
        selected = choose_centerline(
            result,
            CenterlineEstimate((), "NONE", 2.20),
            5,
            1.60,
        )
        command = controller.compute(selected.points, 0.10)
        assert command is not None
        signs.append(math.copysign(1.0, command.target.y))
    assert signs == [1.0] * len(signs)
