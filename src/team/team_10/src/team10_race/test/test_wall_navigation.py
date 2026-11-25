import math

import pytest

from team10_race.centerline import build_centerline, build_scan_centerline
from team10_race.controller import PurePursuitController
from team10_race.geometry import Point2
from team10_race.lidar_processing import ScanPoint
from team10_race.wall_tracker import WallTracker


def segment(function, first_beam):
    points = []
    for index in range(161):
        x = index * 0.04
        y = function(x)
        point = Point2(x, y)
        points.append(ScanPoint(first_beam + index, point.norm, point))
    return points


def tracker():
    return WallTracker(
        minimum_points=8,
        bin_size_m=0.20,
        maximum_residual_m=0.28,
        anchor_x_m=0.30,
        initial_track_width_m=2.20,
        minimum_track_width_m=1.70,
        maximum_track_width_m=2.70,
        width_filter_alpha=0.20,
    )


def controller():
    return PurePursuitController(
        wheelbase=0.340,
        lookahead=1.15,
        maximum_steering=0.488692,
        cruise_speed=0.90,
        medium_speed=0.55,
        corner_speed=0.28,
        medium_steering=0.14,
        corner_steering=0.32,
        steering_filter_alpha=1.0,
        steering_rate_limit=10.0,
    )


def test_binned_scan_centerline_stays_centered_between_straight_walls():
    points = [
        *segment(lambda x: 1.20, 100),
        *segment(lambda x: -1.20, 400),
    ]
    estimate = build_scan_centerline(points, 6.0, 0.25, 2.40)
    assert estimate.source == "SCAN_MIDPOINT"
    assert len(estimate.points) >= 20
    assert max(abs(point.y) for point in estimate.points) < 0.02


@pytest.mark.parametrize("curve_sign", [0.0, 1.0, -1.0])
def test_straight_left_and_right_wall_centerlines(curve_sign):
    center = lambda x: curve_sign * 0.035 * x * x
    observation = tracker().update([
        segment(lambda x: center(x) + 1.10, 100),
        segment(lambda x: center(x) - 1.10, 400),
    ])
    estimate = build_centerline(observation, 6.0, 0.25)
    assert estimate.source == "POLY_BOTH"
    assert len(estimate.points) >= 20
    assert estimate.track_width_m == pytest.approx(2.20, abs=0.03)
    command = controller().compute(estimate.points, 0.10)
    assert command is not None
    if curve_sign == 0.0:
        assert abs(command.steering_angle) < 0.01
    else:
        assert math.copysign(1.0, command.steering_angle) == curve_sign


@pytest.mark.parametrize("visible_side", ["left", "right"])
def test_one_wall_fallback_uses_current_normal_and_recent_width(visible_side):
    center = lambda x: 0.025 * x * x
    segments = [
        segment(lambda x: center(x) + 1.10, 100)
        if visible_side == "left"
        else segment(lambda x: center(x) - 1.10, 400)
    ]
    observation = tracker().update(segments)
    estimate = build_centerline(observation, 6.0, 0.25)
    assert estimate.source == ("POLY_LEFT_FALLBACK" if visible_side == "left" else "POLY_RIGHT_FALLBACK")
    assert len(estimate.points) >= 20
    assert abs(estimate.points[0].y) < 0.12
    assert estimate.points[-1].y > estimate.points[0].y


def test_wide_chicane_opening_still_produces_a_centerline():
    observation = tracker().update([
        segment(lambda x: 2.10, 100),
        segment(lambda x: -2.10, 400),
    ])
    estimate = build_centerline(observation, 6.0, 0.25)
    assert estimate.source == "POLY_BOTH"
    assert len(estimate.points) >= 20
    assert max(abs(point.y) for point in estimate.points) < 0.02


def test_isolated_short_object_is_not_a_wall():
    short = [ScanPoint(index, 1.0, Point2(0.5, 0.9 + index * 0.005)) for index in range(5)]
    observation = tracker().update([short])
    assert observation.left is None
    assert observation.right is None


def test_short_curve_fragments_are_joined_before_fitting():
    def fragment(function, start, stop, first_beam):
        values = []
        steps = 20
        for index in range(steps + 1):
            x = start + (stop - start) * index / steps
            point = Point2(x, function(x))
            values.append(ScanPoint(first_beam + index, point.norm, point))
        return values

    curve = lambda x: 0.04 * x * x
    observation = tracker().update([
        fragment(lambda x: curve(x) + 1.10, -0.10, 0.80, 100),
        fragment(lambda x: curve(x) + 1.10, 1.20, 2.60, 140),
        fragment(lambda x: curve(x) - 1.10, -0.10, 0.80, 400),
        fragment(lambda x: curve(x) - 1.10, 1.20, 2.60, 440),
    ])
    estimate = build_centerline(observation, 6.0, 0.25)
    assert observation.left is not None
    assert observation.right is not None
    assert len(estimate.points) >= 5
