import math

import pytest

from team10_race.geometry import Point2
from team10_race.speed_planner import PreviewSpeedPlanner


def make_planner(**overrides) -> PreviewSpeedPlanner:
    values = dict(
        max_speed=0.90,
        medium_speed=0.55,
        corner_speed=0.28,
        preview_distance=5.50,
        curvature_low=0.05,
        curvature_medium=0.12,
        curvature_high=0.28,
        max_accel=0.50,
        max_decel=0.90,
        braking_margin=0.35,
        lateral_accel_limit=0.06,
        corridor_half_width=0.28,
        emergency_clearance=0.55,
        medium_steering=0.14,
        corner_steering=0.32,
        boost_entry_time=0.20,
        boost_max_steering=0.035,
        boost_max_wall_heading_delta=0.10,
        boost_min_wall_clearance=0.55,
        boost_min_preview_distance=4.0,
        boost_min_retention_preview_distance=2.5,
    )
    values.update(overrides)
    return PreviewSpeedPlanner(**values)


def straight(length: float = 5.0) -> list[Point2]:
    return [Point2(0.5 * index, 0.0) for index in range(1, int(length / 0.5) + 1)]


def arc(radius: float, count: int = 10, step: float = 0.08) -> list[Point2]:
    return [
        Point2(radius * math.sin(step * index), radius * (1.0 - math.cos(step * index)))
        for index in range(1, count + 1)
    ]


def reliable_straight_plan(planner: PreviewSpeedPlanner, dt: float = 1.0):
    return planner.plan(
        straight(),
        [],
        0.0,
        0.0,
        dt,
        centerline_source="SCAN_MIDPOINT",
        wall_heading_delta=0.01,
        minimum_wall_clearance=0.80,
    )


def test_long_confirmed_straight_reaches_boost_speed():
    planner = make_planner()
    plan = reliable_straight_plan(planner)
    plan = reliable_straight_plan(planner)
    assert plan.state == "STRAIGHT_BOOST"
    assert plan.target_speed == pytest.approx(0.90)
    assert plan.final_speed == pytest.approx(0.90)
    assert plan.preview_max_curvature == pytest.approx(0.0)


def test_boost_requires_continuous_hold_and_all_confidence_gates():
    planner = make_planner(boost_entry_time=0.50)
    first = reliable_straight_plan(planner, 0.20)
    second = reliable_straight_plan(planner, 0.20)
    assert first.state == "STRAIGHT_CONFIRM"
    assert second.state == "STRAIGHT_CONFIRM"
    third = reliable_straight_plan(planner, 0.20)
    assert third.state == "STRAIGHT_BOOST"

    blocked = planner.plan(
        straight(), [], 0.0, 0.0, 0.10,
        centerline_source="SCAN_MIDPOINT",
        wall_heading_delta=0.20,
        minimum_wall_clearance=0.80,
    )
    assert blocked.state == "TRANSITION"
    assert not blocked.boost_eligible
    assert blocked.target_speed <= planner.medium_speed


def test_gentle_preview_curve_uses_configured_medium_profile_and_physical_limit():
    planner = make_planner()
    plan = planner.plan(arc(radius=8.0), [], 0.0, 0.0, 2.0)
    assert plan.preview_max_curvature == pytest.approx(0.125, rel=0.03)
    assert 0.50 < plan.target_speed < 0.56
    physical_limit = math.sqrt(planner.lateral_accel_limit / plan.preview_max_curvature)
    assert plan.speed_limit_from_curvature < physical_limit


def test_tight_curve_causes_deceleration_before_curve_entry():
    planner = make_planner()
    planner.reset(0.90)
    path = [Point2(0.25, 0.0), Point2(0.50, 0.0)]
    path.extend(
        Point2(0.50 + point.x, point.y)
        for point in arc(radius=1.5, count=8, step=0.20)
    )
    plan = planner.plan(path, [], 0.0, 0.0, 0.10)
    assert 0.0 <= plan.distance_to_curve < plan.preview_distance
    assert plan.required_braking_distance > 0.0
    assert plan.target_speed < 0.90
    assert plan.final_speed < plan.current_speed


def test_corner_exit_acceleration_is_rate_limited_before_boost_confirmation():
    planner = make_planner(boost_entry_time=0.50)
    planner.reset(0.28)
    plan = reliable_straight_plan(planner, 0.10)
    assert plan.state == "STRAIGHT_CONFIRM"
    assert plan.target_speed == pytest.approx(0.55)
    assert plan.applied_speed_delta == pytest.approx(0.05)
    assert plan.final_speed == pytest.approx(0.33)


def test_large_raw_steering_immediately_exits_boost():
    planner = make_planner()
    reliable_straight_plan(planner)
    planner.reset(0.90)
    plan = planner.plan(
        straight(), [], raw_steering=0.40, filtered_steering=0.01, dt=0.10,
        centerline_source="SCAN_MIDPOINT",
        wall_heading_delta=0.01,
        minimum_wall_clearance=0.80,
    )
    assert plan.speed_limit_from_steering == pytest.approx(0.28)
    assert plan.target_speed == pytest.approx(0.28)
    assert plan.state == "CORNER"
    assert plan.final_speed < 0.90


def test_obstacle_inside_driving_corridor_reduces_speed():
    planner = make_planner()
    planner.reset(0.90)
    plan = planner.plan(straight(), [Point2(0.75, 0.05)], 0.0, 0.0, 0.10)
    assert plan.speed_limit_from_corridor < 0.90
    assert plan.target_speed < 0.90


def test_isolated_side_cone_does_not_reduce_confirmed_straight_speed():
    planner = make_planner()
    plan = planner.plan(
        straight(), [Point2(1.0, 0.75)], 0.0, 0.0, 1.0,
        centerline_source="SCAN_MIDPOINT",
        wall_heading_delta=0.01,
        minimum_wall_clearance=0.80,
    )
    assert plan.speed_limit_from_corridor == pytest.approx(0.90)
    assert plan.target_speed == pytest.approx(0.90)


def test_curvature_is_reported_in_five_meter_distance_bands():
    planner = make_planner(preview_distance=20.0, boost_min_preview_distance=10.0)
    path = [Point2(0.5 * index, 0.0) for index in range(1, 21)]
    path.extend(Point2(10.0 + point.x, point.y) for point in arc(4.0, 12, 0.08))
    plan = planner.plan(path, [], 0.0, 0.0, 0.10)
    assert plan.curvature_bands[0] == pytest.approx(0.0)
    assert max(plan.curvature_bands[1:3]) > planner.curvature_low
    assert plan.state == "BRAKE"


def test_invalid_centerline_returns_zero_for_existing_safety_stop_path():
    planner = make_planner()
    planner.reset(0.50)
    plan = planner.plan([Point2(0.4, 0.0)], [], 0.0, 0.0, 0.10)
    assert not plan.valid
    assert plan.state == "INVALID"
    assert plan.final_speed == 0.0
    assert planner.speed == 0.0
