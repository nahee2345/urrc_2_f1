import math

from team10_race.geometry import Point2
from test_speed_planner import make_planner, reliable_straight_plan


def curve_after_straight(straight_length: float, radius: float) -> list[Point2]:
    path = [
        Point2(0.5 * index, 0.0)
        for index in range(1, int(straight_length / 0.5) + 1)
    ]
    start = path[-1].x
    path.extend(
        Point2(
            start + radius * math.sin(0.08 * index),
            radius * (1.0 - math.cos(0.08 * index)),
        )
        for index in range(1, 7)
    )
    return path


def test_confirmed_boost_survives_shorter_but_braking_safe_preview():
    planner = make_planner(boost_entry_time=0.10)
    entry = reliable_straight_plan(planner, 0.20)
    assert entry.state == "STRAIGHT_BOOST"

    short_straight = [Point2(0.5 * index, 0.0) for index in range(1, 7)]
    retained = planner.plan(
        short_straight,
        [],
        0.0,
        0.0,
        0.10,
        centerline_source="SCAN_PLUS_WALL",
        wall_heading_delta=0.01,
        minimum_wall_clearance=0.80,
    )
    assert not retained.boost_eligible
    assert retained.state == "STRAIGHT_BOOST"

    exited = planner.plan(
        short_straight,
        [],
        0.10,
        0.10,
        0.10,
        centerline_source="SCAN_PLUS_WALL",
        wall_heading_delta=0.01,
        minimum_wall_clearance=0.80,
    )
    assert exited.state != "STRAIGHT_BOOST"


def test_boost_ignores_one_scan_of_weak_far_curvature():
    planner = make_planner(
        boost_entry_time=0.10,
        boost_curvature_exit_persistence=0.10,
    )
    assert reliable_straight_plan(planner, 0.20).state == "STRAIGHT_BOOST"

    spike = planner.plan(
        curve_after_straight(3.0, 12.5), [], 0.0, 0.0, 0.04,
        centerline_source="SCAN_MIDPOINT",
        wall_heading_delta=0.01,
        minimum_wall_clearance=0.80,
    )
    assert planner.curvature_low < spike.preview_max_curvature < planner.curvature_medium
    assert spike.state == "STRAIGHT_BOOST"

    cleared = reliable_straight_plan(planner, 0.04)
    assert cleared.state == "STRAIGHT_BOOST"


def test_persistent_weak_far_curvature_exits_boost_after_point_one_second():
    planner = make_planner(
        boost_entry_time=0.10,
        boost_curvature_exit_persistence=0.10,
    )
    assert reliable_straight_plan(planner, 0.20).state == "STRAIGHT_BOOST"
    path = curve_after_straight(3.0, 12.5)

    assert planner.plan(
        path, [], 0.0, 0.0, 0.04,
        centerline_source="SCAN_MIDPOINT", wall_heading_delta=0.01,
        minimum_wall_clearance=0.80,
    ).state == "STRAIGHT_BOOST"
    assert planner.plan(
        path, [], 0.0, 0.0, 0.04,
        centerline_source="SCAN_MIDPOINT", wall_heading_delta=0.01,
        minimum_wall_clearance=0.80,
    ).state == "STRAIGHT_BOOST"
    assert planner.plan(
        path, [], 0.0, 0.0, 0.04,
        centerline_source="SCAN_MIDPOINT", wall_heading_delta=0.01,
        minimum_wall_clearance=0.80,
    ).state != "STRAIGHT_BOOST"


def test_strong_or_near_curve_still_exits_boost_immediately():
    for path in (
        curve_after_straight(3.0, 4.0),
        curve_after_straight(0.5, 12.5),
    ):
        planner = make_planner(
            boost_entry_time=0.10,
            boost_curvature_exit_persistence=0.10,
        )
        assert reliable_straight_plan(planner, 0.20).state == "STRAIGHT_BOOST"
        plan = planner.plan(
            path, [], 0.0, 0.0, 0.02,
            centerline_source="SCAN_MIDPOINT",
            wall_heading_delta=0.01,
            minimum_wall_clearance=0.80,
        )
        assert plan.state != "STRAIGHT_BOOST"
