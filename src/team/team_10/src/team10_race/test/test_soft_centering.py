import math

import pytest

from team10_race.centerline import CenterlineEstimate, ScanBinDiagnostic
from team10_race.geometry import Point2
from team10_race.relaxed_controller import RelaxedPurePursuitController
from team10_race.soft_centering import (
    CorridorMeasurement,
    SoftCenteringPlanner,
    measure_corridor,
)
from team10_race.wall_tracker import WallObservation


def straight(offset: float, stop: float = 12.0):
    return tuple(Point2(0.2 + 0.2 * index, offset) for index in range(int(stop / 0.2)))


def corridor(
    error: float,
    left_clearance: float = 0.90,
    right_clearance: float = 0.90,
    state: str = "NORMAL",
) -> CorridorMeasurement:
    return CorridorMeasurement(
        left_wall_y=left_clearance + 0.116,
        right_wall_y=-(right_clearance + 0.116),
        left_clearance=left_clearance,
        right_clearance=right_clearance,
        lower_center_bound_y=error - 0.50,
        upper_center_bound_y=error + 0.50,
        corridor_center_y=error,
        vehicle_lateral_error=error,
        wall_safety_state=state,
        has_two_walls=True,
    )


def planner() -> SoftCenteringPlanner:
    return SoftCenteringPlanner(
        deadband_enter_m=0.30,
        deadband_exit_m=0.20,
        full_correction_error_m=0.55,
        recenter_length_m=8.0,
        recenter_speed_gain_s=1.0,
        warning_recenter_length_m=3.0,
        critical_recenter_length_m=0.90,
        warning_clearance_m=0.50,
        critical_clearance_m=0.25,
    )


def controller(target_rate: float = 10.0) -> RelaxedPurePursuitController:
    return RelaxedPurePursuitController(
        wheelbase=0.340,
        lookahead=1.40,
        maximum_steering=0.488692,
        cruise_speed=1.00,
        medium_speed=0.85,
        corner_speed=0.65,
        medium_steering=0.14,
        corner_steering=0.32,
        steering_filter_alpha=1.0,
        steering_rate_limit=10.0,
        soft_centering_planner=planner(),
        target_lateral_rate_limit=target_rate,
    )


def test_centered_vehicle_has_negligible_lateral_correction():
    command = controller().compute(straight(0.0), corridor(0.0), 0.10, 1.0)
    assert command is not None
    assert command.center_deadband_state == "CENTER"
    assert abs(command.target.y) < 1e-6
    assert abs(command.lateral_steering_angle) < 1e-6


def test_point_15_offset_stays_inside_deadband_without_recentering():
    command = controller().compute(straight(0.15), corridor(0.15), 0.10, 1.0)
    assert command is not None
    assert command.center_deadband_state == "CENTER"
    assert command.original_target.y == pytest.approx(0.15)
    assert abs(command.target.y) < 1e-6
    assert abs(command.raw_steering_angle) < 1e-6


def test_point_40_offset_uses_long_gradual_recenter_not_exact_center_target():
    command = controller().compute(straight(0.40), corridor(0.40), 0.10, 1.0)
    assert command is not None
    assert command.center_deadband_state == "CORRECT"
    assert command.recenter_length == pytest.approx(9.0)
    assert 0.0 < command.target.y < 0.08
    assert command.target.y < 0.2 * command.original_target.y


def test_close_left_wall_overrides_with_rightward_safety_correction():
    command = controller().compute(
        straight(-0.65),
        corridor(-0.65, left_clearance=0.20, right_clearance=1.50, state="CRITICAL"),
        0.10,
        1.0,
    )
    assert command is not None
    assert command.wall_safety_state == "CRITICAL"
    assert command.target.y < 0.0
    assert command.raw_steering_angle < 0.0  # internal -Y/right


def test_close_right_wall_overrides_with_leftward_safety_correction():
    command = controller().compute(
        straight(0.65),
        corridor(0.65, left_clearance=1.50, right_clearance=0.20, state="CRITICAL"),
        0.10,
        1.0,
    )
    assert command is not None
    assert command.wall_safety_state == "CRITICAL"
    assert command.target.y > 0.0
    assert command.raw_steering_angle > 0.0  # internal +Y/left


def test_corner_heading_remains_when_lateral_error_is_in_deadband():
    curve = tuple(
        Point2(x, 0.15 + 0.035 * x * x)
        for x in (0.2 + 0.2 * index for index in range(60))
    )
    command = controller().compute(curve, corridor(0.15), 0.10, 1.0)
    assert command is not None
    assert command.center_deadband_state == "CENTER"
    assert command.heading_steering_angle > 0.01
    assert abs(command.lateral_steering_angle) < 1e-6
    assert command.raw_steering_angle == pytest.approx(command.heading_steering_angle)


def test_deadband_hysteresis_does_not_chatter_at_boundary():
    soft = planner()
    states = [
        soft.build(straight(value), corridor(value), 1.0).center_deadband_state
        for value in (0.31, 0.29, 0.25, 0.21, 0.19, 0.22, 0.29)
    ]
    assert states == ["CORRECT", "CORRECT", "CORRECT", "CORRECT", "CENTER", "CENTER", "CENTER"]


def test_target_noise_and_rate_limit_prevent_immediate_lateral_sign_flip():
    control = controller(target_rate=0.10)
    first = control.compute(straight(0.80), corridor(0.80), 0.10, 1.0)
    second = control.compute(straight(-0.80), corridor(-0.80), 0.10, 1.0)
    assert first is not None and second is not None
    assert first.target.y > 0.0
    assert second.target.y >= -1e-9
    assert abs(second.target.y - first.target.y) <= 0.011


def test_corridor_bounds_use_current_lidar_walls_and_vehicle_width():
    bins = tuple(
        ScanBinDiagnostic(x, 0.70 + 0.05 * x, -1.50 + 0.05 * x, 2.20, 4, 4, True)
        for x in (0.4, 0.8, 1.2, 1.6)
    )
    scan = CenterlineEstimate((), "SCAN_MIDPOINT", 2.20, bins=bins)
    result = measure_corridor(
        WallObservation(None, None, 2.20), scan, 0.116, 0.35, 0.50, 0.25
    )
    assert result.left_wall_y == pytest.approx(0.70)
    assert result.right_wall_y == pytest.approx(-1.50)
    assert result.upper_center_bound_y == pytest.approx(0.234)
    assert result.lower_center_bound_y == pytest.approx(-1.034)
    assert result.corridor_center_y == pytest.approx(-0.40)
    assert result.wall_safety_state == "NORMAL"

def test_observed_arc_is_not_translated_into_an_obstacle():
    path=tuple(Point2(.2+.2*i,-.4-.35*(.2+.2*i)) for i in range(20))
    measured=planner().build(path,corridor(0.0),1.0,preserve_observed_origin=True)
    assert measured.points==path
    assert measured.recenter_offset==0.0
    legacy=planner().build(path,corridor(0.0),1.0)
    assert legacy.points[3].y>path[3].y
    command=controller().compute(path,corridor(0.0),.1,1.0,follow_observed_path=True)
    assert command.target.y==pytest.approx(command.original_target.y)


def test_observed_arc_still_applies_critical_wall_avoidance():
    from dataclasses import replace
    path=straight(0.0)
    close=replace(corridor(0.0,right_clearance=.10,state='CRITICAL'),
                  left_wall_y=None,left_clearance=math.inf,upper_center_bound_y=None,has_two_walls=False)
    plan=planner().build(path,close,1.0,preserve_observed_origin=True)
    assert plan.wall_safety_state=='CRITICAL'
    assert plan.points[3].y>path[3].y


def test_recorded_critical_bend_keeps_forward_target_and_clearance_correction():
    import json
    from pathlib import Path
    row=json.loads((Path(__file__).parent/'data/critical_arc_target.json').read_text())
    path=tuple(Point2(**p) for p in row['path'])
    measured=CorridorMeasurement(**row['corridor'])
    plan=planner().build(path,measured,1.0,preserve_observed_origin=True)
    command=controller().compute(path,measured,.1,1.0,follow_observed_path=True)
    assert command is not None
    assert command.wall_safety_state=='CRITICAL'
    assert command.target.x>.05 and command.raw_steering_angle>0
    assert plan.points[3].y>path[3].y
    assert max(a.distance(b) for a,b in zip(path,plan.points))<=.5-measured.right_clearance+1e-9


def test_observed_bend_near_wall_correction_decays_along_clear_arc():
    from dataclasses import replace
    path=straight(0.0)
    close=replace(corridor(0.0,right_clearance=.10,state='CRITICAL'),
                  left_wall_y=None,left_clearance=math.inf,upper_center_bound_y=None,has_two_walls=False)
    plan=planner().build(path,close,1.0,preserve_observed_origin=True)
    assert plan.points[0].y>plan.points[4].y>plan.points[12].y>0
    assert plan.wall_safety_state=='CRITICAL'
