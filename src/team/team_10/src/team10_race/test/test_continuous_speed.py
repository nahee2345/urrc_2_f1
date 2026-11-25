import math
import pytest
from test_speed_planner import make_planner, straight


def test_unconfirmed_straight_is_limited_by_observed_stopping_distance():
    p=make_planner(max_speed=10.,medium_speed=1.5,corner_speed=1.,max_decel=3.,
        max_accel=1.5,lateral_accel_limit=2.5,continuous_limits=True)
    r=p.plan(straight(5),[],0.,0.,.1,centerline_source='SCAN_MIDPOINT')
    assert r.state != 'STRAIGHT_BOOST'
    assert 1.5 < r.target_speed < 5.
    assert r.target_speed==pytest.approx(r.speed_limit_from_visibility)
    assert r.target_speed**2/6 + .3*r.target_speed + .55 <= 5.+1e-8


def test_gentle_curvature_obeys_lateral_bound_at_high_speed():
    p=make_planner(max_speed=10.,continuous_limits=True,lateral_accel_limit=2.5)
    assert p._speed_for_curvature(.04)==pytest.approx(math.sqrt(2.5/.04))


def test_removing_state_ceiling_does_not_remove_emergency_stop():
    p=make_planner(continuous_limits=True)
    r=p.plan(straight(),[],0,0,.1,swept_free_distance_m=0.)
    assert r.target_speed==0.
    assert r.speed_limit_from_corridor==0.


def test_sharp_corner_tiers_are_preserved():
    p=make_planner(continuous_limits=True)
    assert p._speed_for_curvature(.4)==pytest.approx(p.corner_speed)
    assert p._speed_for_steering(.4,.01)==pytest.approx(p.corner_speed)


def test_small_steering_is_bounded_by_lateral_acceleration():
    p=make_planner(max_speed=10.,continuous_limits=True,lateral_accel_limit=2.5)
    assert p._speed_for_steering(.004,.004)==10.
    v=p._speed_for_steering(.02,.01)
    assert v*v*math.tan(.02)/.340 <= 2.5+1e-8


def test_short_horizon_braking_finishes_before_long_horizon_reacceleration():
    p=make_planner(max_speed=10.1,continuous_limits=True,reaction_time=.4,
                   max_decel=3.,max_accel=3.)
    p.reset(8.)
    short=p.plan(straight(22),[],0.,0.,.1,swept_free_distance_m=3.44)
    cap=short.speed_limit_from_visibility
    assert short.final_speed==pytest.approx(7.7)
    previous=short.final_speed
    while p.speed>cap+1e-6:
        r=p.plan(straight(22),[],0.,0.,.1,swept_free_distance_m=21.92)
        assert r.final_speed<=previous
        assert r.target_speed<=cap+1e-6
        previous=r.final_speed
    resumed=p.plan(straight(22),[],0.,0.,.1,swept_free_distance_m=21.92)
    assert resumed.final_speed>previous
    assert math.isinf(resumed.speed_limit_from_brake_commit)
    p.reset()
    assert math.isinf(p._visibility_brake_cap)
