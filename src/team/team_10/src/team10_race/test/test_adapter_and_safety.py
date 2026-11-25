import math

import pytest

from team10_race.race_adapter import CommandGate, competition_to_plugin
from team10_race.safety import scan_is_fresh


def test_adapter_signs_match_verified_legacy_axes():
    linear, angular = competition_to_plugin(1.0, 0.20, 0.340)
    assert linear == -1.0
    assert angular > 0.0  # competition +right
    _, left_angular = competition_to_plugin(1.0, -0.20, 0.340)
    assert left_angular < 0.0
    assert abs(angular) == pytest.approx(abs(left_angular))


def test_gate_defaults_to_stop_and_false_stops_immediately():
    gate = CommandGate(2.0, math.radians(28.0), 0.35)
    gate.set_speed(0.9, 1.0)
    gate.set_steering(0.2, 1.0)
    assert gate.output(1.1) == (0.0, 0.0)
    gate.set_started(True)
    assert gate.output(1.1) == pytest.approx((0.9, 0.2))
    gate.set_started(False)
    assert gate.output(1.1) == (0.0, 0.0)


def test_gate_watchdog_and_restart_require_new_commands():
    gate = CommandGate(2.0, math.radians(28.0), 0.35)
    gate.set_started(True)
    gate.set_speed(0.9, 1.0)
    gate.set_steering(0.2, 1.0)
    assert gate.output(1.36) == (0.0, 0.0)
    gate.set_started(False)
    gate.set_started(True)
    assert gate.output(1.2) == (0.0, 0.0)


def test_scan_watchdog():
    assert not scan_is_fresh(None, 1.0, 0.4)
    assert scan_is_fresh(1.0, 1.3, 0.4)
    assert not scan_is_fresh(1.0, 1.5, 0.4)
