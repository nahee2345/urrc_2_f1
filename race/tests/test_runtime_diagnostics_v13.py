"""Fast ROS-free tests: diagnostics are observational, not assertions of camera success."""
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from runtime_diagnostics_v13 import classify, format_line


def _state(**kwargs):
    d=dict(speed_messages_after_release=20,steering_messages_after_release=20,
           max_abs_speed_command=2.0,max_displacement_m=0.,
           first_crossing=None, laps=0, last_speed_command=2.,
           last_steering_command=0.0)
    d.update(kwargs)
    return d


def test_missing_commands():
    assert '명령 미수신' in classify(_state(speed_messages_after_release=0))


def test_zero_speed_does_not_blame_gazebo():
    assert '속도 명령이 0 유지' in classify(_state(max_abs_speed_command=0))


def test_nonzero_command_with_no_motion():
    assert 'base_link 정지' in classify(_state(max_displacement_m=.02))


def test_motion_without_crossing():
    assert '최초 출발선 통과 없음' in classify(_state(max_displacement_m=1.5))


def test_crossing_without_lap():
    assert '랩 미완료' in classify(_state(max_displacement_m=8.,first_crossing=11.0))


def test_diagnostics_include_team_and_counts():
    line=format_line('team_11',_state(max_displacement_m=3.))
    assert 'team_11' in line and '3.000m' in line and '수신=20' in line
