import math
from pathlib import Path
import pytest
from team_01_race.ramp_brake import RampBrake,brake_model

MODEL=Path(__file__).parents[1]/'models/team_01_car/model.sdf'

def brake(pitch):
    mass,wheels=brake_model(MODEL)
    b=RampBrake(mass,.04,wheels)
    for i in range(30):
        q=(0.,math.sin(math.radians(pitch)/2),0.,math.cos(math.radians(pitch)/2))
        b.update(q,i*.01,i*.01)
    return b

@pytest.mark.parametrize('pitch',[-12.,-8.,8.,12.])
def test_stopped_slope_torque_cancels_gravity_in_both_directions(pitch):
    b=brake(pitch);v=b.command(.29,stop_requested=True)
    assert v['active'] and not v['saturated']
    assert v['required_torque_nm_per_wheel']*pitch<0
    # At zero wheel speed the real P-only torque controller produces the
    # commanded gravity compensation, with no ideal velocity lock.
    torque=sum(bias*gain for bias,(gain,_) in zip(v['velocity_bias_radps'],b.wheels))
    assert torque/b.radius+b.mass*9.81*math.sin(math.radians(pitch))==pytest.approx(0.,abs=1e-9)


def test_zero_actual_speed_does_not_brake_a_positive_drive_request():
    b=brake(8.)
    assert not b.command(.29,stop_requested=False)['active']
    assert b.command(.30,stop_requested=True)['active']
    v=b.command(.31,stop_requested=False)
    assert not v['active'] and v['velocity_bias_radps']==[0.,0.]


def test_stale_imu_retains_existing_hold_but_does_not_invent_a_new_hold():
    b=brake(8.);first=b.command(.29,stop_requested=True)
    stale=b.command(2.,stop_requested=True)
    assert stale['holding_last_pitch'] and stale['commanded_torque_nm_per_wheel']==first['commanded_torque_nm_per_wheel']
    b.command(2.,stop_requested=False)
    assert not b.command(2.1,stop_requested=True)['active']


def test_flat_confirmation_releases_hold_and_maximum_torque_is_respected():
    b=brake(40.);v=b.command(.29,stop_requested=True)
    assert v['saturated']
    assert all(abs(t)<=limit for t,(_,limit) in zip(v['commanded_torque_nm_per_wheel'],b.wheels))
    for i in range(30,90):b.update((0.,0.,0.,1.),i*.01,i*.01)
    assert not b.command(.89,stop_requested=True)['active']


def test_invalid_or_replayed_imu_cannot_confirm_slope():
    b=brake(0.)
    for _ in range(10):b.update((0.,.1,0.,.995),.3,.3)
    assert not b.command(.3,stop_requested=True)['active']
