import math
from team09_awd.kinematics import IndependentActuator,wheel_speeds,WheelImuOdom,euler
from team09_awd.model_builder import validate_dimensions
from team09_awd.synchronization import RGBDPairer,AccelerationGuard
from team09_awd.config import SETTINGS
from team09_awd.core import Result

def test_braking_preserves_steering():
    a=IndependentActuator();a.speed=10.
    speed,angle,wheels=a.step(0.,.15,.05)
    assert speed==9.7 and angle==.15 and len(wheels)==4

def test_reverse_is_signed():
    assert all(v<0 for v in wheel_speeds(-.2,.1))

def test_odometry_reverse():
    o=WheelImuOdom();o.update(0.,-1.,-1.,(0.,0.,0.),0.)
    assert o.update(.1,-1.,-1.,(0.,0.,0.),0.)[0]<0

def test_invalid_attitude():
    assert euler((0,0,0,0)) is None
    assert abs(euler((math.sin(.4),0,0,math.cos(.4)))[0]-.8)<1e-9

def test_model_envelope():
    d=validate_dimensions()
    assert d['full_steer_width_m']<.25
    assert max(abs(x) for x in d['centre_m'])<1e-9

def test_pair_arrival_order():
    p=RGBDPairer();assert p.add('depth',1.,'d') is None
    assert p.add('rgb',1.09,'r')==(1.09,'r',1.,'d')
    assert p.add('rgb',1.09,'r') is None

def test_guard_keeps_straight_acceleration():
    g=AccelerationGuard();r=Result(clearance=.8,wall_quality=1.);r.route_kind='CURVED_LOCAL'
    assert g.apply(8.1,8.,8.,r,1.,SETTINGS['control'])[0]==8.1

def test_guard_holds_closing_side():
    g=AccelerationGuard();r=Result(clearance=.5,wall_quality=1.);r.route_kind='CURVED_LOCAL'
    g.apply(8.,8.,8.,r,1.,SETTINGS['control']);r.clearance=.45
    assert g.apply(8.1,8.,8.,r,1.1,SETTINGS['control'])[0]==8.
