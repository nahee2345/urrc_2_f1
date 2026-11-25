import pytest
from test_soft_centering import controller,corridor,straight


def test_low_speed_lookahead_keeps_the_proven_corner_controller():
    base=controller();adaptive=controller();adaptive.lookahead_speed_gain=.2
    a=base.compute(straight(.4),corridor(.4),.1,2.)
    b=adaptive.compute(straight(.4),corridor(.4),.1,2.)
    assert a==b
    assert adaptive.current_lookahead==1.4


def test_high_speed_target_is_bounded_by_observed_path_and_lookahead():
    c=controller();c.lookahead_speed_gain=.2
    c.compute(straight(0),corridor(0),.1,8.)
    assert c.current_lookahead==pytest.approx(2.4)
    c.compute(straight(0),corridor(0),.1,20.)
    assert c.current_lookahead==2.6
    cmd=c.compute(straight(0,stop=1.2),corridor(0),.1,8.)
    assert cmd.target.x<=1.2
