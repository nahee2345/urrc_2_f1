import numpy as np
import pytest
from team09_awd.config import SETTINGS
from team09_awd.road_heading import plan_road
from team09_awd.overtake import Overtake


def scan(right, left, target):
    angle = np.arange(-360,360)*np.pi/360
    dx, dy = np.cos(angle), np.sin(angle)
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = np.where(dy>0, left/dy, right/dy)
        if target:
            front = target/dx
            distance = np.where((front>0)&(abs(front*dy)<.105), np.minimum(distance,front), distance)
    valid = (distance>0)&(distance<40)
    return np.c_[dx[valid]*distance[valid],dy[valid]*distance[valid]]


@pytest.mark.parametrize("right,left,expected", [(-.4,1.6,"LEFT"),(-1.6,.4,"RIGHT"),(-.34,.34,None),(-1.,1.,None)])
def test_pass(right,left,expected):
    state = Overtake()
    for i in range(3):
        p = scan(right,left,None if left==1. else 3.5+.03*i)
        result = plan_road(p,0.,2.,SETTINGS["right_wall"],SETTINGS["road_heading"],2.5,4.,.2,
                           pass_state=state,stamp=i*.05,pose=(.1*i,0.,0.))
        if not i:
            assert not result.route_kind.startswith("OVERTAKE")
    if expected:
        assert result.route_kind=="OVERTAKE_"+expected
    else:
        assert not result.route_kind.startswith("OVERTAKE")
