"""Straight-line capture must be gradual; steering must not overshoot."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.planning import RoadCorridor, SafetyPlanner
from team01_race.reference_path import GradualReference
from team01_race.steering import SteeringLimiter
from test_v7_grid_and_depth_corner import CONFIG


def road(center=.6,curve=0.,half=1.0):
    x=np.arange(.6,10.0,.16)
    y=center+curve*x*x
    c=np.column_stack((x,y))
    return RoadCorridor(c,half,True,math.degrees(math.atan(curve*6.)),
                        np.column_stack((x,y-half,y+half)))


class GradualCaptureV15(unittest.TestCase):
    def test_straight_is_joined_gradually_not_instantly(self):
        g=GradualReference(CONFIG)
        line=g.build(road(center=.6),0.,5.)
        self.assertIsNotNone(line)
        self.assertAlmostEqual(line[0,1],0.,places=7)
        self.assertLess(abs(np.interp(2.,line[:,0],line[:,1])),.20)
        self.assertAlmostEqual(np.interp(8.,line[:,0],line[:,1]),.6,delta=.03)
        # Smooth start tangent and zero capture-curve jump.
        self.assertLess(abs(line[1,1]/line[1,0]),.008)

    def test_faster_car_joins_over_longer_distance(self):
        g=GradualReference(CONFIG)
        g.build(road(center=.45),0.,1.)
        slow=g.last_capture_m
        g.build(road(center=.45),0.,10.)
        self.assertGreater(g.last_capture_m,slow)

    def test_road_curve_is_preserved_after_join(self):
        g=GradualReference(CONFIG)
        c=road(center=0.,curve=.04,half=1.)
        line=g.build(c,0.,4.)
        self.assertAlmostEqual(
            np.interp(8.,line[:,0],line[:,1]),.04*64,delta=.15)

    def test_steering_arrives_without_target_overshoot(self):
        s=SteeringLimiter({**CONFIG,'steer_target_filter_tau_s':.1})
        for target in (3.,-3.,0.,5.):
            history=[math.degrees(s.apply(math.radians(target),.05,4.))
                     for _ in range(70)]
            lo,hi=min(history),max(history)
            if target>0.:
                self.assertLessEqual(hi,target+0.1)
            if target<0.:
                self.assertGreaterEqual(lo,target-.1)
            self.assertAlmostEqual(history[-1],target,delta=.05)

    def test_shallow_straight_capture_with_live_boundary(self):
        cfg={**CONFIG,'disable_first_lap_racing_line':False,
             'hold_safe_lane_mode':False,
             'straight_capture_max_steer_deg':6.}
        p=SafetyPlanner(cfg)
        p.racing_line.phase='CENTER'
        p.racing_line.center_confirmed=True
        p.racing_line.offset_m=0.
        b=road(center=.45,half=.9).bounds
        empty=np.empty((0,2))
        commands=[]
        for _ in range(40):
            m=p.step(empty,empty,5.,.05,True,True,True,b,actual_speed=2.)
            commands.append(math.degrees(m.steering))
        self.assertLessEqual(max(map(abs,commands)),6.1)
        self.assertNotIn('BRAKE',[p.last_mode])
        self.assertEqual(p.planner_type,'adaptive_pursuit')


if __name__=='__main__':
    unittest.main()
