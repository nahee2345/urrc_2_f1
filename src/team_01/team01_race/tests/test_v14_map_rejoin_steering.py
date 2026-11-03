"""Pure offline invariants for the new mapping, rejoin and steering layers."""
import sys
from pathlib import Path
import unittest
import math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.track_memory import TrackMemory
from team01_race.steering import SteeringLimiter
from team01_race.rejoin import RejoinPlanner,smooth
from team01_race.planning import RoadCorridor

CFG={'path_half_width_m':.18,'max_center_steer_deg':29.8,
     'max_steer_rate_deg_s':35.,'max_steer_accel_deg_s2':170.,
     'overtake_offset_m':.48}


def road():
    x=np.arange(.5,8.,.25)
    c=np.column_stack((x,np.zeros_like(x)))
    return RoadCorridor(c,1.1,True,0.,
        np.column_stack((x,-1.1*np.ones_like(x),1.1*np.ones_like(x))))


class V14(unittest.TestCase):
    def test_quintic_has_smooth_ends(self):
        self.assertAlmostEqual(smooth(0.),0.)
        self.assertAlmostEqual(smooth(1.),1.)
        h=.001
        self.assertLess(abs((smooth(h)-smooth(0.))/h),.01)
        self.assertLess(abs((smooth(1.)-smooth(1.-h))/h),.01)

    def test_rejoin_does_not_snap_after_pass(self):
        p=RejoinPlanner(CFG)
        obstacle=np.array([[4.,0.],[4.1,.05]])
        offset,side=p.update(road(),obstacle,4.,2.,5.,.05,6.,True)
        self.assertNotEqual(side,0)
        self.assertEqual(p.phase,'SHIFT')
        positions=[offset]
        for _ in range(55):
            offset,_=p.update(road(),obstacle,4.,2.,5.,.05,6.,True)
            positions.append(offset)
        self.assertEqual(p.phase,'PASS')
        self.assertGreater(abs(p.offset),.35)
        for _ in range(60):
            offset,_=p.update(road(),np.empty((0,2)),None,None,5.,.05,6.,True)
            positions.append(offset)
        self.assertEqual(p.phase,'CRUISE')
        self.assertAlmostEqual(offset,0.,places=2)
        self.assertLess(max(abs(a-b) for a,b in zip(positions,positions[1:])),.08)

    def test_stale_rear_refuses_passing(self):
        p=RejoinPlanner(CFG)
        _,side=p.update(road(),np.array([[4.,0.]]),4.,2.,-1.,.05,6.,True)
        self.assertEqual(side,0)

    def test_steering_acceleration_and_rate(self):
        s=SteeringLimiter(CFG)
        points=[s.apply(math.radians(25.),.05,6.) for _ in range(30)]
        rate=math.radians(35.)*.05
        self.assertLessEqual(max(b-a for a,b in zip([0.]+points,points)),rate+1e-6)
        self.assertLessEqual(abs(points[-1]),math.radians(25.1))

    def test_slam_track_requires_geometric_loop(self):
        t=TrackMemory(CFG)
        self.assertFalse(t.try_close((0.,0.,0.)))
        r=road()
        n=145
        for i in range(n+1):
            a=2*math.pi*i/n
            pose=(15*math.cos(a),15*math.sin(a),a+math.pi/2)
            t.observe(pose,r)
        self.assertGreater(t.total_m,65.)
        self.assertTrue(t.try_close((15.,0.,math.pi/2)))
        self.assertTrue(t.complete)
        self.assertIsNotNone(t.target((15.,0.,math.pi/2),4.))

if __name__=='__main__':unittest.main()
