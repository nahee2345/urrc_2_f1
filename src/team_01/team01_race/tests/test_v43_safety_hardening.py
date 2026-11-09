"""Guarded control edge cases; execution reserved for independent QA."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.map_quality import MapQuality
from team01_race.track_memory import TrackMemory
from team01_race.steering_intent import SteeringIntent
from team01_race.rejoin import RejoinPlanner
from team01_race.planning_road import RoadCorridor


class SafetyHardening(unittest.TestCase):
    def test_slam_jump_revokes_loop_closure(self):
        m=MapQuality()
        cov=[0.]*36
        cov[0]=cov[7]=cov[35]=.01
        m.odom(1.,cov,0.,0.)
        m.info(1.,3)
        m.odom(1.1,cov,100.,0.)
        self.assertFalse(m.closed)
        self.assertEqual(m.good,0)

    def test_invalid_steering_never_propagates_nan(self):
        s=SteeringIntent({})
        self.assertEqual(s.update(float('nan'),0.,0.,.05),0.)
        self.assertEqual(s.phase,'INVALID_STEERING_INTENT')

    def test_map_wrong_way_disallowed(self):
        t=TrackMemory({})
        t.complete=True
        t.samples=[(float(i),0.,1.8) for i in range(60)]
        self.assertIsNone(t.target((20.,0.,3.14159),3.))
        self.assertEqual(t.reason,'MAP_DIRECTION_MISMATCH')

    def test_overtake_with_obstacle_holds_offset(self):
        cfg={'path_half_width_m':.18,'overtake_offset_m':.48}
        p=RejoinPlanner(cfg)
        x=np.arange(.5,10.,.2)
        b=np.column_stack((x,-np.ones(len(x)),np.ones(len(x))))
        road=RoadCorridor(np.column_stack((x,x*0)),1.,True,0.,b)
        p.phase='SHIFT';p.side=1;p.pass_offset=.48
        p.offset=.20
        p.update(road,np.array([[2.,.48]]),3.,0.,4.,.05,2.,True)
        self.assertEqual(p.phase,'HOLD')
        self.assertAlmostEqual(p.offset,.20)


if __name__=='__main__':unittest.main()
