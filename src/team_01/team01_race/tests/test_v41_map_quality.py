"""Graph loop detection plus VO covariance and live curb agreement gate."""
import unittest
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.map_quality import MapQuality,map_target_agrees
from team01_race.planning_road import RoadCorridor


class V41(unittest.TestCase):
    def quality(self):
        q=MapQuality()
        cov=[0.]*36;cov[0]=cov[7]=.01;cov[35]=.03
        for i in range(3):q.odom(10.+i*.05,cov,i*.05,0.)
        return q

    def test_provisional_map_needs_rtab_loop(self):
        q=self.quality()
        self.assertFalse(q.ready(10.12,True,True))
        q.info(10.10,42)
        self.assertTrue(q.ready(10.12,True,True))

    def test_bad_covariance_or_stale_info_blocks_guidance(self):
        q=self.quality();q.info(10.10,42)
        self.assertFalse(q.ready(15.,True,True))
        c=[0.]*36;c[0]=9999.;c[7]=.01;c[35]=.01
        q.odom(10.2,c,0.,0.)
        self.assertFalse(q.ready(10.21,True,True))

    def test_unobserved_or_outside_map_waypoint_is_untrusted(self):
        xs=np.linspace(.5,5.,60)
        centers=np.column_stack((xs,np.zeros(len(xs))))
        bounds=np.column_stack((xs,-.6+np.zeros(len(xs)),.6+np.zeros(len(xs))))
        road=RoadCorridor(centers,.6,True,0.,bounds)
        self.assertTrue(map_target_agrees(road,(2.,0.),.18))
        self.assertFalse(map_target_agrees(road,(2.,.5),.18))
        self.assertFalse(map_target_agrees(road,(7.,0.),.18))


if __name__=='__main__':unittest.main()
