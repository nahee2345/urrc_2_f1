"""Near/middle/far road metrics require actual measured boundary support."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.road_roi import road_roi_metrics
from team01_race.planning_road import RoadCorridor


def road(horizon):
    x=np.arange(.65,horizon,.15)
    y=.04*x*x
    b=np.column_stack((x,y-1.,y+1.))
    return RoadCorridor(np.column_stack((x,y)),1.,True,0.,b)


class V28(unittest.TestCase):
    def test_far_is_unknown_when_camera_cannot_prove_it(self):
        result=road_roi_metrics(road(5.1))
        self.assertTrue(result['near']['valid'])
        self.assertTrue(result['middle']['valid'])
        self.assertFalse(result['far']['valid'])

    def test_far_prelook_available_on_extended_verified_rails(self):
        result=road_roi_metrics(road(10.))
        self.assertTrue(result['far']['valid'])
        self.assertGreater(result['far']['peak_abs_curvature_m_inv'],.03)

    def test_no_previous_frame_can_make_unknown_far_roi_valid(self):
        c=RoadCorridor(np.empty((0,2)),0.,False,0.)
        result=road_roi_metrics(c)
        self.assertFalse(any(item['valid'] for item in result.values()))


if __name__=='__main__':unittest.main()
