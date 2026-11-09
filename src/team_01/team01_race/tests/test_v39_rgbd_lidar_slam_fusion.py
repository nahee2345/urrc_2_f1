"""Measured front/rear LiDAR fill each other's blind regions."""
import math
import pathlib
import sys
import unittest
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.scan_fusion import fused_ranges


def scan(angle,values,step=.1):
    return {'ranges':values,'angle_min':angle,'angle_increment':step,
            'range_min':.1,'range_max':16.}


class ScanTests(unittest.TestCase):
    def test_front_and_rear_see_different_near_obstacles(self):
        front=scan(0.,[2.])
        rear=scan(0.,[2.])
        result=fused_ranges(front,rear)
        self.assertTrue(np.isfinite(result[len(result)//2]))
        self.assertTrue(np.isfinite(result[0]) or np.isfinite(result[-1]))
        self.assertTrue(np.isinf(result[len(result)//4]))

    def test_missing_returns_never_imply_open_road(self):
        scan_empty=scan(0.,[float('nan'),float('inf'),0.])
        result=fused_ranges(scan_empty,None)
        self.assertTrue(np.isinf(result).all())

    def test_return_behind_vehicle_has_correct_range(self):
        result=fused_ranges(None,scan(0.,[2.]))
        self.assertAlmostEqual(min(result),2.208,places=3)


if __name__=='__main__':unittest.main()
