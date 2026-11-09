"""Optical-ray registration of wider RGB onto depth FoV."""
import math
import pathlib
import sys
import unittest
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.slam_alignment import wide_rgb_to_depth, camera_focal, paired_stamp


class AlignmentTests(unittest.TestCase):
    def test_central_ray_is_unchanged(self):
        image=np.zeros((240,424,3),dtype=np.uint8)
        image[:,211:213,:]=255
        aligned=wide_rgb_to_depth(image,424,240)
        self.assertEqual(aligned.shape,(240,424,3))
        self.assertGreater(aligned[120,211,0],220)

    def test_depth_intrinsics_not_wide_rgb_intrinsics(self):
        self.assertGreater(camera_focal(424,1.6057),camera_focal(424,1.832596))

    def test_synchronization_rejects_stale_pair(self):
        self.assertTrue(paired_stamp(100.0,100.05))
        self.assertFalse(paired_stamp(100.0,100.3))


if __name__=='__main__':unittest.main()
