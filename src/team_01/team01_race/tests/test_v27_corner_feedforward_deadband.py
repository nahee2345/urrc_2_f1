"""Steering avoids chasing straight noise yet anticipates true curvature."""
import math
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.stanley_control import stanley_steer

CFG={'straight_deadband_cross_m':.10,
     'straight_deadband_heading_deg':2.,
     'straight_deadband_curvature':.012}


def line(offset=0.,curve=0.):
    x=np.linspace(0.,10.,101)
    return np.column_stack((x,offset+curve*x*x))


def command(path,speed=3.):
    return stanley_steer(path,0.,0.,0.,speed,.34,.52,CFG)


class V27(unittest.TestCase):
    def test_small_straight_center_jitter_ignored(self):
        for y in (-.08,-.03,0.,.04,.09):
            self.assertAlmostEqual(command(line(y)),0.,delta=.00001)

    def test_large_straight_offset_corrects_gradually(self):
        self.assertGreater(command(line(.40)),0.)
        self.assertLess(command(line(-.40)),0.)
        self.assertLess(abs(math.degrees(command(line(.40)))),9.)

    def test_true_curvature_produces_feedforward_not_deadband(self):
        self.assertGreater(command(line(curve=.045)),.015)
        self.assertLess(command(line(curve=-.045)),-.015)

    def test_higher_speed_reduces_cross_track_action(self):
        self.assertLess(command(line(.40),speed=7.),
                        command(line(.40),speed=2.))


if __name__=='__main__':unittest.main()
