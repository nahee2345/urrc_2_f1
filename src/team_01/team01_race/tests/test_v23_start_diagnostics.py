"""Dark-camera readiness must not bypass the independent driving sensor gate."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.start_diagnostics import camera_ready


class V23(unittest.TestCase):
    def test_dark_baseline_required(self):
        self.assertTrue(camera_ready(True,0.,None,.15,True,True))
        self.assertFalse(camera_ready(False,0.,None,.15,True,True))
        self.assertFalse(camera_ready(True,None,None,.15,True,True))
        self.assertFalse(camera_ready(True,0.,None,1.7,True,True))
        # Lights may run before Depth/LiDAR are ready, but the vehicle may
        # NOT accelerate until SafetyPlanner receives both live sensors.
        self.assertTrue(camera_ready(True,0.,None,.15,False,True))
        self.assertTrue(camera_ready(True,0.,None,.15,True,False))

    def test_once_lights_begin_readiness_is_closed(self):
        self.assertFalse(camera_ready(True,0.,13.2,.1,True,True))


if __name__=='__main__':unittest.main()
