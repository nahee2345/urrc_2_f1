"""Start readiness must require darkness baseline and healthy sensors."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.start_diagnostics import camera_ready


class V23(unittest.TestCase):
    def test_dark_baseline_and_good_sensors_required(self):
        self.assertTrue(camera_ready(True,0.,None,.15,True,True))
        self.assertFalse(camera_ready(False,0.,None,.15,True,True))
        self.assertFalse(camera_ready(True,None,None,.15,True,True))
        self.assertFalse(camera_ready(True,0.,None,1.7,True,True))
        self.assertFalse(camera_ready(True,0.,None,.15,False,True))
        self.assertFalse(camera_ready(True,0.,None,.15,True,False))

    def test_once_lights_begin_readiness_is_closed(self):
        self.assertFalse(camera_ready(True,0.,13.2,.1,True,True))


if __name__=='__main__':unittest.main()
