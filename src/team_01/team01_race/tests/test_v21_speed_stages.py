"""Speed stage does not automatically rise on completion of a provisional map."""
import unittest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.speed_stages import SpeedStages


class V21(unittest.TestCase):
    def test_unmapped_always_caps_at_five(self):
        for speed in (5.,8.,10.):
            self.assertEqual(SpeedStages({'validated_speed_stage_mps':speed}).cap(False),5.)
    def test_mapped_cap_requires_explicit_choice(self):
        for speed in (5.,8.,10.):
            self.assertEqual(SpeedStages({'validated_speed_stage_mps':speed}).cap(True),speed)
    def test_invalid_config_is_conservative(self):
        self.assertEqual(SpeedStages({'validated_speed_stage_mps':15.}).cap(True),5.)

if __name__=='__main__':unittest.main()
