"""Independent QA approval and SLAM confidence are both necessary."""
import unittest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.speed_stages import SpeedStages


class V21(unittest.TestCase):
    def test_unmapped_caps_at_five_even_when_approved(self):
        for speed in (5.,8.,10.):
            self.assertEqual(SpeedStages({'validated_speed_stage_mps':speed,
                'qa_approved_first_corner':True,'qa_approved_monza_lap':True,
                'qa_approved_8mps':True}).cap(False),5.)

    def test_no_qa_approval_no_speed_increase(self):
        self.assertEqual(SpeedStages({'validated_speed_stage_mps':10.}).cap(True),5.)

    def test_both_corner_and_lap_required(self):
        cfg={'validated_speed_stage_mps':8.,'qa_approved_first_corner':True}
        self.assertEqual(SpeedStages(cfg).cap(True),5.)
        cfg['qa_approved_monza_lap']=True
        self.assertEqual(SpeedStages(cfg).cap(True),8.)

    def test_ten_requires_eight_mps_approval(self):
        cfg={'validated_speed_stage_mps':10.,'qa_approved_first_corner':True,
             'qa_approved_monza_lap':True}
        self.assertEqual(SpeedStages(cfg).cap(True),8.)
        cfg['qa_approved_8mps']=True
        self.assertEqual(SpeedStages(cfg).cap(True),10.)

    def test_twenty_unavailable_at_current_sensor_range(self):
        self.assertEqual(SpeedStages({'validated_speed_stage_mps':20.}).cap(True),5.)


if __name__=='__main__':unittest.main()
