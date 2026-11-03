"""Recovery only delays acceleration; never blocks a safety brake."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.planning_gate import Motion
from team01_race.mode_recovery import RecoveryGuard,CaptureMode


class V18(unittest.TestCase):
    def test_unsafe_brake_is_immediate(self):
        r=RecoveryGuard({'motion_restart_hold_s':.12})
        m=r.update(Motion(0.,0.,'BRAKE','curb'),.05)
        self.assertEqual(m.mode,'BRAKE')
        self.assertEqual(r.reasons['curb'],1)
        self.assertEqual(r.last_fault,'curb')

    def test_restart_requires_healthy_interval(self):
        r=RecoveryGuard({'motion_restart_hold_s':.12})
        r.update(Motion(0.,0.,'STOP','stale lidar'),.05)
        first=r.update(Motion(2.,0.,'STRAIGHT','safe'),.05)
        self.assertEqual(first.speed,0.)
        self.assertEqual(first.mode,'RECOVER')
        for _ in range(5):out=r.update(Motion(2.,0.,'STRAIGHT','safe'),.05)
        self.assertEqual(out.speed,2.)

    def test_capture_hysteresis_ignores_threshold_noise(self):
        c=CaptureMode()
        for d in [3.9,4.1,3.8,4.2]:
            self.assertTrue(c.update(d,.034))
        self.assertFalse(c.update(6.,.05))
        self.assertFalse(c.update(4.,.034))
        self.assertTrue(c.update(2.,.02))


if __name__=='__main__':unittest.main()
