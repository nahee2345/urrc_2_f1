"""Timekeeping contracts for physically meaningful actuation and recovery."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.steering import SteeringLimiter
from team01_race.mode_recovery import RecoveryGuard
from team01_race.planning_gate import Motion


class ClockTests(unittest.TestCase):
    def test_steering_does_not_move_during_paused_gazebo(self):
        control=SteeringLimiter({'max_center_steer_deg':29.8})
        control.apply(.30,.05,1.)
        previous=control.angle
        self.assertEqual(control.apply(-.30,0.,1.),previous)

    def test_recovery_hold_uses_physics_time(self):
        recovery=RecoveryGuard({'motion_restart_hold_s':.12})
        recovery.update(Motion(0.,0.,'BRAKE','risk'),.05)
        self.assertEqual(recovery.update(
            Motion(.8,0.,'STRAIGHT','ready'),0.).mode,'RECOVER')
        self.assertGreater(recovery.remaining,0.)
        self.assertEqual(recovery.update(
            Motion(.8,0.,'STRAIGHT','ready'),.15).mode,'RECOVER')
        self.assertEqual(recovery.update(
            Motion(.8,0.,'STRAIGHT','ready'),.02).mode,'STRAIGHT')


if __name__=='__main__':unittest.main()
