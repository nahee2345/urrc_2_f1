"""Reachable steering rescue, without changing path footprint or steering slew."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.steering_recovery import choose_safe_steer
from team01_race.steering import SteeringLimiter


CFG={'max_center_steer_deg':29.8,
     'max_steer_rate_deg_s':35.,'max_steer_accel_deg_s2':170.,
     'steer_target_filter_tau_s':.10}


class SafeLimitedControl(unittest.TestCase):
    def test_requested_first_command_kept_if_safe(self):
        x=SteeringLimiter(CFG)
        steer,mode=choose_safe_steer(x,.1,.05,1.,lambda angle:True,.52,True)
        self.assertEqual(mode,'VERIFIED')
        self.assertEqual(x.angle,steer)

    def test_near_boundary_finds_reachable_smaller_turn(self):
        x=SteeringLimiter(CFG)
        x.angle=x.filtered_target=.20
        steer,mode=choose_safe_steer(
            x,.20,.05,1.,lambda angle:angle<.199,.52,True)
        self.assertEqual(mode,'BOUNDED_REPLAN')
        self.assertIsNotNone(steer)
        self.assertLess(steer,.199)
        self.assertGreater(steer,.18)

    def test_no_safe_candidate_fails_closed_and_restores_state(self):
        x=SteeringLimiter(CFG)
        x.angle=.14;x.rate=.03;x.filtered_target=.17
        old=(x.angle,x.rate,x.filtered_target)
        steer,mode=choose_safe_steer(
            x,.20,.05,1.,lambda angle:False,.52,True)
        self.assertIsNone(steer)
        self.assertEqual(mode,'NO_REACHABLE_SAFE_STEER')
        self.assertEqual(old,(x.angle,x.rate,x.filtered_target))

    def test_never_try_alternatives_if_not_stoppable_within_verified_horizon(self):
        x=SteeringLimiter(CFG)
        steer,mode=choose_safe_steer(
            x,.25,.05,1.,lambda angle:abs(angle)<.0001,.52,False)
        self.assertIsNone(steer)
        self.assertEqual(mode,'NO_REACHABLE_SAFE_STEER')


if __name__=='__main__':unittest.main()
