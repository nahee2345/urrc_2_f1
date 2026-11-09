"""A one-way corner is valid; unsupported immediate left-right flip is not."""
import math
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.steering_intent import SteeringIntent
from team01_race.steering_joint_feedback import SteeringJointFeedback


class IntentTests(unittest.TestCase):
    def test_monotone_same_side_corner_passes(self):
        gate=SteeringIntent({})
        seq=[0,5,10,15]
        out=[math.degrees(gate.update(math.radians(d),12.,.12,.05)) for d in seq]
        for a,b in zip(seq,out):self.assertAlmostEqual(a,b)
        self.assertEqual(gate.phase,'LEFT_CORNER')

    def test_jittery_opposite_flip_is_held(self):
        gate=SteeringIntent({})
        for d in (0.,5.,10.,15.):
            gate.update(math.radians(d),12.,.12,.05)
        self.assertEqual(gate.update(math.radians(-15.),0.,.01,.05),0.)
        self.assertEqual(gate.phase,'REVERSAL_HELD')

    def test_real_opposite_corner_confirmed(self):
        gate=SteeringIntent({})
        gate.update(math.radians(15.),13.,.1,.05)
        first=[gate.update(math.radians(-15.),-11.,.13,.05)
               for _ in range(3)]
        self.assertEqual(first[:2],[0.,0.])
        self.assertLess(first[2],0.)
        self.assertEqual(gate.phase,'OPPOSITE_CORNER_CONFIRMED')

    def test_long_straight_does_not_unlock_unsupported_flip(self):
        gate=SteeringIntent({})
        gate.update(math.radians(15.),12.,.12,.05)
        for _ in range(40):
            gate.update(0.,0.,0.,.05)
        self.assertEqual(gate.update(math.radians(-15.),0.,0.,.05),0.)
        self.assertEqual(gate.phase,'REVERSAL_HELD')

    def test_actual_steering_feedback_reduces_error(self):
        closed=SteeringJointFeedback({'steering_joint_feedback_gain':.35,
                                      'steering_joint_feedback_max_deg':3.})
        corrected=closed.correct(.25,.10,2.)
        self.assertGreater(corrected,.25)
        self.assertLessEqual(corrected-.25,math.radians(3.)+1e-6)
        self.assertEqual(closed.source,'JOINT_CLOSED_LOOP')

    def test_no_joint_measurement_does_not_fabricate_angle(self):
        closed=SteeringJointFeedback({})
        self.assertEqual(closed.correct(.12,None,2.),.12)
        self.assertEqual(closed.source,'OPEN_LOOP')


if __name__=='__main__':unittest.main()
