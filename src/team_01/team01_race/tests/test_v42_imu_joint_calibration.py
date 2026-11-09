"""Sensor signs, joint response lag and explicit QA feedback approval."""
import math
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.feedback_guard import FeedbackGuard


class Feedback(unittest.TestCase):
    def test_unapproved_feedback_never_controls_wheels(self):
        g=FeedbackGuard({})
        g.command_sent(.20,10.)
        for t in (10.05,10.10,10.15,10.20,10.25):
            self.assertFalse(g.observe(t,.12,.35,1.,t-.02,t-.02))
        self.assertEqual(g.status,'QA_CALIBRATION_REQUIRED')

    def test_approved_consistent_sign_requires_multiple_frames(self):
        g=FeedbackGuard({'qa_approved_imu_joint_calibration':True})
        g.command_sent(.20,10.)
        for i in range(3):
            self.assertFalse(g.observe(10.1+i*.05,.12,.35,1.,
                                      10.08+i*.05,10.08+i*.05))
        self.assertTrue(g.observe(10.25,.12,.35,1.,10.23,10.23))

    def test_wrong_joint_or_imu_sign_disables_feedback(self):
        g=FeedbackGuard({'qa_approved_imu_joint_calibration':True})
        g.command_sent(.2,10.)
        self.assertFalse(g.observe(10.1,-.12,.4,1.,10.08,10.08))
        self.assertEqual(g.status,'STEERING_OR_YAW_SIGN_MISMATCH')
        self.assertFalse(g.observe(10.2,.12,-.3,1.,10.18,10.18))

    def test_stale_samples_block_qa_approved_feedback(self):
        g=FeedbackGuard({'qa_approved_imu_joint_calibration':True})
        g.command_sent(.2,10.)
        self.assertFalse(g.observe(11.,.1,.4,1.,10.,10.))
        self.assertEqual(g.status,'SENSOR_MISSING_OR_STALE')

    def test_measured_command_to_joint_delay_is_reported(self):
        g=FeedbackGuard({'qa_approved_imu_joint_calibration':True})
        g.command_sent(0.,10.)
        g.command_sent(.2,10.1)
        g.observe(10.3,.2,.25,1.,10.29,10.29)
        self.assertAlmostEqual(g.response_delay,.19,places=6)


if __name__=='__main__':unittest.main()
