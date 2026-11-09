"""Drive-chain outcomes for later independent verification."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.drive_monitor import classify_drive


class DriveChain(unittest.TestCase):
    def test_no_controller(self):
        self.assertEqual(classify_drive(None,None,None,None,0),'NO_CONTROLLER_COMMAND')

    def test_no_odom(self):
        self.assertEqual(classify_drive(1.,{'send_enabled':True},None,None,2.),
                         'ODOMETRY_MISSING_OR_STALE')

    def test_no_motion(self):
        a={'send_enabled':True,'sent_linear_mps':2.}
        self.assertEqual(classify_drive(2.,a,0.,0.,0.),
                         'COMMAND_SENT_NO_MEASURED_MOTION')


if __name__=='__main__':unittest.main()
