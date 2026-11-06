"""Gazebo physics-time watchdog with an independent wall-time cutoff."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.command_freshness import fresh_command


class Watchdog(unittest.TestCase):
    def test_slow_wall_clock_does_not_zero_recent_sim_command(self):
        # Headless renderer: only 0.06 sim seconds in 1.1 wall seconds.
        self.assertTrue(fresh_command(101.06,101.,12.1,11.))

    def test_physics_time_stale_is_rejected(self):
        self.assertFalse(fresh_command(101.4,101.,11.2,11.))

    def test_long_wall_stall_is_still_rejected(self):
        self.assertFalse(fresh_command(101.02,101.,14.,11.))

    def test_unknown_clock_retains_conservative_wall_timeout(self):
        self.assertTrue(fresh_command(0.,0.,11.1,11.))
        self.assertFalse(fresh_command(0.,0.,11.31,11.))

    def test_sim_clock_reset_or_future_stamp_is_rejected(self):
        self.assertFalse(fresh_command(100.,101.,11.1,11.))

    def test_no_speed_message_yet_is_rejected(self):
        self.assertFalse(fresh_command(100.,0.,11.1,0.))


if __name__=='__main__':unittest.main()
