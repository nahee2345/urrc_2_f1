"""A red countdown always blocks the spawn timeout alternative."""
import sys
import pathlib
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team01_race.start_fallback import StartFallback


class Gate:
    lights_out=False
    first_bright_time=None
    saw_full_red=False


class Tests(unittest.TestCase):
    def test_seven_seconds_after_first_odometry(self):
        p=StartFallback(7.)
        g=Gate()
        self.assertFalse(p.active(100.,g))
        p.observe_spawn(100.)
        self.assertFalse(p.active(106.99,g))
        self.assertTrue(p.active(107.,g))

    def test_visible_red_disables_fallback(self):
        p=StartFallback(7.);p.observe_spawn(90.)
        g=Gate();g.first_bright_time=92.
        self.assertFalse(p.active(105.,g))
        g.lights_out=True
        self.assertFalse(p.active(106.,g))

    def test_spawn_timestamp_cannot_be_restarted(self):
        p=StartFallback(7.);p.observe_spawn(50.);p.observe_spawn(56.)
        self.assertEqual(p.spawn_wall,50.)


if __name__=='__main__':unittest.main()
