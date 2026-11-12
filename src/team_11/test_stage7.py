import unittest
from team_11.signal_core import StartGate

class SignalTests(unittest.TestCase):
    def test_darkness_never_starts(self):
        g=StartGate()
        for i in range(40):
            self.assertFalse(g.observe(0,i*.1))
    def test_three_lamps_never_arms(self):
        g=StartGate()
        for i in range(10):
            self.assertFalse(g.observe(3,i*.1))
        for i in range(10,20):
            self.assertFalse(g.observe(0,i*.1))
    def test_four_then_dark_starts(self):
        g=StartGate()
        self.assertFalse(g.observe(4,.1))
        self.assertFalse(g.observe(5,.2))
        self.assertFalse(g.observe(4,.3))
        self.assertFalse(g.observe(0,.4))
        self.assertFalse(g.observe(0,.5))
        self.assertTrue(g.observe(0,.6))
    def test_flicker_not_off(self):
        g=StartGate()
        for i in range(3):g.observe(5,i*.1)
        self.assertFalse(g.observe(0,.3))
        self.assertFalse(g.observe(1,.4))
        self.assertFalse(g.observe(0,.5))
        self.assertFalse(g.observe(0,.6))
        self.assertTrue(g.observe(0,.7))
    def test_timeout_resets(self):
        g=StartGate()
        for i in range(3):g.observe(5,i*.1)
        self.assertFalse(g.observe(0,5.1))
        self.assertFalse(g.observe(0,5.2))
        self.assertFalse(g.observe(0,5.3))
    def test_last_lit_frame_extends_off_window(self):
        g=StartGate()
        self.assertFalse(g.observe(4,0.0))
        self.assertFalse(g.observe(5,0.1))
        # Repeated illuminated frames keep the start gate valid.
        self.assertFalse(g.observe(5,3.8))
        self.assertFalse(g.observe(0,4.0))
        self.assertFalse(g.observe(0,4.2))
        self.assertTrue(g.observe(0,4.4))

    def test_delayed_off_not_treated_as_off_without_red(self):
        g=StartGate()
        for i in range(35):
            self.assertFalse(g.observe(0,i*.1))

    def test_invalid_frame_resets(self):
        g=StartGate()
        for i in range(3):g.observe(5,i*.1)
        self.assertFalse(g.observe(0,.3,False))
        for i in range(4):self.assertFalse(g.observe(0,.4+i*.1))

if __name__=="__main__":unittest.main()
