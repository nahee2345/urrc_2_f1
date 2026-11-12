import math
import unittest
from team_11.stage5_core import convert
class TestStage5(unittest.TestCase):
    def test_sign(self):
        self.assertLess(convert(1,math.radians(10),True,True,True)[1],0)
        self.assertGreater(convert(1,-math.radians(10),True,True,True)[1],0)
    def test_gate(self):
        self.assertEqual(convert(1,0,False,True,True),(0.,0.))
    def test_timeout(self):
        self.assertEqual(convert(1,0,True,False,True),(0.,0.))
    def test_speed_limit(self):
        self.assertEqual(convert(99,0,True,True,True)[0],15.)
        self.assertEqual(convert(-99,0,True,True,True)[0],-2.)
    def test_nan(self):
        self.assertEqual(convert(float("nan"),0,True,True,True),(0.,0.))
