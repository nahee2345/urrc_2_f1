import unittest
from team_11.red_lamps import count_red_lamps
from team_11.signal_core import StartGate

def frame(lamps):
    w,h=300,140
    pixels=bytearray(w*h*3)
    for i in range(lamps):
        x0=30+i*43
        for y in range(28,42):
            for x in range(x0,x0+10):
                p=(y*w+x)*3
                pixels[p:p+3]=b'\xff\x00\x00'
    return pixels,w,h,w*3

class RedPixelsTests(unittest.TestCase):
    def test_red_lamp_clusters(self):
        for lamps in range(6):
            with self.subTest(lamps=lamps):
                pixels,w,h,stride=frame(lamps)
                self.assertEqual(count_red_lamps(pixels,w,h,stride,'rgb8'),lamps)
    def test_five_lamps_preferred_over_first_four(self):
        pixels,w,h,stride=frame(5)
        self.assertEqual(count_red_lamps(pixels,w,h,stride,'rgb8'),5)

    def test_bgr8_five_lamps(self):
        pixels,w,h,stride=frame(5)
        for i in range(0,len(pixels),3):
            pixels[i],pixels[i+2]=pixels[i+2],pixels[i]
        self.assertEqual(count_red_lamps(pixels,w,h,stride,'bgr8'),5)

    def test_requires_four_then_dark(self):
        gate=StartGate()
        for i in range(5):
            pixels,w,h,stride=frame(5)
            self.assertFalse(gate.observe(count_red_lamps(pixels,w,h,stride,'rgb8'),i*.1))
        for i in range(3):
            pixels,w,h,stride=frame(0)
            result=gate.observe(count_red_lamps(pixels,w,h,stride,'rgb8'),.6+i*.1)
        self.assertTrue(result)
    def test_reject_unsupported_encoding(self):
        pixels,w,h,stride=frame(5)
        with self.assertRaises(ValueError):
            count_red_lamps(pixels,w,h,stride,'mono8')

if __name__=="__main__":unittest.main()
