"""Static camera transition tests; no ROS/Gazebo installed."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team11_race.camera_start import LightTransition,red_evidence


def simulated_image(red_clusters, width=640,height=360,encoding="rgb8"):
    # Include five red lamps clustered inside only two of the former screen
    # columns, reproducing the v11 camera alignment failure mode.
    data=bytearray([9,9,9]*(width*height))
    red=(250,3,2) if encoding=="rgb8" else (2,3,250)
    for x,y in red_clusters:
        for dy in range(0,16):
            for dx in range(0,4):
                i=((y+dy)*width+x+dx)*3
                data[i:i+3]=bytes(red)
    return data, width*3,encoding


class CameraStartTests(unittest.TestCase):
    def test_five_clustered_lamps_are_detected(self):
        lamps=[(280+i*15,115) for i in range(5)]
        img,step,enc=simulated_image(lamps)
        count=red_evidence(img,640,360,step,enc)
        self.assertGreaterEqual(count,6)
        self.assertGreater(count,red_evidence(*((simulated_image([])[0],640,360,step,enc))))

    def test_red_then_dark_arms_but_not_before(self):
        g=LightTransition()
        self.assertFalse(g.update(0))
        self.assertFalse(g.update(20))
        self.assertFalse(g.update(23))
        self.assertTrue(g.saw_red)
        for i in range(3):self.assertFalse(g.update(0))
        self.assertTrue(g.update(0))
        self.assertTrue(g.started)

    def test_requires_prior_red_and_not_camera_dropout(self):
        g=LightTransition()
        for _ in range(8):self.assertFalse(g.update(0))
        self.assertFalse(g.update(-1))
        self.assertFalse(g.started)

    def test_four_lamps_then_zero(self):
        g=LightTransition()
        for count in [15,25,19,0,0,0,0]:
            result=g.update(count)
        self.assertTrue(result)

    def test_bgr_and_invalid_image(self):
        buf,step,enc=simulated_image([(310,125)],encoding="bgr8")
        self.assertGreater(red_evidence(buf,640,360,step,enc),0)
        self.assertEqual(red_evidence(buf,640,360,step,"mono8"),-1)
        self.assertEqual(red_evidence(buf[:10],640,360,step,enc),-1)


if __name__=="__main__":
    unittest.main()
