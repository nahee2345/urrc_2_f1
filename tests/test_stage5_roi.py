"""Monaco 카메라 5등 자동 교정: 가짜 영상 단위 검사. 실제 카메라 미검증."""
import unittest
from stage5.roi import detect_five_red_lenses
from scripts.stage5_calibrate_lights import stable_rois


def fake_frame(w=160,h=120,count=5):
    raw=bytearray(bytes((40,40,45))*(w*h))
    for j in range(count):
        x=28+j*22
        for yy in range(20,25):
            for xx in range(x,x+5):
                k=(yy*w+xx)*3
                raw[k:k+3]=bytes((240,4,4))
    return bytes(raw)

class RoiTests(unittest.TestCase):
    def test_five_red_bulbs_detected(self):
        rois=detect_five_red_lenses(160,120,fake_frame())
        self.assertEqual(len(rois),5)
        self.assertTrue(all(0<a[0]<a[2]<1 for a in rois))
        self.assertEqual(rois,stable_rois([rois,rois,rois]))
    def test_four_red_bulbs_rejected(self):
        with self.assertRaises(ValueError):
            detect_five_red_lenses(160,120,fake_frame(count=4))
    def test_blank_and_bad_size_rejected(self):
        with self.assertRaises(ValueError):
            detect_five_red_lenses(160,120,bytes((0,0,0))*(160*120))
        with self.assertRaises(ValueError):
            detect_five_red_lenses(160,120,b"")
    def test_roi_motion_rejected(self):
        base=detect_five_red_lenses(160,120,fake_frame())
        moved=[[a+.1,b,c+.1,d] for a,b,c,d in base]
        with self.assertRaises(ValueError):stable_rois([base,moved,base])

if __name__=="__main__":unittest.main()
