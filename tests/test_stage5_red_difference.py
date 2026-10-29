"""신호등 이외의 빨간 물체가 2~3개 있는 실제 상황을 재현한 회귀 시험."""
import unittest
from stage5.roi import detect_five_red_lenses,red_components
from scripts.stage5_calibrate_lights import stable_rois,read_ppm,save_ppm
from pathlib import Path
from tempfile import TemporaryDirectory


def frame(red_lamps=5,extra=2,shift=0):
    w,h=160,120
    a=bytearray(bytes((41,44,48))*(w*h))
    def rect(x,y,color,span=5):
        for yy in range(y,y+span):
            for xx in range(x,x+span):
                k=(yy*w+xx)*3
                a[k:k+3]=bytes(color)
    for i in range(extra):
        rect(13+130*i//max(extra-1,1),80,(245,8,12),span=4)
    for i in range(red_lamps):
        rect(28+i*22+shift,22,(242,6,8))
    return w,h,bytes(a)


class RealRedSelectionTests(unittest.TestCase):
    def test_original_image_has_seven_red_components(self):
        w,h,frame_on=frame(extra=2)
        self.assertEqual(len(red_components(w,h,frame_on)),7)
        with self.assertRaises(ValueError):
            detect_five_red_lenses(w,h,frame_on)
    def test_difference_suppresses_static_red_objects(self):
        w,h,off=frame(red_lamps=0,extra=2)
        _,_,on=frame(red_lamps=5,extra=2)
        candidates=red_components(w,h,on,off)
        self.assertEqual(len(candidates),5)
        rois=detect_five_red_lenses(w,h,on,off)
        self.assertEqual(len(rois),5)
        self.assertEqual(rois,stable_rois([rois,rois,rois]))
    def test_eight_red_candidates_still_calibrate_from_off(self):
        w,h,off=frame(red_lamps=0,extra=3)
        _,_,on=frame(red_lamps=5,extra=3)
        self.assertEqual(len(red_components(w,h,on)),8)
        self.assertEqual(len(detect_five_red_lenses(w,h,on,off)),5)
    def test_four_new_lights_refused(self):
        w,h,off=frame(red_lamps=0)
        _,_,on=frame(red_lamps=4)
        with self.assertRaisesRegex(ValueError,"5개가 보이지"):
            detect_five_red_lenses(w,h,on,off)
    def test_unchanged_red_objects_refused(self):
        w,h,off=frame(red_lamps=5)
        with self.assertRaises(ValueError):
            detect_five_red_lenses(w,h,off,off)
    def test_no_baseline_fails_closed_on_extra_red(self):
        w,h,on=frame()
        with self.assertRaisesRegex(ValueError,"차분 교정"):
            detect_five_red_lenses(w,h,on)
    def test_misalignment_fails_closed(self):
        w,h,off=frame(red_lamps=0)
        _,_,on=frame()
        broken=bytearray(on)
        for yy in range(22,27):
            for xx in range(72,77):
                k=(yy*w+xx)*3
                broken[k:k+3]=bytes((41,44,48))
        with self.assertRaises(ValueError):
            detect_five_red_lenses(w,h,bytes(broken),off)
    def test_baseline_file_round_trip(self):
        w,h,off=frame(red_lamps=0)
        with TemporaryDirectory() as d:
            p=Path(d)/"off.ppm"
            save_ppm(p,w,h,off)
            self.assertEqual(read_ppm(p),(w,h,off))
            p.write_bytes(b"corrupt")
            with self.assertRaises(ValueError):read_ppm(p)


if __name__=="__main__":unittest.main()
