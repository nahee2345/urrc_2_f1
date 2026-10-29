"""실제 Monaco Gazebo RGB 원본 관찰 기반 좁은 도로 영상 대역 회귀.

실제 현장 프레임은 CI에서 별도로 읽어야 하며 본 테스트는 synthetic이다.
"""
import unittest
from stage4.lane import estimate_lane


def narrow_monaco_band(w=160,h=120):
    px=bytearray(bytes((0,0,0))*(w*h))
    # 실제 frame의 도로 진입 픽셀: y~92..105 사이에 쐐기 모양 차도.
    rows={92:(60,98),94:(36,102),96:(10,111),
          98:(0,120),100:(0,130),102:(0,139),104:(0,149)}
    for y,(left,right) in rows.items():
        for dy in (0,1):
            for x in range(left,right):
                k=((y+dy)*w+x)*3
                px[k:k+3]=bytes((41,43,47))
    return bytes(px)


class MonacoRealCameraBandRegression(unittest.TestCase):
    def test_actual_monaco_band_around_77_to_87_percent(self):
        est=estimate_lane(160,120,narrow_monaco_band())
        self.assertTrue(est.valid,est)
        self.assertGreaterEqual(est.samples,4)
        self.assertGreater(est.confidence,.32)
    def test_only_one_band_line_is_not_road(self):
        source=bytearray(narrow_monaco_band())
        w=160
        for y in range(96,120):
            source[y*w*3:(y+1)*w*3]=bytes((0,0,0))*w
        self.assertFalse(estimate_lane(160,120,bytes(source)).valid)

if __name__=="__main__":
    unittest.main()
