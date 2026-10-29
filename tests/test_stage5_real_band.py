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
    def test_real_camera_style_near_full_width_with_far_boundaries(self):
        # 합성 회귀: 실제 CI RGB에서 관찰한 근거리 전폭 차도/전방 경계 패턴.
        # 실주행 PASS 증거는 GitHub Actions Gazebo 통합 검증만 인정한다.
        w,h=160,120
        px=bytearray(bytes((0,0,0))*(w*h))
        for y in range(75,120):
            if y>=94:
                left,right=0,w
            else:
                left=max(0,65-4*(y-75))
                right=min(w,109+3*(y-75))
            for x in range(left,right):
                k=(y*w+x)*3
                px[k:k+3]=bytes((40,42,46))
        est=estimate_lane(w,h,bytes(px))
        self.assertTrue(est.valid,est)
        self.assertGreaterEqual(est.samples,4)

    def test_only_one_band_line_is_not_road(self):
        source=bytearray(narrow_monaco_band())
        w=160
        for y in range(96,120):
            source[y*w*3:(y+1)*w*3]=bytes((0,0,0))*w
        self.assertFalse(estimate_lane(160,120,bytes(source)).valid)

class MonacoLookaheadDirectionTest(unittest.TestCase):
    def test_far_road_right_near_center_drives_right_early(self):
        w,h=160,120
        bg=bytes((88,96,75))
        img=bytearray(bg*(w*h))
        for row in range(55,112):
            frac=(row-55)/56
            # 카메라 가까이는 중앙 도로, 먼 도로는 우측으로 휘어 있음.
            center=int(102-22*frac)
            half=int(10+22*frac)
            for x in range(max(0,center-half),min(w,center+half)):
                k=(row*w+x)*3
                img[k:k+3]=bytes((40,42,45))
        from stage4.lane import estimate_lane
        lane=estimate_lane(w,h,bytes(img))
        self.assertTrue(lane.valid,lane)
        self.assertGreater(lane.error,.10)
    def test_no_remote_roads_remains_fail_closed(self):
        from stage4.lane import estimate_lane
        rgb=bytes((95,95,95))*(160*120)
        self.assertFalse(estimate_lane(160,120,rgb).valid)


if __name__=="__main__":
    unittest.main()
