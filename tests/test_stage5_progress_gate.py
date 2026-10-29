"""Monaco 세 바퀴 CI 정지 판정: 카메라 분할/차량 실제 이동 실패 검증."""
import unittest
from scripts.stage5_race import ProgressWatchdog


class ProgressMonacoTest(unittest.TestCase):
    def test_no_motion_detected_without_fake_three_laps(self):
        x=ProgressWatchdog(lane_timeout_s=10,no_progress_timeout_s=30)
        d={"start_allowed":True,"lane_valid":True}
        pos={"distance_measured_xy_m":0.}
        self.assertIsNone(x.check(d,pos,1))
        self.assertIsNone(x.check(d,pos,30))
        self.assertIn("물리 차량 이동",x.check(d,pos,32))
    def test_lost_lane_detected(self):
        x=ProgressWatchdog(lane_timeout_s=10,no_progress_timeout_s=50)
        p={"distance_measured_xy_m":0.}
        self.assertIsNone(x.check({"start_allowed":True,"lane_valid":True},p,1))
        self.assertIsNone(x.check({"start_allowed":True,"lane_valid":False},p,9))
        self.assertIn("RGB 도로",x.check({"start_allowed":True,"lane_valid":False},p,11))
    def test_moving_with_valid_lane_does_not_false_fail(self):
        x=ProgressWatchdog(lane_timeout_s=10,no_progress_timeout_s=20)
        for t in range(1,90,4):
            d={"start_allowed":True,"lane_valid":True}
            pose={"distance_measured_xy_m":t*.14}
            self.assertIsNone(x.check(d,pose,float(t)))
    def test_before_light_out_does_not_time_out(self):
        x=ProgressWatchdog(lane_timeout_s=10,no_progress_timeout_s=10)
        for t in range(1,50,5):
            self.assertIsNone(x.check({"start_allowed":False,"lane_valid":False},None,t))
    def test_missing_pose_after_start_is_not_pass(self):
        x=ProgressWatchdog(lane_timeout_s=10,no_progress_timeout_s=15)
        x.check({"start_allowed":True,"lane_valid":True},None,0)
        self.assertIn("위치 계측 누락",x.check({"start_allowed":True,"lane_valid":True},None,16))


if __name__=="__main__":unittest.main()
