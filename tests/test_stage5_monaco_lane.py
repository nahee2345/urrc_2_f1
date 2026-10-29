"""Monaco 좁은 도로·그림자·급커브 RGB 기준선 회귀 시험.

모든 이미지는 테스트용 합성 RGB. 실제 Gazebo 카메라/무충돌 완주 증거가 아니다.
"""
from __future__ import annotations
import unittest
from stage4.lane import estimate_lane,_asphalt
from stage4.safety import CameraDrive
from vehicle_generator.model import Vehicle
from vehicle_generator.sensors import SENSOR_SPECS,build


def frame(w=160,h=120,shade=39,curve=0.,coverage=.50,visible=True):
    # RGB(78,82,67): Monaco 노면 외부 녹갈색 바닥 (도로 아님)
    data=bytearray(bytes((78,82,67))*(w*h))
    if not visible:
        return bytes(data)
    for y in range(int(h*coverage),h):
        f=max(0.,min(1.,(.93-y/h)/(.93-.55)))
        center=.50+curve*f
        half=.13-f*.025
        left=max(0,int(w*(center-half)))
        right=min(w,int(w*(center+half)))
        for x in range(left,right):
            k=(y*w+x)*3
            data[k:k+3]=bytes((shade,shade+2,shade+5))
    return bytes(data)


class MonacoLaneRecoveryTests(unittest.TestCase):
    def test_monza_initial_center_still_valid(self):
        est=estimate_lane(160,120,frame(shade=39))
        self.assertTrue(est.valid)
        self.assertLess(abs(est.error),.05)
        self.assertGreaterEqual(est.samples,3)
    def test_shadowed_monaco_black_road(self):
        est=estimate_lane(160,120,frame(shade=9))
        self.assertTrue(est.valid)
    def test_brightened_monaco_asphalt(self):
        est=estimate_lane(160,120,frame(shade=100))
        self.assertTrue(est.valid)
    def test_curved_road_still_visible(self):
        est=estimate_lane(160,120,frame(shade=34,curve=.30))
        self.assertTrue(est.valid)
        self.assertGreater(est.error,.07)
    def test_reverse_curve_steering_sign(self):
        left=estimate_lane(160,120,frame(shade=32,curve=-.27))
        right=estimate_lane(160,120,frame(shade=32,curve=.27))
        self.assertTrue(left.valid)
        self.assertTrue(right.valid)
        self.assertLess(left.error,0)
        self.assertGreater(right.error,0)
    def test_grass_only_must_stop(self):
        est=estimate_lane(160,120,frame(visible=False))
        self.assertFalse(est.valid)
    def test_full_dark_image_must_stop(self):
        # 카메라가 검게 막힌 상황을 도로라고 오인하면 안 된다.
        a=bytes((28,28,28))*(160*120)
        est=estimate_lane(160,120,a)
        self.assertFalse(est.valid)
    def test_only_far_road_must_stop(self):
        data=bytearray(frame(coverage=.52))
        for y in range(int(120*.73),120):
            for x in range(160):
                k=(y*160+x)*3
                data[k:k+3]=bytes((78,82,67))
        self.assertFalse(estimate_lane(160,120,bytes(data)).valid)
    def test_nonroad_rgb_not_asphalt(self):
        self.assertFalse(_asphalt(78,82,67))
        self.assertFalse(_asphalt(255,0,0))
        self.assertFalse(_asphalt(240,240,240))
    def test_camera_pitch_only_for_driving_rgb(self):
        self.assertAlmostEqual(SENSOR_SPECS["rgb"]["pitch"],-.26)
        self.assertAlmostEqual(SENSOR_SPECS["start_camera"]["pitch"],-.55)
        sdf,_=build(Vehicle(name="urrc_ai_01",drive="fwd"),["rgb"])
        self.assertIn("-0.26",sdf)
    def test_missing_lane_does_not_disable_emergency_stop(self):
        c=CameraDrive(Vehicle(name="urrc_ai_01",drive="fwd"),speed_limit_mps=.55)
        lane=estimate_lane(160,120,frame(shade=39,curve=.20))
        self.assertTrue(lane.valid)
        c.permission(True,1.)
        c.camera(lane,1.)
        c.tick(1.)
        self.assertGreater(c.tick(1.1)["speed_mps"],0)
        c.camera(estimate_lane(160,120,frame(visible=False)),1.15)
        stopped=c.tick(1.16)
        self.assertFalse(stopped["allowed"])
        self.assertEqual(stopped["speed_mps"],0)
        self.assertEqual(stopped["drive_wheel_radps"]["front_left"],0)
    def test_rate_limits_still_hold(self):
        c=CameraDrive(Vehicle(name="urrc_ai_01",drive="fwd"),speed_limit_mps=.55)
        c.permission(True,1.)
        c.camera(estimate_lane(160,120,frame(curve=.30)),1.)
        c.tick(1.)
        v=c.tick(1.1)
        self.assertLessEqual(v["speed_mps"],.55)
        self.assertLessEqual(abs(v["race_steer_rad"]),c.steer_limit_rad)
        self.assertLessEqual(abs(v["race_steer_rad"]),c.steer_rate_rps*.1+1e-6)

if __name__=="__main__":
    unittest.main()
