"""4단계 AI-01 실제 RGB 도로 인식/물리 구동 인터록 모의 입력 회귀시험.

카메라 프레임은 테스트가 구성한 모의 색상 데이터이다.
실제 Gazebo의 화면 인식/모터 주행 PASS로 혼동하지 않는다.
"""
from dataclasses import replace
import math
import unittest
from xml.etree import ElementTree as ET
from vehicle_generator.model import Vehicle,generate
from vehicle_generator.sensors import build
from stage4.lane import estimate_lane
from stage4.safety import CameraDrive
from scripts.stage4_bridge import bridge_yaml


def image(w=160,h=120,center=.5,road_width=.30,visible=True):
    data=bytearray(bytes((78,82,67))*(w*h))
    if visible:
        left=int(w*max(0,center-road_width/2))
        right=int(w*min(1,center+road_width/2))
        for y in range(int(h*.57),h):
            for x in range(left,right):
                k=(y*w+x)*3
                data[k:k+3]=bytes((39,42,46))
    return bytes(data)


class Stage4VisionTests(unittest.TestCase):
    def test_center_road(self):
        m=estimate_lane(160,120,image())
        self.assertTrue(m.valid)
        self.assertAlmostEqual(m.error,0.,delta=.05)
        self.assertGreater(m.confidence,.6)
    def test_left_and_right_road_error(self):
        l=estimate_lane(160,120,image(center=.38))
        r=estimate_lane(160,120,image(center=.62))
        self.assertTrue(l.valid)
        self.assertTrue(r.valid)
        self.assertLess(l.error,0.)
        self.assertGreater(r.error,0.)
    def test_blank_is_fail_closed(self):
        m=estimate_lane(160,120,image(visible=False))
        self.assertFalse(m.valid)
    def test_invalid_image_and_prior(self):
        with self.assertRaises(ValueError):estimate_lane(0,120,b"")
        with self.assertRaises(ValueError):estimate_lane(160,120,image(),float("nan"))
    def test_fine_white_lane_marking_not_road(self):
        raw=bytearray(image())
        for y in range(75,110):
            for x in (48,49,110,111):
                k=(y*160+x)*3
                raw[k:k+3]=bytes((230,230,230))
        m=estimate_lane(160,120,bytes(raw))
        self.assertTrue(m.valid)


class Stage4SafetyTests(unittest.TestCase):
    def make(self):
        return CameraDrive(replace(Vehicle(),name="urrc_ai_01",drive="fwd"))
    def test_gate_disabled_zero_wheels(self):
        c=self.make()
        c.camera(estimate_lane(160,120,image()),1.)
        for t in (1.0,1.1,1.2):
            out=c.tick(t)
            self.assertFalse(out["allowed"])
            self.assertEqual(out["speed_mps"],0.)
            self.assertTrue(all(v==0 for v in out["drive_wheel_radps"].values()))
    def test_true_only_with_both_fresh(self):
        c=self.make()
        c.camera(estimate_lane(160,120,image()),1.)
        c.permission(True,1.)
        c.tick(1.0)
        x=c.tick(1.1)
        self.assertTrue(x["allowed"])
        self.assertGreater(x["speed_mps"],0)
        self.assertEqual(set(x["drive_wheel_radps"]),{"front_left","front_right"})
        self.assertNotIn("rear_left",x["drive_wheel_radps"])
    def test_gate_false_stops_immediately(self):
        c=self.make();c.camera(estimate_lane(160,120,image()),1)
        c.permission(True,1);c.tick(1);c.tick(1.1)
        c.permission(False,1.11)
        x=c.tick(1.12)
        self.assertFalse(x["allowed"])
        self.assertEqual(x["speed_mps"],0)
        self.assertEqual(x["steer_left_rad"],0)
        self.assertTrue(all(v==0 for v in x["drive_wheel_radps"].values()))
    def test_lost_camera_brakes(self):
        c=self.make()
        c.camera(estimate_lane(160,120,image()),1)
        c.permission(True,1)
        c.tick(1);c.tick(1.1)
        c.permission(True,1.6)
        out=c.tick(1.65)
        self.assertFalse(out["allowed"])
        self.assertEqual(out["speed_mps"],0.)
        self.assertIn("카메라",out["reason"])
    def test_lost_permission_brakes(self):
        c=self.make()
        c.camera(estimate_lane(160,120,image()),1)
        c.permission(True,1)
        c.tick(1);c.tick(1.1)
        c.camera(estimate_lane(160,120,image()),1.3)
        out=c.tick(1.31)
        self.assertFalse(out["allowed"])
        self.assertEqual(out["speed_mps"],0.)
        self.assertIn("허가",out["reason"])
    def test_no_road_does_not_start(self):
        c=self.make();c.permission(True,1)
        c.camera(estimate_lane(160,120,image(visible=False)),1)
        self.assertFalse(c.tick(1.)["allowed"])
    def test_steering_sign_right(self):
        c=self.make()
        c.camera(estimate_lane(160,120,image(center=.64)),1)
        c.permission(True,1.)
        c.tick(1)
        out=c.tick(1.1)
        self.assertTrue(out["allowed"])
        self.assertGreater(out["race_steer_rad"],0)
        self.assertLess(out["steer_right_rad"],0)
        self.assertGreaterEqual(out["speed_mps"],0)
    def test_time_reversal_rejected(self):
        c=self.make()
        c.tick(1.)
        with self.assertRaises(ValueError):c.tick(.9)
        self.assertFalse(c.started)


class Stage4ModelTests(unittest.TestCase):
    def test_black_body_visual(self):
        v=Vehicle()
        root=ET.fromstring(generate(v))
        body=root.find("./model/link[@name='base_link']/visual[@name='body_visual']")
        rgb=[float(x) for x in body.findtext("material/diffuse").split()[:3]]
        self.assertEqual(len(rgb),3)
        self.assertTrue(all(x<.05 for x in rgb))
        self.assertTrue(all(x>0 for x in rgb))
    def test_stage2_sensor_does_not_remove_black(self):
        root=ET.fromstring(build(Vehicle(name="urrc_ai_01",drive="fwd"),["rgb"])[0])
        visual=root.find("./model/link[@name='base_link']/visual[@name='body_visual']")
        self.assertIn("0.028",visual.findtext("material/diffuse"))
    def test_stage4_bridge_only_rgb_and_actuator(self):
        s=bridge_yaml({"vehicle":"urrc_ai_01",
                       "topics":["/model/urrc_ai_01/contacts/body"]})
        self.assertIn("/model/urrc_ai_01/sensors/rgb",s)
        self.assertIn("std_msgs/msg/Float64",s)
        self.assertEqual(s.count("direction: ROS_TO_GZ"),4)
        self.assertNotIn("centerline",s)
        with self.assertRaises(ValueError):
            bridge_yaml({"vehicle":"urrc_ai_10","topics":[]})

if __name__=="__main__":unittest.main()
