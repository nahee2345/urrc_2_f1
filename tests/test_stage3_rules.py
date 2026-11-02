"""3단계 출발·랩·안전 규칙의 회귀검사.

가짜 RGB 픽셀/가상 게이트 단위 검사이며 실제 빨간불 영상 인식 실험은 아니다.
"""
from __future__ import annotations
from dataclasses import dataclass
import tempfile
from pathlib import Path
import json
import csv
import math
import unittest
from stage3.vision import LightOutGate,rgb_from_ros,roi_red_ratios
from stage3.lap import crossing,LapSupervisor,gates_from_upstream
from stage3.contact import ContactEvent,classify
from stage3.ros_gate import load_calibration


ROIS=[[0.01+i*.2,.15,0.10+i*.2,.42] for i in range(5)]


def frame_on(n_on=5,width=100,height=20):
    b=bytearray(width*height*3)
    for k,roi in enumerate(ROIS):
        if k>=n_on:continue
        x0,y0,x1,y1=roi
        for y in range(int(y0*height),int(y1*height)):
            for x in range(int(x0*width),int(x1*width)):
                p=(y*width+x)*3
                b[p:p+3]=bytes((245,5,5))
    return bytes(b)


@dataclass
class MockImage:
    width:int=100
    height:int=20
    encoding:str="rgb8"
    step:int=300
    data:bytes=b""


class VisionTests(unittest.TestCase):
    def test_all_five_red_then_off_with_operator_permission(self):
        g=LightOutGate(ROIS,stable_frames=2)
        # 원본 운영자의 Bool 신호가 소등 직전까지 False여도 빨간 5등은 관측한다.
        self.assertFalse(g.update(100,20,frame_on(4),.01,False))
        self.assertFalse(g.update(100,20,frame_on(5),.10,False))
        self.assertFalse(g.update(100,20,frame_on(5),.20,False))
        self.assertEqual(g.state,"WAIT_ALL_OFF")
        self.assertFalse(g.update(100,20,frame_on(0),.30,False))
        self.assertFalse(g.update(100,20,frame_on(0),.40,True))
        self.assertTrue(g.update(100,20,frame_on(0),.50,True))
        self.assertEqual(g.state,"RELEASED")
        self.assertTrue(g.update(100,20,frame_on(0),.60,True))
        self.assertFalse(g.update(100,20,frame_on(0),.70,False))
        self.assertEqual(g.state,"WAIT_FIVE_RED")

    def test_never_release_when_start_already_dark(self):
        g=LightOutGate(ROIS,stable_frames=2)
        for i in range(10):
            self.assertFalse(g.update(100,20,frame_on(0),i*.1,True))
        self.assertEqual(g.state,"WAIT_FIVE_RED")

    def test_never_release_on_four_red(self):
        g=LightOutGate(ROIS,stable_frames=2)
        for i in range(5):
            self.assertFalse(g.update(100,20,frame_on(4),.1*i,True))
        for i in range(5,9):
            self.assertFalse(g.update(100,20,frame_on(0),.1*i,True))

    def test_flicker_debounce(self):
        g=LightOutGate(ROIS,stable_frames=3)
        for t,num in ((.1,5),(.2,4),(.3,5),(.4,5),(.5,5),(.6,0),(.7,5),(.8,0),(.9,0)):
            self.assertFalse(g.update(100,20,frame_on(num),t,True))
        self.assertTrue(g.update(100,20,frame_on(0),1.0,True))

    def test_timestamp_gap_resets(self):
        g=LightOutGate(ROIS,stable_frames=2)
        g.update(100,20,frame_on(5),.1,True)
        g.update(100,20,frame_on(5),.2,True)
        self.assertEqual(g.state,"WAIT_ALL_OFF")
        self.assertFalse(g.update(100,20,frame_on(0),2.0,True))
        self.assertEqual(g.state,"WAIT_FIVE_RED")

    def test_color_threshold_and_wrong_format(self):
        ratios=roi_red_ratios(100,20,frame_on(5),ROIS)
        self.assertTrue(all(v>.7 for v in ratios))
        self.assertTrue(all(v==0 for v in roi_red_ratios(100,20,frame_on(0),ROIS)))
        with self.assertRaises(ValueError):roi_red_ratios(100,20,frame_on(5),ROIS[:4])
        with self.assertRaises(ValueError):rgb_from_ros(MockImage(encoding="32FC1"))

    def test_ros_pixel_conversion(self):
        bgr=MockImage(2,1,"bgr8",6,bytes([5,6,250, 3,4,240]))
        w,h,rgb=rgb_from_ros(bgr)
        self.assertEqual((w,h), (2,1))
        self.assertEqual(rgb,bytes([250,6,5,240,4,3]))
        with self.assertRaises(ValueError):rgb_from_ros(MockImage(2,1,"rgb8",6,b"\0"))

    def test_calibration_fail_closed(self):
        with tempfile.TemporaryDirectory() as t:
            f=Path(t)/"cfg.json"
            f.write_text(json.dumps({"calibrated":False,"source_sensor":"start_camera","rois":ROIS}))
            with self.assertRaises(ValueError):load_calibration(f)
            f.write_text(json.dumps({"calibrated":True,"source_sensor":"start_camera","rois":ROIS}))
            self.assertEqual(len(load_calibration(f).rois),5)


class LapTests(unittest.TestCase):
    @staticmethod
    def gate(x):
        return dict(x=x,y=0.,z=0.,yaw=0.,half_width=1.0,height_tolerance=.6)

    def timer(self):
        return LapSupervisor(self.gate(0),[self.gate(1),self.gate(2),self.gate(3)],
                             target=3,max_gap=.5,max_step=10.)

    def do(self,lap):
        t=[0.]
        def sample(x):
            t[0]+=.1
            return lap.update((t[0],float(x),0.,0.))
        sample(-.5)
        lap.arm(t[0])
        approach=sample(.5)
        self.assertEqual(approach[0]["type"],"approach")
        self.assertEqual(lap.laps,0)
        for n in range(3):
            for x in (1.5,2.5,3.5,-.5):
                sample(x)
            events=sample(.5)
            self.assertEqual(events[-1]["type"],"lap")
            self.assertEqual(lap.laps,n+1)
        return lap

    def test_three_valid_laps_only_from_white_finish(self):
        lap=self.do(self.timer())
        self.assertTrue(lap.finished)
        self.assertEqual(lap.raw_finish_crossings,4)
        self.assertEqual(lap.status()["status"],"PASS_LAPS_ONLY")
        lap.contact_monitor_ready=True
        self.assertEqual(lap.status()["status"],"PASS_THREE_LAPS")
        self.assertEqual(len(lap.lap_seconds),3)

    def test_no_lap_without_checkpoints(self):
        lap=self.timer()
        lap.update((0,-.5,0,0));lap.arm(0)
        lap.update((.1,.5,0,0))
        lap.update((.2,-.5,0,0))
        events=lap.update((.3,.5,0,0))
        self.assertEqual(lap.laps,0)
        self.assertEqual(lap.raw_finish_crossings,2)
        self.assertEqual(events[-1]["type"],"rejected_finish")
        self.assertEqual(lap.rejected,1)

    def test_reversed_and_outside_finish_not_counted(self):
        f=self.gate(0)
        self.assertIsNone(crossing((0,.5,0,0),(.1,-.5,0,0),f))
        self.assertIsNone(crossing((0,-.5,2,0),(.1,.5,2,0),f))
        self.assertIsNone(crossing((0,-.5,0,1),(.1,.5,0,1),f))
        self.assertAlmostEqual(crossing((0,-.5,0,0),(.1,.5,0,0),f),.05)

    def test_collision_disqualifies(self):
        lap=self.timer();lap.update((0,-.5,0,0));lap.arm(0)
        lap.contact("road",.1)
        self.assertTrue(lap.active)
        lap.contact("wall",.2)
        self.assertFalse(lap.active)
        self.assertEqual(lap.collision_count,1)
        self.assertEqual(lap.status()["status"],"NOT_PASSED")

    def test_invalid_position_gap_disqualifies(self):
        lap=self.timer();lap.update((0,-.5,0,0));lap.arm(0)
        with self.assertRaises(ValueError):lap.update((1,.5,0,0))
        self.assertFalse(lap.active)

    def test_upstream_metadata_gate_reader(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/"src/urrc_track_gazebo/tracks/processed"
            p.mkdir(parents=True)
            (p/"monza.json").write_text(json.dumps({"start_finish":{"x":0,"y":0,"z":0,"yaw":0}}))
            with (p/"monza_centerline.csv").open("w",newline="") as f:
                out=csv.writer(f);out.writerow(["x_m","y_m","z_m","yaw_rad","road_width_m"])
                for i in range(31):out.writerow([i,0,0,0,2])
            finish,cps=gates_from_upstream(Path(t),"monza",spacing_m=4)
            self.assertEqual(finish["x"],0)
            self.assertGreaterEqual(len(cps),3)


class ContactTests(unittest.TestCase):
    def test_actual_collider_names(self):
        own="urrc_ai_01"
        def e(other):return ContactEvent(f"{own}::base_link::body_collision",other,1.0)
        self.assertEqual(classify(e("urrc_monza::road::road_collision"),own),"road")
        self.assertEqual(classify(e("urrc_monza::wall_inner::wall_collision"),own),"wall")
        self.assertEqual(classify(e("other_car::body::collision"),own),"vehicle")
        self.assertEqual(classify(e("other::something"),own),"unknown_collision")

if __name__=="__main__":unittest.main()
