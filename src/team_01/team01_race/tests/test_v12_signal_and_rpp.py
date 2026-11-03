"""Real slow-camera regression; previous Monza CI observed 1->3->4->0."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.signal_vision import SignalVision
from team01_race.planning import TrafficGate
from team01_race.road_surface import aligned_depth_rgb
from test_v5_signal_runtime import camera_scene

class V12Regression(unittest.TestCase):
    def test_sparse_1_3_4_to_zero_sequence_arms(self):
        vision=SignalVision()
        gate=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        for i in range(24):
            n,s=vision.observe(camera_scene(0))
            self.assertFalse(gate.observe(n,s,timestamp=i*.1))
        for lamps,t in ((1,8.),(3,9.25),(4,10.5)):
            n,s=vision.observe(camera_scene(lamps))
            self.assertFalse(gate.observe(n,s,timestamp=t))
        self.assertTrue(gate.saw_full_red)
        self.assertEqual(gate.status,'ARMED')
        for t in (11.85,13.1,14.35):
            n,s=vision.observe(camera_scene(0))
            gate.observe(n,s,timestamp=t)
        self.assertTrue(gate.lights_out)

    def test_single_four_lamp_flash_cannot_arm(self):
        vision=SignalVision()
        gate=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        for i in range(30):
            n,s=vision.observe(camera_scene(0))
            gate.observe(n,s,timestamp=i*.1)
        n,s=vision.observe(camera_scene(4))
        gate.observe(n,s,timestamp=8.)
        for t in (8.8,9.7,10.9,11.8):
            n,s=vision.observe(camera_scene(0))
            self.assertFalse(gate.observe(n,s,timestamp=t))
        self.assertFalse(gate.saw_full_red)

    def test_three_lamps_never_arm(self):
        vision=SignalVision()
        gate=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        for i in range(25):
            n,s=vision.observe(camera_scene(0))
            gate.observe(n,s,timestamp=.1*i)
        for i in range(12):
            n,s=vision.observe(camera_scene(3))
            self.assertFalse(gate.observe(n,s,timestamp=10.+i*.5))
        for i in range(10):
            n,s=vision.observe(camera_scene(0))
            self.assertFalse(gate.observe(n,s,timestamp=17.+i*.5))

    def test_rgb_depth_async_pairing_is_speed_bounded(self):
        self.assertTrue(aligned_depth_rgb(20.,20.7,0.,1.1))
        self.assertFalse(aligned_depth_rgb(20.,20.7,10.,1.1))
        self.assertFalse(aligned_depth_rgb(20.,22.,0.,1.1))
        self.assertFalse(aligned_depth_rgb(0.,22.,0.,1.1))
