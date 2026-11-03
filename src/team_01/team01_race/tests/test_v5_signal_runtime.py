"""Regression fixtures for the live failure reported in v4 console logs."""
from pathlib import Path
import sys
import unittest
import numpy as np
import cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.signal_vision import SignalVision
from team01_race.planning import TrafficGate, SafetyPlanner, estimate_road_corridor
from test_offline import CFG


def camera_scene(lit=0, exposure=1.0, wall_red=True):
    a=np.empty((360,640,3),np.uint8);a[:]=(55,56,56)
    # Nine stationary bright red objects outside the start gantry.
    if wall_red:
        for k in range(9):
            cv2.rectangle(a,(20+68*k,285),(28+68*k,302),(205,20,20),-1)
    # Five dark lens faces remain visible after the starter turns out.
    for k in range(5):
        x=242+39*k
        cv2.circle(a,(x,165),12,(101,19,20),-1)
        if k<lit:
            cv2.circle(a,(x,165),12,(252,9,8),-1)
    return np.clip(a.astype(float)*exposure,0,255).astype(np.uint8)


class V5SignalTests(unittest.TestCase):
    def test_never_arm_on_nine_static_red_objects(self):
        vision=SignalVision()
        gate=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        for i in range(110):
            n,s=vision.observe(camera_scene())
            self.assertFalse(gate.observe(n,s,timestamp=.05*i))
        self.assertTrue(vision.ready)
        self.assertEqual(vision.peak_blobs,0)

    def test_sequential_lights_four_then_all_out(self):
        vision=SignalVision()
        gate=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        seq=[(0,.05*i) for i in range(35)]
        seq += [(i+1,2.0+i*.65+j*.05) for i in range(4) for j in range(3)]
        seq += [(0,5.0+i*.05) for i in range(4)]
        last=False
        for n,t in seq:
            detected,strength=vision.observe(camera_scene(n))
            last=gate.observe(detected,strength,timestamp=t)
            if t<5.:
                self.assertFalse(last)
        self.assertTrue(last)
        self.assertEqual(vision.peak_blobs,4)
        self.assertTrue(gate.lights_out)

    def test_four_first_visible_lights_cleared_after_short_delay(self):
        v=SignalVision();g=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        for i in range(25):
            k,s=v.observe(camera_scene(0));g.observe(k,s,timestamp=i*.05)
        for t in (4.0,4.1,4.2,4.3,4.4):
            k,s=v.observe(camera_scene(4));g.observe(k,s,timestamp=t)
        for t in (4.9,5.0,5.1,5.2):
            k,s=v.observe(camera_scene(0));g.observe(k,s,timestamp=t)
        self.assertTrue(g.lights_out)

    def test_short_4_lamp_flicker_is_not_launch(self):
        v=SignalVision();g=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        for i in range(25):
            k,s=v.observe(camera_scene(0));g.observe(k,s,timestamp=i*.05)
        for t in (4.0,4.1):
            k,s=v.observe(camera_scene(4));g.observe(k,s,timestamp=t)
        for t in (4.2,4.3,4.4,4.5):
            k,s=v.observe(camera_scene(0));self.assertFalse(g.observe(k,s,timestamp=t))

    def test_static_background_brightness_shift_does_not_arm(self):
        v=SignalVision();g=TrafficGate(required=4,on_frames=2,off_frames=3,min_sequence_s=3.)
        for i in range(25):
            k,s=v.observe(camera_scene(0));g.observe(k,s,timestamp=i*.05)
        for i in range(35):
            k,s=v.observe(camera_scene(0,exposure=.94));self.assertFalse(g.observe(k,s,timestamp=2+i*.05))

    def test_lidar_fallback_only_with_two_consistent_boundaries(self):
        cfg={**CFG,'require_visual_road':True,'permit_lidar_verified_fallback':True,'unmapped_speed_mps':.75}
        p=SafetyPlanner(cfg)
        one_wall=np.column_stack((np.arange(.7,7.,.08),np.ones(len(np.arange(.7,7.,.08)))))
        self.assertEqual(p.step(np.empty((0,2)),one_wall,5.,.1,True,True,True,None).speed,0.)
        two_walls=np.vstack((one_wall,np.column_stack((one_wall[:,0],-one_wall[:,1]))))
        m=p.step(np.empty((0,2)),two_walls,5.,.1,True,True,True,None)
        self.assertGreater(m.speed,0.)
        self.assertLessEqual(m.speed,.75)
        self.assertEqual(p.road_source,'lidar')

if __name__=='__main__':unittest.main()
