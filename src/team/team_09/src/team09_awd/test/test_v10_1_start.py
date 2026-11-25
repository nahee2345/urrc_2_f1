import unittest
from pathlib import Path
import numpy as np
from team09_awd.start_signal import StartSignal

def frame(count=0,black=False):
 a=np.full((480,640,3),25,np.uint8)
 for i in range(5):a[210:225,240+i*30:254+i*30]=(0,0,255) if i<count else (3,3,30)
 return np.zeros_like(a) if black else a

class SignalTest(unittest.TestCase):
 def five(self,d):
  for i in range(3):d.update(frame(5),i*.033)
  self.assertEqual(d.state,'FIVE_RED')
 def test_dark_initial_never_go(self):
  d=StartSignal()
  for i in range(20):d.update(frame(),i*.033)
  self.assertEqual(d.state,'WAIT_RED')
 def test_sequential_then_all_off(self):
  d=StartSignal();t=0
  for count in range(1,6):
   for _ in range(5):t+=.033;d.update(frame(count),t)
  self.assertEqual(d.state,'FIVE_RED')
  for _ in range(3):t+=.033;d.update(frame(),t)
  self.assertEqual(d.state,'GO')
 def test_partial_off_not_go(self):
  d=StartSignal();self.five(d)
  for i in range(4):d.update(frame(1),.1+i*.033)
  self.assertNotEqual(d.state,'GO')
 def test_blackout_not_go(self):
  d=StartSignal();self.five(d)
  for i in range(5):d.update(frame(black=True),.1+i*.033)
  self.assertNotEqual(d.state,'GO')
 def test_stale_gap_resets(self):
  d=StartSignal();self.five(d);d.update(frame(),1.)
  self.assertEqual(d.state,'WAIT_RED')
 def test_duplicate_frame_not_counted(self):
  d=StartSignal();self.five(d)
  for i in range(10):d.update(frame(),.1)
  self.assertNotEqual(d.state,'GO')
 def test_wrong_color(self):
  d=StartSignal();a=frame(5);a=a[:,:,[2,1,0]].copy()
  for i in range(10):d.update(a,i*.033)
  self.assertEqual(d.state,'WAIT_RED')
 def test_gantry_world_and_camera_wired(self):
  root=Path(__file__).resolve().parents[1]
  launch=(root/'launch/experiment.launch.py').read_text()
  self.assertIn('monza_race.sdf',launch)
  self.assertIn("executable='visual_start'",launch)
  self.assertIn('/urrc_awd/start_camera/image@sensor_msgs/msg/Image',launch)
