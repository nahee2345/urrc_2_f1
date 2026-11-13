import json
import math
import tempfile
import unittest
from pathlib import Path
from team_11.practice_frame import local_to_world, read_spawn, wrap_angle

class PracticeFrameTests(unittest.TestCase):
    def test_spawn_origin_is_world_grid(self):
        self.assertEqual(local_to_world(0,0,120,-15,math.pi/2),(120,-15))

    def test_rotated_local_forward_and_left(self):
        x,y=local_to_world(2,1,10,20,math.pi/2)
        self.assertAlmostEqual(x,9)
        self.assertAlmostEqual(y,22)

    def test_identity_transform(self):
        x,y=local_to_world(1.3,.2,0,0,0)
        self.assertAlmostEqual(x,1.3)
        self.assertAlmostEqual(y,.2)

    def test_heading_wrap(self):
        self.assertAlmostEqual(wrap_angle(2*math.pi+.3),.3)

    def test_official_grid_file_read_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            package=Path(tmp)/"src"/"urrc_track_gazebo"/"tracks"/"processed"
            package.mkdir(parents=True)
            p=package/"monza.json"
            payload='{"grid":[{"position":1,"x":42.5,"y":-8.1,"yaw":1.57}]}'
            p.write_text(payload)
            self.assertEqual(read_spawn(Path(tmp),"monza",1),(42.5,-8.1,1.57))
            self.assertEqual(p.read_text(),payload)

if __name__=="__main__":
    unittest.main()
