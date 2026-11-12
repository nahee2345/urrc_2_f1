"""Offline SDF validation for standalone competition ROS/Gazebo replay."""
from __future__ import annotations
import pathlib
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

TEAM=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(TEAM/"sim_test"))
from generate_world import build_world

class StandaloneCsvWorldTests(unittest.TestCase):
    def test_run_shell_bash_syntax_and_ros_nounset_guard(self):
        path=TEAM/"sim_test/run.sh"
        result=subprocess.run(["bash","-n",str(path)],
                              capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        s=path.read_text()
        self.assertEqual(s.count("set +u"),2)
        self.assertEqual(s.count("set -u"),2)

    def test_generator_contains_actual_team11_vehicle_and_atomic_lamps(self):
        with tempfile.TemporaryDirectory() as tmp:
            output=pathlib.Path(tmp)/"monza_test.sdf"
            result=build_world("monza",1,output)
            self.assertGreater(result["road_patches"],200)
            root=ET.parse(output).getroot()
            world=root.find("world")
            self.assertEqual(world.attrib["name"],"monza")
            vehicle=world.find("model[@name='team_11']")
            self.assertIsNotNone(vehicle)
            self.assertEqual(len(vehicle.findall("link")),7)
            self.assertIsNotNone(vehicle.find(
                "link[@name='base_link']/sensor[@name='front_rgb']"))
            lights=world.find("model[@name='team11_test_lights']")
            self.assertEqual(len(lights.findall("link/visual")),5)
            self.assertEqual(lights.findtext("static"),"true")
            road=world.find("model[@name='team11_csv_test_road']")
            self.assertIsNotNone(road)
            self.assertEqual(len(road.findall("link")),1)
            self.assertGreater(len(road.findall("link/collision")),200)
            self.assertEqual(len(road.findall("link/visual")),
                             len(road.findall("link/collision")))
            names=[p.get("filename") for p in world.findall("plugin")]
            self.assertIn("gz-sim-sensors-system",names)
            self.assertIn("gz-sim-user-commands-system",names)

    def test_gui_and_server_are_independent_and_failures_are_printed(self):
        source=(TEAM/"sim_test/run_replay.py").read_text()
        self.assertIn('gz_cmd=["gz","sim","-s","-r","-v","4"]',source)
        self.assertIn('gui_cmd=["gz","sim","-g","-v","4"]',source)
        self.assertIn('gui,f=launch(gui_cmd,out/"gazebo_gui.log",env)',source)
        self.assertIn('GUI exited (code=',source)
        self.assertIn('print_log_tail(out/"gazebo_gui.log")',source)
        self.assertIn('default=180',source)
        self.assertIn('os.environ.get("DISPLAY")',source)
        self.assertIn('os.environ.get("WAYLAND_DISPLAY")',source)
        self.assertIn('for p in (gate,clock,bridge,gui,gz)',source)

    def test_replay_grids_1_and_11_use_bundled_track_coordinates(self):
        import json
        for track in ("monza","spa","monaco","cota","hungaroring","baku"):
            with tempfile.TemporaryDirectory() as tmp:
                root=TEAM/"maps"/track
                grids=json.loads((root/"track.json").read_text())["grid"]
                for grid in (1,11):
                    with self.subTest(track=track,grid=grid):
                        output=pathlib.Path(tmp)/f"{grid}.sdf"
                        build_world(track,grid,output,road_stride=20)
                        model=ET.parse(output).getroot().find(
                            "world/model[@name='team_11']")
                        numbers=[float(v) for v in model.findtext("pose").split()]
                        self.assertAlmostEqual(numbers[0],grids[grid-1]["x"],6)
                        self.assertAlmostEqual(numbers[1],grids[grid-1]["y"],6)
                        self.assertAlmostEqual(
                            numbers[2],grids[grid-1]["road_z"]+.04,6)

if __name__=="__main__":
    unittest.main()
