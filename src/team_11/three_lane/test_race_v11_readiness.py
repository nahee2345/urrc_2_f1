"""Guard the standalone team's race-v11 launch and startup integration."""
import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/"team11_race"

class RaceV11ReadinessTests(unittest.TestCase):
    def test_control_and_runtime_python_compile(self):
        for path in [PKG/"control.py",PKG/"adapter.py",
                     PKG/"pose_decode.py",PKG/"camera_start.py",
                     PKG/"start_policy.py",PKG/"perception.py",
                     PKG/"strategy.py",PKG/"planner.py",
                     ROOT/"launch/race.launch.py"]:
            with self.subTest(path=path.name):
                ast.parse(path.read_text(encoding="utf-8"),filename=str(path))

    def test_route_matching_first_and_camera_gate_retained(self):
        controller=(PKG/"control.py").read_text()
        self.assertLess(controller.index("result=match_map_from_spawn("),
                        controller.index("name=discover_world()"))
        self.assertIn('permission=choose_start(',controller)
        self.assertIn('self.allowed,self.camera_gate.started',controller)
        self.assertIn('self.perception.neighbor_motion(',controller)
        self.assertIn('RaceDirector()',controller)
        self.assertIn('LaserTracker()',controller)
        self.assertIn('pose_frames=',controller)
        self.assertIn('camera_frames=',controller)
        self.assertNotIn("/model/team_11/urrc_timing",controller)

    def test_private_model_world_pose_only(self):
        model=ET.parse(ROOT/"models/team11_car/model.sdf").getroot().find("model")
        plugins=[p for p in model.findall("plugin")
                 if p.get("filename")=="gz-sim-pose-publisher-system"]
        self.assertEqual(len(plugins),1)
        publisher=plugins[0]
        self.assertEqual(publisher.findtext("publish_model_pose"),"true")
        self.assertEqual(publisher.findtext("publish_link_pose"),"false")
        self.assertEqual(publisher.findtext("topic"),"/team11/pose_11")

    def test_external_start_contract(self):
        import json
        manifest=json.loads((ROOT/"race_entry.json").read_text())
        launch=(ROOT/"launch/race.launch.py").read_text()
        self.assertEqual(manifest["entry_id"],"team_11")
        self.assertIn("/race/start/team_11",launch)
        self.assertIn("/team11/pose_11",launch)
        self.assertIn("/team11/odom_11@nav_msgs/msg/Odometry[gz.msgs.Odometry",launch)
        self.assertIn('odom_world_aligned', (PKG/"control.py").read_text())
        self.assertEqual(manifest["vehicle_model_file"],
                         "src/team_11/models/team11_car/model.sdf")

if __name__=="__main__":
    unittest.main()
