"""TEAM11 portable submission tests; no ROS, Gazebo or external world files."""
import json
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from team11_race.planner import Planner,RouteMap
from team11_race.trackdetect import TRACKS,map_assets,matching_grid,detect_from_topic_list


class StandaloneSubmissionTests(unittest.TestCase):
    def setUp(self):
        self.package=ROOT

    def test_manifest_is_self_contained(self):
        root=json.loads((ROOT/"race_entry.json").read_text())
        self.assertTrue((self.package/"package.xml").is_file())
        self.assertTrue((self.package/"setup.py").is_file())
        self.assertFalse((self.package/"team11_race"/"package.xml").exists())
        self.assertEqual(root["entry_id"],"team_11")
        self.assertEqual(root["ros_package"],"team11_race")
        self.assertTrue((self.package/root["vehicle_model_file"].removeprefix("src/team_11/")).is_file())
        for key in ("speed_topic","steering_topic","lidar_topic"):
            self.assertTrue(root[key].endswith("_11"),root[key])

    def test_all_six_tracks_match_own_grid(self):
        for track in TRACKS:
            with self.subTest(track=track):
                assets=map_assets(self.package,track)
                route=RouteMap(assets)
                self.assertGreater(len(route.center),1000)
                self.assertTrue(all(len(route.routes[i])==len(route.center) for i in range(3)))
                grid=route.meta["grid"][0]
                pose=(grid["x"],grid["y"],grid["road_z"]+.04,grid["yaw"])
                self.assertEqual(matching_grid(assets,pose),1)
                s,_,_=route.nearest(*pose[:2])
                lane=min(range(3),key=lambda k:
                    math.hypot(route.sample(k,s)[0]-pose[0],route.sample(k,s)[1]-pose[1]))
                planner=Planner(route,lane,max_speed=8.0)
                speed,steer,status=planner.step(pose,.05)
                self.assertEqual(status,"DRIVE",(track,status))
                self.assertGreaterEqual(speed,0.)
                self.assertLessEqual(abs(steer),.52+1e-9)

    def test_auto_world_detection_and_ambiguous_world(self):
        self.assertEqual(detect_from_topic_list(
            "/clock\n/world/monaco/scene/info\n/world/monaco/pose/info"),"monaco")
        self.assertIsNone(detect_from_topic_list("/clock\n/model/team11/pose_11"))
        with self.assertRaises(ValueError):
            detect_from_topic_list("/world/monaco/scene/info\n/world/baku/scene/info")

    def test_private_vehicle_topics_end_in_11(self):
        root=ET.parse(self.package/"models/team11_car/model.sdf").getroot()
        topics=[e.text.strip() for e in root.findall(".//topic") if e.text]
        self.assertTrue(topics)
        for topic in topics:
            self.assertTrue(topic.startswith("/team11/"),topic)
            self.assertTrue(topic.endswith("_11"),topic)
        self.assertEqual(root.find("model").attrib["name"], "team_11")
        self.assertIn("/team11/pose_11",topics)
        self.assertIn("/team11/cmd_vel_11",topics)
        self.assertIn("/team11/start_image_11",topics)

    def test_no_external_source_dependency_in_runtime(self):
        for name in ("planner.py","trackdetect.py","control.py","adapter.py"):
            content=(self.package/"team11_race"/name).read_text()
            self.assertNotIn("urrc_2/",content)
            self.assertNotIn("urrc_f1_ws/",content)
        launch=(self.package/"launch/race.launch.py").read_text()
        self.assertIn("/race/start/team_11",launch)
        controller=(self.package/"team11_race/control.py").read_text()
        adapter=(self.package/"team11_race/adapter.py").read_text()
        self.assertIn('("race_start_topic","/race/start/team_11")',controller)
        self.assertIn('("race_start_topic","/race/start/team_11")',adapter)
        self.assertNotIn("gz sim",launch)
        self.assertNotIn("ros_gz_sim",launch)


if __name__=="__main__":
    unittest.main()
