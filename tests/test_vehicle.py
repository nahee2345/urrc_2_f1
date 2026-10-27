import json
import math
from pathlib import Path
import unittest
from xml.etree import ElementTree as ET
from dataclasses import replace
from vehicle_generator.model import Vehicle, WHEELS, generate, manifest, LIMITS
from vehicle_generator.control import setpoints
from scripts.check_upstream import contract_check

ROOT=Path(__file__).resolve().parents[1]

class Stage1Tests(unittest.TestCase):
    def test_three_drives_xml_physics(self):
        for drive,expected in (("fwd",2),("rwd",2),("awd",4)):
            with self.subTest(drive=drive):
                v=Vehicle(name=f"stage1_{drive}",drive=drive)
                root=ET.fromstring(generate(v))
                self.assertEqual(root.attrib["version"],"1.9")
                m=root.find("model")
                self.assertEqual(m.findtext("static"),"false")
                self.assertEqual(m.attrib["canonical_link"],"base_link")
                self.assertEqual(len(m.findall("link")),7)
                self.assertEqual(len(m.findall("joint")),6)
                self.assertEqual(len(m.findall(".//link/collision")),5)
                motors=[p for p in m.findall("plugin") if p.findtext("use_force_commands")=="true"]
                self.assertEqual(len(motors),expected)
                self.assertTrue(all(p.findtext("cmd_max")==str(v.motor_max_torque) for p in motors))
                self.assertEqual(len([p for p in m.findall("plugin") if "joint-position-controller" in p.attrib.get("filename","")]),2)
                for joint in m.findall("joint"):
                    self.assertNotEqual(joint.findtext("parent"),joint.findtext("child"))
                    self.assertEqual(joint.attrib["type"],"revolute")
                for link in m.findall("link"):
                    self.assertGreater(float(link.findtext("inertial/mass")),0)
                    for field in ("ixx","iyy","izz"):
                        self.assertGreater(float(link.findtext(f"inertial/inertia/{field}")),0)

    def test_size_margin_and_spawn(self):
        v=Vehicle()
        m=manifest(v)
        self.assertEqual(m["spawn_z_offset_m"],v.wheel_radius)
        self.assertTrue(all(a < b for a,b in zip(v.envelope(),LIMITS)))
        self.assertAlmostEqual(m["mass_kg"],1.74)

    def test_size_limit_failure(self):
        for changes in (dict(body_length=0.56),dict(wheel_track=0.22),dict(body_height=0.25),dict(max_steer_deg=36),dict(wheel_radius=-1),dict(drive="teleport"),dict(name="no bad")):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                generate(replace(Vehicle(),**changes))

    def test_source_contract_mock_fixture(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            (p/"race").mkdir()
            (p/"src/urrc_track_gazebo/worlds").mkdir(parents=True)
            (p/"src/urrc_track_gazebo/tracks/processed").mkdir(parents=True)
            (p/"release.json").write_text(json.dumps({"role":"competition","vehicle_max_m":[0.55,0.25,0.18],"contract_version":2}))
            (p/"race/event_config.json").write_text(json.dumps({"light_count":5,"light_interval_seconds":1.0,"qualifying_laps":2,"final_laps":10,"checkpoint_spacing_m":10.0,"track_pool":["monza","spa","monaco","cota","hungaroring","baku"]}))
            for t in ["monza","spa","monaco","cota","hungaroring","baku"]:
                (p/"src/urrc_track_gazebo/worlds"/f"{t}_race.sdf").touch()
                (p/"src/urrc_track_gazebo/tracks/processed"/f"{t}.json").touch()
            (p/"run_v10.sh").touch();(p/"test_v10.sh").touch()
            self.assertEqual(contract_check(p)[0]["contract_version"],2)

    def test_steering_race_right_sign_and_ackermann(self):
        v=Vehicle()
        neutral=setpoints(v,1.0,0)
        self.assertEqual(neutral["steer_left_rad"],0)
        self.assertAlmostEqual(neutral["wheel_radps"]["rear_left"],-25.)
        right=setpoints(v,1.0,0.35)
        self.assertLess(right["steer_right_rad"],right["steer_left_rad"])
        self.assertLess(right["steer_left_rad"],0)
        self.assertGreater(abs(right["wheel_radps"]["rear_left"]),abs(right["wheel_radps"]["rear_right"]))
        left=setpoints(v,1.0,-0.35)
        self.assertGreater(left["steer_left_rad"],left["steer_right_rad"])
        self.assertGreater(left["steer_right_rad"],0)
        self.assertGreater(abs(left["wheel_radps"]["rear_right"]),abs(left["wheel_radps"]["rear_left"]))
        clipped=setpoints(v,1.0,math.pi)
        self.assertLessEqual(abs(clipped["steer_right_rad"]),math.radians(35)+0.05)

    def test_invalid_commands(self):
        with self.assertRaises(ValueError):setpoints(Vehicle(),math.nan,0)
        with self.assertRaises(ValueError):setpoints(Vehicle(),1,math.inf)

if __name__=="__main__":unittest.main()
