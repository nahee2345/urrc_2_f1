"""Gazebo SDF hardening checks, independent of Gazebo runtime."""
from dataclasses import replace
import unittest
from xml.etree import ElementTree as ET
from vehicle_generator.model import Vehicle, generate
from scripts.stage1_smoke import make_smoke_world, model_pose_stream


class SDFSemantics(unittest.TestCase):
    def test_plugin_system_class_names(self):
        for drive in ("fwd", "rwd", "awd"):
            with self.subTest(drive=drive):
                model = ET.fromstring(generate(Vehicle(name=f"example_{drive}",drive=drive))).find("model")
                for plugin in model.findall("plugin"):
                    self.assertTrue(plugin.attrib["name"].startswith("gz::sim::systems::"))
                drives = [p for p in model.findall("plugin") if p.attrib["name"]=="gz::sim::systems::JointController"]
                steers = [p for p in model.findall("plugin") if p.attrib["name"]=="gz::sim::systems::JointPositionController"]
                self.assertEqual(len(drives),4 if drive=="awd" else 2)
                self.assertEqual(len(steers),2)

    def test_canonical_link_is_model_attribute(self):
        model = ET.fromstring(generate(Vehicle())).find("model")
        self.assertEqual(model.attrib["canonical_link"],"base_link")
        self.assertIsNone(model.find("canonical_link"))
        self.assertEqual(model.find("link").attrib["name"],"base_link")

    def test_joint_frames_and_link_references(self):
        model = ET.fromstring(generate(Vehicle())).find("model")
        links = {link.attrib["name"] for link in model.findall("link")}
        for joint in model.findall("joint"):
            self.assertIn(joint.findtext("parent"),links)
            self.assertIn(joint.findtext("child"),links)
            self.assertEqual(joint.find("pose").attrib["relative_to"],joint.findtext("child"))
            self.assertEqual(joint.find("axis/xyz").attrib["expressed_in"],"__model__")

    def test_independent_ground_world_and_vehicle(self):
        world = ET.fromstring(make_smoke_world(Vehicle())).find("world")
        self.assertEqual(world.attrib["name"],"urrc_stage1_smoke")
        self.assertEqual(len(world.findall("model")),2)
        self.assertEqual(world.find("model[@name='stage1_ground']").findtext("static"),"true")
        car = world.find("model[@name='urrc_stage1_car']")
        self.assertEqual(car.findtext("static"),"false")
        self.assertEqual(car.attrib["canonical_link"],"base_link")
        self.assertAlmostEqual(float(car.findtext("pose").split()[2]),.042)

    def test_pose_telemetry_parser(self):
        data='''pose {
  name: "stage1_ground"
  position { x: 0 y: 0 z: 0 }
}
pose {
  name: "urrc_stage1_car"
  position { x: 0.1 y: 0.0 z: 0.04 }
  orientation { w: 1 }
}
pose {
  name: "urrc_stage1_car"
  position { x: 0.4 y: 0.1 z: 0.04 }
  orientation { w: 1 }
}
'''
        self.assertEqual(model_pose_stream(data,"urrc_stage1_car"),[(.1,0.0,.04),(.4,.1,.04)])

    def test_invalid_geometry_cannot_enter_gazebo(self):
        with self.assertRaises(ValueError):
            make_smoke_world(replace(Vehicle(),wheel_radius=0.1))


if __name__=="__main__":
    unittest.main()
