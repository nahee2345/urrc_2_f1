"""Stage 2 bridge and isolated sensor runtime fixture regression tests."""
from dataclasses import replace
import tempfile
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from vehicle_generator.model import Vehicle
from vehicle_generator.sensors import build,load_presets
from scripts.bridge_sensors import yaml_bridge
from scripts.stage2_sensor_smoke import isolated_world, has_sample


class SensorRuntimeContracts(unittest.TestCase):
    def test_bridge_messages_are_one_way_and_scoped(self):
        v=Vehicle(name="urrc_ai_05")
        _,meta=build(v,["rgb","lidar3d"])
        yaml=yaml_bridge(meta)
        self.assertEqual(yaml.count("direction: GZ_TO_ROS"),3)
        self.assertIn("sensor_msgs/msg/Image",yaml)
        self.assertIn("sensor_msgs/msg/LaserScan",yaml)
        self.assertNotIn("PointCloud2",yaml)
        opt=yaml_bridge(meta,include_points=True)
        self.assertIn("sensor_msgs/msg/PointCloud2",opt)
        self.assertIn("/model/urrc_ai_05/sensors/lidar3d/points",opt)

    def test_stage2_test_world_does_not_include_race(self):
        car=Vehicle(name="urrc_probe")
        tree=ET.fromstring(isolated_world(car,["rgb","depth","lidar3d"]))
        world=tree.find("world")
        self.assertEqual(world.attrib["name"],"urrc_stage1_smoke")
        self.assertIsNotNone(world.find("plugin[@name='gz::sim::systems::Sensors']"))
        self.assertIsNotNone(world.find("model[@name='sensor_obstacle']"))
        self.assertEqual(len(world.find("model[@name='urrc_probe']").findall(".//link/sensor")),4)
        self.assertEqual(len(world.findall("model")),3)

    def test_pointcloud_payload_validation(self):
        with tempfile.TemporaryDirectory() as folder:
            f=Path(folder)/"points.log"
            f.write_text("header { } width: 360 height: 16 data: \"abc\"")
            self.assertTrue(has_sample(f,"pointcloud"))
            f.write_text("width: 360 height: 16 no_data: \"foo\"")
            self.assertFalse(has_sample(f,"pointcloud"))

    def test_camera_payload_validation(self):
        with tempfile.TemporaryDirectory() as folder:
            f=Path(folder)/"image.log"
            f.write_text("width: 160 height: 120 data: \"abcdef\"")
            self.assertFalse(has_sample(f,"camera")) # require actual substantial transport output
            f.write_text("width: 160 height: 120 data: \""+"x"*200+"\"")
            self.assertTrue(has_sample(f,"camera"))

    def test_compact_profile_mass_and_dimensions(self):
        profiles=load_presets()
        for ai, p in profiles.items():
            v=replace(Vehicle(),name=ai.replace("-","_"),drive=p["drive"])
            _,manifest=build(v,p["driving"])
            self.assertLessEqual(manifest["envelope_m"]["length"],.55)
            self.assertLessEqual(manifest["envelope_m"]["width"],.25)
            self.assertLessEqual(manifest["envelope_m"]["height"],.18)
            self.assertLess(manifest["total_mass_kg"],2.2)


if __name__=="__main__":unittest.main()
