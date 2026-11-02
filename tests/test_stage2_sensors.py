"""Stage 2 static sensor generation acceptance tests; no fake sensor frames."""
from dataclasses import replace
from pathlib import Path
import json
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET

from vehicle_generator.model import Vehicle, LIMITS
from vehicle_generator.sensors import SENSOR_SPECS, build, bounding, load_presets, sensors_for

class SensorProfiles(unittest.TestCase):
    def test_all_ten_profiles_build(self):
        profiles=load_presets()
        for ai,plan in profiles.items():
            with self.subTest(ai=ai):
                car=replace(Vehicle(), name="urrc_"+ai.lower().replace("-","_"),drive=plan["drive"])
                xml,info=build(car,plan["driving"])
                mod=ET.fromstring(xml).find("model")
                self.assertEqual(mod.findtext("static"),"false")
                self.assertEqual(len(mod.findall("joint")),6)
                nodes=mod.findall("link[@name='base_link']/sensor")
                self.assertEqual(len(nodes),info["sensor_count"])
                self.assertEqual(len(mod.findall(".//link/collision")),5+len(nodes))
                self.assertEqual(len(mod.findall("link")),7)
                self.assertEqual(len(info["topics"]),len(nodes))
                self.assertEqual(len({x.findtext("topic") for x in nodes}),len(nodes))
                self.assertEqual(nodes[0].attrib["name"],"start_camera")
                for a,b in zip(info["envelope_m"].values(),LIMITS):
                    self.assertLessEqual(a,b)
                self.assertAlmostEqual(info["total_mass_kg"],car.mass+info["sensor_mass_kg"],places=5)
                self.assertGreater(info["total_mass_kg"],car.mass)

    def test_start_camera_is_not_driving_sensor(self):
        plans=load_presets()
        self.assertEqual(plans["AI-01"]["driving"],["rgb"])
        self.assertEqual(plans["AI-02"]["driving"],["lidar3d"])
        for c in plans.values():
            self.assertNotIn("start_camera",c["driving"])
            self.assertEqual(sensors_for(c["driving"]).count("start_camera"),1)

    def test_stereo_has_two_independent_eyes(self):
        car=Vehicle(name="ai03")
        xml,info=build(car,["stereo"])
        sensors={s.attrib["name"]:s for s in ET.fromstring(xml).findall(".//link/sensor")}
        self.assertEqual(set(sensors),{"start_camera","stereo_left","stereo_right"})
        self.assertGreater(float(sensors["stereo_left"].findtext("pose").split()[1]),0)
        self.assertLess(float(sensors["stereo_right"].findtext("pose").split()[1]),0)
        self.assertEqual(sensors["stereo_left"].attrib["type"],"camera")
        self.assertEqual(sensors["stereo_right"].attrib["type"],"camera")

    def test_depth_and_lidar_are_real_sdf_sensor_types(self):
        xml,_=build(Vehicle(),["depth","lidar2d","lidar3d"])
        sensors={s.attrib["name"]:s for s in ET.fromstring(xml).findall(".//link/sensor")}
        self.assertEqual(sensors["depth"].attrib["type"],"depth_camera")
        self.assertEqual(sensors["depth"].findtext("camera/image/format"),"R_FLOAT32")
        self.assertEqual(sensors["lidar2d"].findtext("ray/scan/vertical/samples"),"1")
        self.assertEqual(sensors["lidar3d"].findtext("ray/scan/vertical/samples"),"16")
        self.assertEqual(sensors["lidar3d"].attrib["type"],"gpu_lidar")

    def test_combined_mass_inertia_and_center_of_mass(self):
        car=Vehicle()
        _,info=build(car,["rgb","depth","lidar3d"])
        root=ET.fromstring(build(car,["rgb","depth","lidar3d"])[0])
        iner=root.find(".//link[@name='base_link']/inertial")
        self.assertAlmostEqual(float(iner.findtext("mass")),
                               car.body_mass+info["sensor_mass_kg"],places=6)
        self.assertNotEqual(iner.findtext("pose"),"0 0 0 0 0 0")
        for a in ("ixx","iyy","izz"):
            self.assertGreater(float(iner.findtext("inertia/"+a)),0)
        self.assertGreater(info["total_mass_kg"],car.mass)

    def test_bad_sensor_specs_rejected(self):
        for bad in ([],["bogus"],["rgb","rgb"],["stereo","stereo_left"],["start_camera"]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                sensors_for(bad)

    def test_config_requires_exactly_ten(self):
        with tempfile.TemporaryDirectory() as t:
            path=Path(t)/"config.json"
            path.write_text(json.dumps({"AI-01":{"drive":"rwd","driving":["rgb"]}}))
            with self.assertRaises(ValueError):
                load_presets(path)

    def test_sensor_dimensions_and_topics_are_unique(self):
        for mode in (["rgb"],["lidar3d"],["stereo","lidar3d"],["rgb","depth","lidar3d"]):
            v=Vehicle()
            xml,info=build(v,mode)
            roots=ET.fromstring(xml).findall(".//link/sensor")
            self.assertEqual(len(roots),1+sum(2 if k=="stereo" else 1 for k in mode))
            self.assertTrue(all(s.findtext("always_on")=="true" for s in roots))
            self.assertTrue(all(s.findtext("update_rate") is not None for s in roots))
            self.assertEqual(len(set(info["topics"])),len(info["topics"]))
            self.assertTrue(all(s.findtext("topic").startswith("/model/"+v.name+"/")
                                for s in roots))


if __name__=="__main__":unittest.main()
