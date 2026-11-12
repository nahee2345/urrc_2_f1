#!/usr/bin/env python3
"""Static geometry and official contract verification, standard library only."""
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).parent
SDF = ROOT / "models" / "team11_yellow_car" / "model.sdf"

class ModelContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ET.parse(SDF).getroot().find("model")
        assert cls.model is not None

    def test_base_link_canonical_and_zero(self):
        self.assertEqual(self.model.attrib.get("canonical_link"), "base_link")
        base = self.model.find("link[@name='base_link']")
        self.assertIsNotNone(base)
        self.assertIsNone(base.find("pose"))
        self.assertEqual(float(base.findtext("inertial/mass")),1.6)

    def test_dimensions(self):
        size = [float(x) for x in self.model.find("link[@name='base_link']/collision/geometry/box/size").text.split()]
        self.assertEqual(size, [.53, .22, .11])
        # For wheels at (+/-0.18,+/-0.102,-.045) with radius .045,
        # axle width .046: overall +/- .125 width, +/- .09 Z.
        self.assertLessEqual(.53, .55)
        self.assertLessEqual(2*(.102 + .046/2), .25 + 1e-9)
        self.assertLessEqual(2*.09, .18 + 1e-9)

    def test_four_wheels_and_front_steering(self):
        joints = {n.get("name"):n for n in self.model.findall("joint")}
        self.assertEqual(len(joints),6)
        for side in ("left","right"):
            self.assertEqual(joints[f"front_{side}_steering_joint"].findtext("axis/limit/upper"),".5235987756")
            self.assertIn(f"rear_{side}_wheel_joint",joints)

    def test_independent_sensors(self):
        sensors = self.model.findall("link[@name='base_link']/sensor")
        self.assertEqual({x.findtext("topic") for x in sensors},{"/scan_11","/camera_11","/camera_zoom_11"})

    def test_forward_direction_markers_and_wheel_axes(self):
        base = self.model.find("link[@name='base_link']")
        self.assertIsNotNone(base.find("sensor[@name='team11_start_camera']"))
        visuals = {v.attrib["name"]:v for v in base.findall("visual")}
        self.assertAlmostEqual(float(visuals["front_marker_left"].findtext("pose").split()[0]), .263)
        self.assertAlmostEqual(float(visuals["rear_marker_left"].findtext("pose").split()[0]), -.263)
        for name in ("front_left","front_right","rear_left","rear_right"):
            wheel=self.model.find("link[@name='"+name+"_wheel']")
            self.assertAlmostEqual(float(wheel.findtext("pose").split()[3]), -1.5707963268)

    def test_wheels_support_15ms_target(self):
        radius=.045
        required_omega=15.0/radius
        for side in ("front_left","front_right","rear_left","rear_right"):
            joint=self.model.find(f"joint[@name='{side}_wheel_joint']")
            self.assertIsNotNone(joint)
            self.assertGreaterEqual(float(joint.findtext("axis/limit/velocity")),required_omega)

    def test_ackermann_plugin(self):
        plugin=self.model.find("plugin")
        self.assertEqual(plugin.get("filename"),"gz-sim-ackermann-steering-system")
        self.assertEqual(plugin.findtext("topic"),"/model/team_11/cmd_vel_11")

if __name__ == "__main__":
    unittest.main()
