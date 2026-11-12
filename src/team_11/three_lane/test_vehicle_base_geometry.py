"""Regression: organizer race_v9 base_link collision AABB center mismatch.

The earlier organizer diagnostic used an offset of +0.05m.\nThe full 3D collision AABB spans Z=-0.04 to +0.14m,\nso its center is +0.05m, closing the 0.0115m mismatch.
Reframe only base_link by +0.05m; counter-translate all of its child
visual/collision/sensor poses and the inertial center-of-mass pose to keep
the same physical car geometry, wheel contacts and sensor world positions.
"""
from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET

MODEL = (Path(__file__).resolve().parents[1] /
         "models/team11_car/model.sdf")

# Baseline model-frame Z values before the base_link frame is recentered.
BASE_CHILD_Z = {
    "chassis_visual": .061,
    "chassis_collision": .061,
    "front_wing_visual": .037,
    "front_wing_collision": .037,
    "nose_visual": .101,
    "nose_collision": .101,
    "rear_wing_visual": .133,
    "rear_wing_collision": .133,
    "camera_housing_visual": .105,
    "camera_housing_collision": .105,
    "front_lidar_housing_visual": .109,
    "front_lidar_housing_collision": .109,
    "rear_lidar_housing_visual": .109,
    "rear_lidar_housing_collision": .109,
    "rear_pylon_r_visual": .107,
    "rear_pylon_r_collision": .107,
    "rear_pylon_l_visual": .107,
    "rear_pylon_l_collision": .107,
    "team11_imu": .075,
    "front_rgb": .108,
    "front_lidar": .109,
    "rear_lidar": .109,
}


def pose(element):
    values = element.findtext("pose", default="0 0 0 0 0 0")
    data = [float(x) for x in values.split()]
    if len(data) != 6 or not all(math.isfinite(x) for x in data):
        raise ValueError("invalid SDF pose")
    return data


class VehicleBaseGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ET.parse(MODEL).getroot().find("model")
        cls.base = cls.model.find("link[@name='base_link']")
        assert cls.base is not None

    def test_base_link_origin_matches_full_collision_center(self):
        offset = pose(self.base)
        for actual, expected in zip(offset, (0, 0, .05, 0, 0, 0)):
            self.assertAlmostEqual(actual, expected, places=7)
        # Organizer's reported collision AABB center minus new base origin.
        self.assertAlmostEqual(.05 - offset[2], 0., places=7)

    def test_all_base_children_retain_previous_model_frame_positions(self):
        origin = pose(self.base)
        found = {}
        for child in self.base:
            if child.tag in ("visual", "collision", "sensor"):
                name = child.attrib["name"]
                data = pose(child)
                found[name] = data[2] + origin[2]
        self.assertEqual(set(found), set(BASE_CHILD_Z))
        for name, original_z in BASE_CHILD_Z.items():
            with self.subTest(child=name):
                self.assertAlmostEqual(found[name], original_z, places=7)

    def test_center_of_mass_and_wheel_contact_geometry_unchanged(self):
        origin=pose(self.base)
        inertia=pose(self.base.find("inertial"))
        self.assertAlmostEqual(origin[2]+inertia[2], 0, places=7)
        wheel_links=[self.model.find(f"link[@name='{loc}_wheel']")
                     for loc in ("front_left", "front_right", "rear_left", "rear_right")]
        self.assertTrue(all(link is not None for link in wheel_links))
        for wheel in wheel_links:
            self.assertAlmostEqual(pose(wheel)[2], 0., places=7)
            self.assertIsNotNone(wheel.find("collision[@name='rubber_collision']"))
        self.assertEqual(len(self.model.findall("joint")), 6)

    def test_private_topics_unchanged(self):
        topics=[element.text.strip() for tag in ("topic", "odom_topic", "tf_topic")
                for element in self.model.findall(".//"+tag) if element.text]
        expected={
            "/team11/imu_11",
            "/team11/start_image_11",
            "/team11/front_scan_11",
            "/team11/rear_scan_11",
            "/team11/joint_states_11",
            "/team11/cmd_vel_11",
            "/team11/odom_11",
            "/team11/gz_tf_11",
            "/team11/pose_11",
        }
        self.assertEqual(set(topics), expected)
        for topic in topics:
            self.assertTrue(topic.endswith("_11"), topic)


if __name__ == "__main__":
    unittest.main()
