"""Check TEAM11 base_link frame against the full 3D collision AABB.

Initial local-frame diagnostic used a +0.0615m reference. The complete
vehicle's collision AABB is -0.040m..+0.140m along model Z and its center
is +0.050m. This removes the remaining 0.0115m (1.15cm) discrepancy
without changing original car, wheel, sensor or center-of-mass positions.
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

def complete_collision_aabb(model):
    """Compute every link's collision extents in model coordinates.

    All links have zero Euler rotation in this car. Cylinder wheels have
    local 90-degree X rotation, so their Z extent is the tire radius.
    """
    minima=[float("inf")]*3
    maxima=[float("-inf")]*3
    for link in model.findall("link"):
        link_pose=pose(link)
        self_rot=link_pose[3:]
        if any(abs(v)>1e-8 for v in self_rot):
            raise ValueError("Unsupported rotated model link in AABB regression")
        for collision in link.findall("collision"):
            child=pose(collision)
            center=[link_pose[i]+child[i] for i in range(3)]
            geometry=collision.find("geometry")
            box=geometry.find("box") if geometry is not None else None
            cylinder=geometry.find("cylinder") if geometry is not None else None
            if box is not None:
                if any(abs(v)>1e-8 for v in child[3:]):
                    raise ValueError("Unexpected rotated box in TEAM11 geometry")
                size=[float(v) for v in box.findtext("size").split()]
                extent=[v/2 for v in size]
            elif cylinder is not None:
                radius=float(cylinder.findtext("radius"))
                length=float(cylinder.findtext("length"))
                roll,pitch,yaw=child[3:]
                if abs(pitch)>1e-8 or abs(yaw)>1e-8:
                    raise ValueError("Unexpected cylinder rotation in TEAM11 geometry")
                # Rotated circular cylinder (symmetry axis originally +Z).
                ux,uy,uz=0.,-math.sin(roll),math.cos(roll)
                extent=[abs(unit)*length/2 + radius*math.sqrt(max(0.,1-unit**2))
                        for unit in (ux,uy,uz)]
            else:
                raise ValueError("Unrecognized collision primitive")
            for axis in range(3):
                minima[axis]=min(minima[axis],center[axis]-extent[axis])
                maxima[axis]=max(maxima[axis],center[axis]+extent[axis])
    return minima,maxima


class VehicleBaseGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ET.parse(MODEL).getroot().find("model")
        cls.base = cls.model.find("link[@name='base_link']")
        assert cls.base is not None

    def test_base_link_origin_matches_full_collision_center(self):
        offset=pose(self.base)
        low,high=complete_collision_aabb(self.model)
        for axis in range(3):
            with self.subTest(axis=axis):
                centroid=(low[axis]+high[axis])/2
                self.assertAlmostEqual(centroid,offset[axis],places=7)
        self.assertAlmostEqual(low[2],-.04,places=7)
        self.assertAlmostEqual(high[2],.14,places=7)
        self.assertAlmostEqual(offset[2],.05,places=7)

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
