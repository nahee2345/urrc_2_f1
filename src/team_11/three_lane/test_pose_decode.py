"""Pose extractor tests for private TEAM11 Gazebo PosePublisher bridge."""
import pathlib,sys,unittest
from types import SimpleNamespace as N
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from team11_race.pose_decode import extract_vehicle_pose,pose_frame_examples


def item(child,base="world",xyz=(.378,-2.056,1.3)):
    return N(child_frame_id=child,
             header=N(frame_id=base),
             transform=N(translation=N(x=xyz[0],y=xyz[1],z=xyz[2]),
                         rotation=N(w=.75,x=0.,y=0.,z=.66)))


class PoseDecodeTests(unittest.TestCase):
    def test_prefers_model_world_frame(self):
        msg=N(transforms=[item("team_11::base_link",xyz=(0,0,.05)),
                          item("team_11",xyz=(.378,-2.056,1.3))])
        pose=extract_vehicle_pose(msg)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose[0],.378)
        self.assertEqual(pose[-1],"team_11")

    def test_accepts_scoped_world_base(self):
        msg=N(transforms=[item("team_11::base_link")])
        self.assertEqual(extract_vehicle_pose(msg)[:3],(.378,-2.056,1.3))

    def test_accepts_world_base_without_model_scope(self):
        msg=N(transforms=[item("base_link")])
        self.assertIsNotNone(extract_vehicle_pose(msg))

    def test_private_single_world_transform_without_name(self):
        self.assertEqual(extract_vehicle_pose(N(transforms=[item("",xyz=(.378,-2.056,1.3))]))[:3],
                         (.378,-2.056,1.3))

    def test_rejects_other_team_pose(self):
        msg=N(transforms=[item("team_07"),item("team_07::base_link",base="team_07")])
        self.assertIsNone(extract_vehicle_pose(msg))

    def test_unmatched_frames_exposed_for_diagnostics(self):
        msg=N(transforms=[item("foo",base="bar")])
        self.assertEqual(pose_frame_examples(msg),[("bar","foo")])


if __name__=="__main__":
    unittest.main()
