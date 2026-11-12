"""Offline regression for the official GitHub track mesh replay (no network)."""
from __future__ import annotations

import json
import pathlib
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

TEAM=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(TEAM/"sim_test"))
from generate_official_world import build_official_world
from official_maps import OFFICIAL_REPO,PACKAGE_SUBDIR


def make_mock_package(root,track):
    package=root/"src"/"urrc_track_gazebo"
    (package/"worlds").mkdir(parents=True)
    mesh=package/"models"/f"urrc_{track}"/"meshes"
    mesh.mkdir(parents=True)
    (mesh/"road.obj").write_text("o mock\n")
    (package/"models"/f"urrc_{track}"/"model.sdf").write_text(
        f'<sdf version="1.9"><model name="urrc_{track}">'
        f'<static>true</static><link name="track"><visual name="road">'
        f'<geometry><mesh><uri>model://urrc_{track}/meshes/road.obj</uri>'
        f'</mesh></geometry></visual></link></model></sdf>')
    (package/"worlds"/f"{track}_race.sdf").write_text(
        f'<sdf version="1.9"><world name="{track}">'
        f'<plugin filename="gz-sim-physics-system" '
        f'name="gz::sim::systems::Physics"/>'
        f'<model name="race_start_gantry"><static>true</static>'
        f'<pose>0 0 1.2 0 0 1.45</pose><link name="gantry_visuals">'
        f'<visual name="red_lens_1"/></link></model>'
        f'<include><uri>model://urrc_{track}</uri><name>urrc_{track}</name></include>'
        f'</world></sdf>')
    return package


class OfficialWorldTests(unittest.TestCase):
    def test_upstream_repo_is_only_source(self):
        self.assertEqual(OFFICIAL_REPO,
                         "https://github.com/nahee2345/urrc_f1_ws.git")
        self.assertEqual(PACKAGE_SUBDIR.as_posix(),"src/urrc_track_gazebo")

    def test_all_six_official_worlds_preserve_gantry_mesh_and_vehicle(self):
        for track in ("monza","spa","monaco","cota","hungaroring","baku"):
            with self.subTest(track=track), tempfile.TemporaryDirectory() as tmp:
                tmp=pathlib.Path(tmp)
                pkg=make_mock_package(tmp,track)
                src=(pkg/"worlds"/f"{track}_race.sdf").read_bytes()
                output=tmp/"generated.sdf"
                report=build_official_world(track,11,pkg,output)
                self.assertTrue(output.exists())
                self.assertEqual(report["official_mesh_model"],
                                 f"model://urrc_{track}")
                self.assertEqual(
                    (pkg/"worlds"/f"{track}_race.sdf").read_bytes(),src,
                    "Official source must remain unmodified")
                world=ET.parse(output).getroot().find("world")
                self.assertIsNotNone(
                    world.find(f"include[uri='model://urrc_{track}']"))
                self.assertIsNotNone(
                    world.find("model[@name='race_start_gantry']"))
                self.assertIsNone(world.find("model[@name='team11_csv_test_road']"))
                self.assertIsNotNone(world.find("model[@name='team_11']"))
                lights=world.find("model[@name='team11_test_lights']")
                self.assertEqual(len(lights.findall("link/visual")),5)
                self.assertEqual(lights.findtext("static"),"true")
                slot=json.loads((TEAM/"maps"/track/"track.json").read_text())[
                    "grid"][10]
                xyz=[float(x) for x in
                     world.find("model[@name='team_11']").findtext(
                         "pose").split()[:3]]
                self.assertAlmostEqual(xyz[0],slot["x"],places=5)
                self.assertAlmostEqual(xyz[1],slot["y"],places=5)

    def test_missing_required_official_mesh_is_not_silently_ignored(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp=pathlib.Path(tmp)
            pkg=make_mock_package(tmp,"monza")
            (pkg/"models/urrc_monza/meshes/road.obj").unlink()
            with self.assertRaises(FileNotFoundError):
                build_official_world("monza",1,pkg,tmp/"test.sdf")

    def test_default_runner_loads_upstream_not_synthetic(self):
        source=(TEAM/"sim_test/run_replay.py").read_text()
        self.assertIn("sync_official_map(",source)
        self.assertIn("build_official_world(",source)
        self.assertIn('if args.csv_test:',source)
        self.assertIn('"GZ_SIM_RESOURCE_PATH"',source)
        self.assertIn('f"{args.track}_grid.gui.config"',source)
        self.assertIn('ap.add_argument("--refresh-maps"',source)


if __name__=="__main__":
    unittest.main()
