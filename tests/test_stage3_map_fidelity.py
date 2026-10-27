"""공식 몬자 고저차와 3단계 복사본 보존 계약 검사."""
from pathlib import Path
import json
import tempfile
import unittest
from stage3.terrain import _vertices,_hash,check_fidelity

ROOT=Path(__file__).resolve().parents[1]
ORIGINAL=ROOT/"upstream/urrc_f1_ws/src/urrc_track_gazebo"

class SourceElevationTests(unittest.TestCase):
    def test_source_road_has_nonzero_elevation(self):
        report=ORIGINAL/"docs/monza_geometry_report.json"
        if not report.exists():self.skipTest("Git submodule 미설치")
        data=json.loads(report.read_text())
        self.assertGreater(data["elevation_range_m"],2.0)
        road=ORIGINAL/"models/urrc_monza/meshes/road.obj"
        m=_vertices(road)
        self.assertGreater(m["height_range_m"],2.0)
        self.assertGreater(m["vertices"],10000)

    def test_source_raw_sdf_uses_3d_road_and_red_lights(self):
        p=ORIGINAL/"worlds/monza_race.sdf"
        if not p.exists():self.skipTest("Git submodule 미설치")
        world=p.read_text()
        model=(ORIGINAL/"models/urrc_monza/model.sdf").read_text()
        self.assertIn("model://urrc_monza",world)
        self.assertIn("road_collision.obj",model)
        self.assertIn("road.obj",model)
        self.assertEqual(world.count('name="race_red_'),5)

    def test_sample_mesh_honors_z_range_and_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"road.obj"
            path.write_text("".join(f"v 0 1 {i*.01}\n" for i in range(200)))
            z=_vertices(path)
            self.assertAlmostEqual(z["height_range_m"],1.99)
            self.assertEqual(z["vertices"],200)
            self.assertEqual(_hash(path),_hash(path))
            path.write_text("".join(f"v 0 1 0\n" for i in range(200)))
            with self.assertRaises(ValueError):_vertices(path)

if __name__=="__main__":unittest.main()
