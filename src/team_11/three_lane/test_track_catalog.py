"""Offline map integrity and track auto-identification tests."""
import tempfile
from pathlib import Path
import unittest

from track_catalog import TRACKS, choose, identify_spawn, metadata, validate_bundle


class MapCatalogTests(unittest.TestCase):
    def test_six_bundled_maps(self):
        self.assertEqual(set(TRACKS), {"monza", "spa", "monaco", "cota", "hungaroring", "baku"})
        for track in TRACKS:
            with self.subTest(track=track):
                result = validate_bundle(track)
                self.assertGreater(result["samples"], 1000)

    def test_explicit_world_name(self):
        for name in TRACKS:
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "world.sdf"
                path.write_text(f'<sdf version="1.9"><world name="{name}"/></sdf>')
                self.assertEqual(choose("auto", world_file=path), name)

    def test_spawn_matches_each_map(self):
        for name in TRACKS:
            grid = metadata(name)["grid"][0]
            with self.subTest(track=name):
                self.assertEqual(identify_spawn(grid["x"], grid["y"], grid["yaw"]), name)

    def test_fail_closed(self):
        with self.assertRaises(ValueError):
            choose("auto")
        with self.assertRaises(ValueError):
            identify_spawn(100000, 100000, 0)


if __name__ == "__main__":
    unittest.main()
