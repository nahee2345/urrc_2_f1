"""Offline tests for bundled six-map TEAM 11 practice route data."""
import json
from pathlib import Path
import tempfile
import unittest

from core import TRACKS
from map_library import (UnknownTrack, build_route_bundle, detect_track,
                         map_directory, normalize_track, track_from_spawn)


class TrackBundleTests(unittest.TestCase):
    def test_six_tracks(self):
        self.assertEqual(set(TRACKS), {
            "monza", "spa", "monaco", "cota", "hungaroring", "baku"})
        for name in TRACKS:
            self.assertTrue((map_directory(name) / "centerline.csv").is_file())
            meta = json.loads((map_directory(name) / "grid.json").read_text())
            self.assertEqual(meta["track"], name)
            self.assertGreaterEqual(len(meta["grid"]), 12)

    def test_world_names(self):
        for name in TRACKS:
            self.assertEqual(normalize_track(name + "_race.sdf"), name)
            self.assertEqual(normalize_track(name + "_grid.sdf"), name)
            self.assertEqual(detect_track(world_name=name + "_race",
                                          state_file="/no/such/run.json"), name)
        with self.assertRaises(UnknownTrack):
            detect_track(world_name="unknown_track")

    def test_missing_map_fails_closed(self):
        with self.assertRaises(UnknownTrack):
            detect_track(state_file="/no/such/run.json")
        with self.assertRaises(UnknownTrack):
            detect_track(explicit="monza", world_name="spa")

    def test_event_state(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = Path(tmp) / "last_run.json"
            manifest.write_text('{"track":"baku"}')
            self.assertEqual(detect_track(state_file=manifest), "baku")

    def test_spawn_matches_exact_known_grid(self):
        for name in TRACKS:
            point = json.loads((map_directory(name) / "grid.json").read_text())["grid"][0]
            detected = track_from_spawn(point["x"], point["y"],
                                        point["road_z"], point["yaw"])
            self.assertEqual(detected, name)

    def test_generate_all_routes(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name in TRACKS:
                result = build_route_bundle(name, tmp)
                directory = Path(result["route_prefix"])
                self.assertEqual(result["track"], name)
                self.assertGreater(result["lap_length_m"], 100)
                self.assertGreater(result["lane_spacing_m"], 0.25)
                self.assertGreater(result["sample_count"], 500)
                self.assertEqual(result["grid_count"], 20)
                for index in range(3):
                    lines = (directory / f"lane_{index}.csv").read_text().splitlines()
                    self.assertEqual(lines[0], "s_m,x_m,y_m,z_m,yaw_rad")
                    self.assertEqual(len(lines), result["sample_count"] + 1)


if __name__ == "__main__":
    unittest.main()
