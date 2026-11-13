"""Route geometry regression tests. Run python3 -m unittest discover -s src/team_11 -p 'test_*.py'."""
import math
import unittest
from routes import generate

class RouteTests(unittest.TestCase):
    def setUp(self):
        self.track = [
            {"x_m": float(i), "y_m": 0.0, "z_m": 0.2, "yaw_rad": 0.0, "road_width_m": 2.4}
            for i in range(5)
        ]

    def test_right_center_left(self):
        paths, narrow = generate(self.track)
        self.assertEqual(narrow, 0)
        self.assertLess(paths[1][0]["y_m"], paths[2][0]["y_m"])
        self.assertLess(paths[2][0]["y_m"], paths[3][0]["y_m"])
        self.assertAlmostEqual(paths[1][0]["y_m"], -0.965)
        self.assertAlmostEqual(paths[3][0]["y_m"], 0.965)

    def test_wall_clearance(self):
        paths, _ = generate(self.track)
        for lane in (1, 2, 3):
            for p in paths[lane]:
                self.assertGreaterEqual(
                    p["road_width_m"]/2 - abs(p["lane_offset_m"]) - 0.125,
                    0.11 - 1e-9)

    def test_legacy_constant_route_spacing_when_road_width_changes(self):
        changing=[dict(p,road_width_m=w) for p,w in
                  zip(self.track,(2.4,3.0,4.0,2.8,2.6))]
        paths,_=generate(changing)
        for lane in (1,3):
            values=[p["lane_offset_m"] for p in paths[lane]]
            self.assertTrue(all(abs(v-values[0])<1e-10 for v in values))
        self.assertAlmostEqual(paths[1][0]["lane_offset_m"],-.965)
        self.assertAlmostEqual(paths[3][0]["lane_offset_m"],.965)

    def test_narrow_road_forbids_changes(self):
        data = [dict(p, road_width_m=0.7) for p in self.track]
        paths, narrow = generate(data)
        self.assertEqual(narrow, len(data))
        self.assertTrue(all(not p["safe_to_change_lane"] for p in paths[1]))

if __name__ == "__main__":
    unittest.main()
