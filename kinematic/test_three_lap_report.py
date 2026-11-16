#!/usr/bin/env python3
"""Regression tests for race3 result extraction; does not require Gazebo."""
import tempfile
import unittest
from pathlib import Path
from three_lap_report import extract, append

class ThreeLapReportTests(unittest.TestCase):
    def make_case(self, gazebo_text, rc=0):
        temp = tempfile.TemporaryDirectory()
        base = Path(temp.name)
        prepared = base / "prepared"
        prepared.mkdir()
        (prepared / "battle_state.csv").write_text("time_s,team_s\n")
        (base / "gazebo.log").write_text(gazebo_text)
        case = base / "grid1.log"
        case.write_text("Log: " + str(prepared / "battle_state.csv") + "\n")
        return temp, case, rc

    def test_three_laps_rank_and_speeds(self):
        temp, case, rc = self.make_case(
            "[BATTLE] RACE3_RESULT PASS_3_LAPS rank=2/3 laps=3 "
            "team_peak_mps=15 team_average_mps=10.25 "
            "wall_margin_min_m=0.21 min_body_clearance_m=0.8\n"
            "[BATTLE] RESULT PASS_3_LAPS\n")
        try:
            row = extract(case, "monaco", "1", "3", rc)
            self.assertEqual(row["rank"], "2/3")
            self.assertEqual(row["team_peak_mps"], "15")
            self.assertEqual(row["team_average_mps"], "10.25")
            self.assertEqual(row["wall_boundary_status"], "CLEAR_GEOMETRIC")
        finally:
            temp.cleanup()

    def test_geometric_wall_crossing(self):
        temp, case, rc = self.make_case(
            "[BATTLE] WALL_BOUNDARY t=10 vehicle=team_11 margin=-0.05\n"
            "[BATTLE] RACE3_RESULT FAIL_WALL_BOUNDARY rank=UNRANKED "
            "team_peak_mps=11 team_average_mps=8 "
            "wall_margin_min_m=-0.05 min_body_clearance_m=1.2\n"
            "[BATTLE] RESULT FAIL_WALL_BOUNDARY\n", 12)
        try:
            row = extract(case, "spa", "2", "1", rc)
            self.assertEqual(row["wall_boundary_status"], "CROSSED")
            self.assertEqual(row["failure_category"], "GEOMETRIC_WALL_BOUNDARY")
            self.assertEqual(row["rank"], "UNRANKED")
        finally:
            temp.cleanup()

    def test_vehicle_collision_is_distinct(self):
        temp, case, rc = self.make_case(
            "[BATTLE] COLLISION t=15 pair=team_11/practice_s_09\n"
            "[BATTLE] RESULT FAIL_COLLISION\n", 12)
        try:
            row = extract(case, "cota", "1", "3", rc)
            self.assertEqual(row["failure_category"], "VEHICLE_COLLISION")
            self.assertEqual(row["wall_boundary_status"], "UNKNOWN")
        finally:
            temp.cleanup()

    def test_summary_write(self):
        temp, case, rc = self.make_case("[BATTLE] RESULT FAIL_TIMEOUT\n", 12)
        try:
            row = extract(case, "baku", "1", "3", rc)
            path = Path(temp.name) / "summary.csv"
            append(path, row)
            append(path, row)
            lines = path.read_text().strip().splitlines()
            self.assertEqual(len(lines), 3)
            self.assertIn("team_average_mps", lines[0])
        finally:
            temp.cleanup()

if __name__ == "__main__":
    unittest.main()
