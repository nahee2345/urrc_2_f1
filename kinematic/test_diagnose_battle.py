#!/usr/bin/env python3
"""Unit tests for existing-log battle failure classification; no Gazebo needed."""
import tempfile
import unittest
from pathlib import Path
from diagnose_battle import classify


class BattleDiagnosisTests(unittest.TestCase):
    def run_case(self, result, incident="", rc=12):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / "prepared"
            folder.mkdir()
            (folder / "battle_state.csv").write_text("time_s,team_s\n")
            (folder.parent / "gazebo.log").write_text(
                incident + "\n[BATTLE] RESULT " + result +
                " overtake=0 defense=0 defense_status=UNVERIFIED_AT_FINISH\n")
            case = Path(tmp) / "case.log"
            case.write_text("Log: " + str(folder / "battle_state.csv") + "\n")
            return classify(case, rc)

    def test_task_failure_is_not_collision(self):
        result = self.run_case("FAIL_NO_OVERTAKE")
        self.assertEqual(result["failure_category"], "TASK_NO_OVERTAKE")
        self.assertEqual(result["wall_contact"], "NOT_MEASURED")

    def test_vehicle_collision_pair(self):
        result = self.run_case("FAIL_COLLISION",
                               "[BATTLE] COLLISION t=10 pair=team_11/practice_s_09")
        self.assertEqual(result["failure_category"], "VEHICLE_COLLISION")
        self.assertEqual(result["vehicle_pair"], "team_11/practice_s_09")

    def test_clearance_is_not_collision(self):
        result = self.run_case("FAIL_CLEARANCE",
                               "[BATTLE] CLEARANCE_BREACH pair=team_11/practice_rear_11 actual=0.48 required=0.5")
        self.assertEqual(result["failure_category"], "VEHICLE_CLEARANCE_BREACH")
        self.assertIn("0.48m", result["detail"])

    def test_scenario_collision_is_separate(self):
        result = self.run_case("FAIL_SCENARIO_COLLISION",
                               "[BATTLE] COLLISION t=10 pair=practice_s_09/practice_rear_11")
        self.assertEqual(result["failure_category"], "PRACTICE_VEHICLE_COLLISION")

    def test_defense_partial_is_not_impact(self):
        result = self.run_case("PARTIAL_DEFENSE_UNVERIFIED", rc=0)
        self.assertEqual(result["failure_category"], "TASK_DEFENSE_UNVERIFIED")

    def test_timeout_without_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = Path(tmp) / "case.log"
            case.write_text("FAIL: no battle result within 600 seconds\n")
            result = classify(case, 11)
            self.assertEqual(result["failure_category"], "EXECUTION_TIMEOUT")


if __name__ == "__main__":
    unittest.main()
