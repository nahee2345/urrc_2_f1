"""Monaco 5단계 단일 터미널 실행기 안전 회귀검사.

실제 렌즈/카메라 신호는 모의 데이터로 PASS하지 않는다.
실제 Gazebo 시뮬레이션과 ROS2 연결은 Ubuntu 현장에서 확인한다.
"""
from __future__ import annotations
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from scripts.stage5_start import (
    calibrated, stages, base_environment, calibrate_in_one_terminal, ROOT
)


class OneTerminalTests(unittest.TestCase):
    def test_all_commands_are_in_one_launcher(self):
        commands = stages(minutes=120)
        self.assertEqual([x[0] for x in commands],
                         ["world_prepare", "bridge_prepare", "race"])
        self.assertIn("stage5_prepare_monaco.py", " ".join(commands[0][1]))
        self.assertIn("stage5_bridge.py", " ".join(commands[1][1]))
        self.assertIn("stage5_race.py", " ".join(commands[2][1]))
        self.assertIn("0.55", commands[2][1])
    def test_calibrate_only_does_not_drive(self):
        commands = stages(calibrate_only=True)
        self.assertEqual(len(commands),2)
        self.assertNotIn("race", [x[0] for x in commands])
    def test_bounds_reject_unsafe_speed_and_time(self):
        for speed,minutes in ((.95,120),(-1,120),(.55,0),(.55,181)):
            with self.assertRaises(ValueError):
                stages(speed=speed,minutes=minutes)
    def test_invalid_or_non_monaco_settings_are_not_calibrated(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"calibration.json"
            self.assertFalse(calibrated(p))
            p.write_text(json.dumps({
                "calibrated":False,"track":"monaco","source_sensor":"start_camera",
                "rois":[[.1,.1,.2,.2]]*5
            }))
            self.assertFalse(calibrated(p))
            p.write_text(json.dumps({
                "calibrated":True,"track":"monza","source_sensor":"start_camera",
                "rois":[[.1,.1,.2,.2]]*5
            }))
            self.assertFalse(calibrated(p))
            p.write_text("{not-json")
            self.assertFalse(calibrated(p))
    def test_ros2_and_gazebo_share_partition(self):
        with mock.patch.dict(os.environ,{"GZ_PARTITION":"monaco_test_unique"}):
            env=base_environment()
            self.assertEqual(env["GZ_PARTITION"],"monaco_test_unique")
            self.assertIn(str(ROOT),env["PYTHONPATH"].split(os.pathsep))
    def test_python_direct_script_help_succeeds(self):
        with tempfile.TemporaryDirectory() as d:
            p=subprocess.run([sys.executable,str(ROOT/"scripts/stage5_start.py"),
                              "--help"],cwd=d,text=True,capture_output=True,timeout=12)
            self.assertEqual(p.returncode,0)
            self.assertIn("--calibrate-only",p.stdout)
            self.assertIn("--recalibrate",p.stdout)
            self.assertNotIn("ModuleNotFoundError",p.stderr)
    def test_dry_run_without_ros_or_gazebo(self):
        with tempfile.TemporaryDirectory() as d:
            p=subprocess.run([sys.executable,str(ROOT/"scripts/stage5_start.py"),
                              "--dry-run"],cwd=d,text=True,capture_output=True,timeout=12)
            self.assertEqual(p.returncode,0,p.stderr)
            self.assertIn("stage5_prepare_monaco.py",p.stdout)
            self.assertIn("stage5_bridge.py",p.stdout)
            self.assertIn("stage5_race.py",p.stdout)
            self.assertIn("5등",p.stdout)
    def test_calibration_fails_closed_before_motor_start(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"calibration.json"
            with mock.patch("scripts.stage5_start.subprocess.Popen") as popen:
                popen.side_effect=OSError("Gazebo 모의 실행 실패")
                with self.assertRaises(OSError):
                    calibrate_in_one_terminal(base_environment(),p,seconds=3)
                self.assertFalse(p.exists())


if __name__=="__main__":
    unittest.main()
