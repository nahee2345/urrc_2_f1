"""단일 Monaco GUI 재사용: 교정 후 새 Gazebo를 띄우지 않는 회귀 검증."""
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
from scripts.stage5_race import select_commands,build_commands
from stage5.session import run_stage5,ensure_live,log_tail,ROOT


class FakeProcess:
    def __init__(self):
        self.pid=123
        self.killed=False
    def poll(self):
        return 1 if self.killed else None


class Stage5SessionReuseTests(unittest.TestCase):
    def test_reuse_omits_second_gazebo_and_bridge(self):
        commands=[("monaco_gazebo",["gz","sim"]),("ros_bridge",["ros2","run"]),
                  ("camera_light_gate",["python","gate"]),
                  ("rgb_controller",["python","drive"]),
                  ("race_supervisor",["python","supervisor"]),
                  ("isolated_operator_permission",["ros2","topic"])]
        result=select_commands(commands,reuse_gazebo=True)
        self.assertEqual(len(result),4)
        self.assertNotIn("monaco_gazebo",[x[0] for x in result])
        self.assertNotIn("ros_bridge",[x[0] for x in result])
        self.assertEqual(select_commands(commands,reuse_gazebo=False),commands)

    def test_carry_same_gui_from_calibration_into_race(self):
        gui,bridge=FakeProcess(),FakeProcess()
        active=[]
        def launch(name,cmd,env,session):
            proc=gui if name=="gazebo" else bridge
            session.append((name,proc,io.BytesIO()))
            active.append(name)
            return proc
        observed=[]
        def inspect(command,env):
            observed.append((command,gui.poll(),bridge.poll(),list(active)))
            return 0
        with mock.patch("stage5.session.launch_one",side_effect=launch),\
             mock.patch("stage5.session.await_gazebo"),\
             mock.patch("stage5.session.ensure_live"),\
             mock.patch("stage5.session.calibrate_on_existing"),\
             mock.patch("stage5.session.close_process"),\
             mock.patch("scripts.stage5_start.calibrated",side_effect=[False,True,True]),\
             mock.patch("scripts.stage5_start.run_race",side_effect=inspect):
            result=run_stage5(Path("generated/stage5/AI-01/calibration.json"),
                              .55,10,False,False,{"GZ_PARTITION":"test"})
        self.assertEqual(result,0)
        self.assertEqual(active,["gazebo","bridge"])
        self.assertEqual(len(observed),1)
        self.assertIsNone(observed[0][1])
        self.assertIsNone(observed[0][2])
        self.assertIn("--reuse-gazebo",observed[0][0])
        self.assertEqual(observed[0][3],["gazebo","bridge"])

    def test_valid_calibration_avoids_restart_and_recalibration(self):
        gui,bridge=FakeProcess(),FakeProcess()
        launched=[]
        def launch(name,cmd,env,session):
            launched.append(name)
            proc=gui if name=="gazebo" else bridge
            session.append((name,proc,io.BytesIO()))
            return proc
        with mock.patch("stage5.session.launch_one",side_effect=launch),\
             mock.patch("stage5.session.await_gazebo"),\
             mock.patch("stage5.session.ensure_live"),\
             mock.patch("stage5.session.calibrate_on_existing") as calibration,\
             mock.patch("stage5.session.close_process"),\
             mock.patch("scripts.stage5_start.calibrated",return_value=True),\
             mock.patch("scripts.stage5_start.run_race",return_value=0):
            result=run_stage5("generated/stage5/AI-01/calibration.json",
                              .55,10,False,False,{"GZ_PARTITION":"test"})
        self.assertEqual(result,0)
        self.assertEqual(launched,["gazebo","bridge"])
        calibration.assert_not_called()

    def test_gazebo_startup_failure_displays_log_tail(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"gazebo.log"
            p.write_text("Gazebo render error test\n")
            self.assertIn("render error",log_tail(p))
            self.assertIn("로그 없음",log_tail(Path(d)/"none.log"))

    def test_race_script_accepts_reuse_flag(self):
        result=subprocess.run([sys.executable,str(ROOT/"scripts/stage5_race.py"),"--help"],
                              cwd=ROOT,text=True,capture_output=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn("--reuse-gazebo",result.stdout)

if __name__=="__main__":unittest.main()
