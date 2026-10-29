"""5단계 현장 종료 오류·실제 카메라/전륜·위치 진단 증거 회귀검사."""
from pathlib import Path
import json
import subprocess
import tempfile
import unittest
from unittest import mock
from scripts.stage5_race import diagnostic_line,read_diagnostics,stop_motors


class Stage5LiveDiagnosticsTest(unittest.TestCase):
    def test_missing_status_exposes_uncertainty(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertIsNone(read_diagnostics(Path(folder)/"missing.json"))
            self.assertIn("미수신",diagnostic_line(None,None))
    def test_real_status_read_and_natural_summary(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/"drive.json"
            p.write_text(json.dumps({"camera_frames":277,"start_allowed":True,
                "lane_valid":False,"stop_reason":"연속 도로 경계 부족",
                "front_left_radps":0}))
            x=read_diagnostics(p)
            summary=diagnostic_line(x,{"displacement_from_start_xy_m":0.12,
                                       "distance_measured_xy_m":0.13,"laps":0})
            self.assertIn("277프레임",summary)
            self.assertIn("도로검출=False",summary)
            self.assertIn("연속 도로 경계 부족",summary)
            self.assertIn("실제 변위=0.12m",summary)
    def test_invalid_status_refuses_to_invent_data(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/"drive.json"
            p.write_text("{broken")
            self.assertIsNone(read_diagnostics(p))
    def test_exit_sends_four_explicit_zero_commands(self):
        response=subprocess.CompletedProcess([],0,"","")
        with mock.patch("scripts.stage5_race.subprocess.run",return_value=response) as call:
            problems=stop_motors({"GZ_PARTITION":"test"})
        self.assertEqual(problems,[])
        self.assertEqual(call.call_count,4)
        topics=[]
        for item in call.call_args_list:
            args=item.args[0]
            self.assertEqual(args[args.index("-p")+1],"data: 0")
            topics.append(args[args.index("-t")+1])
        self.assertEqual(set(topics),{
            "/model/urrc_ai_01/drive/front_left",
            "/model/urrc_ai_01/drive/front_right",
            "/model/urrc_ai_01/steer/left",
            "/model/urrc_ai_01/steer/right",
        })
    def test_stop_failure_does_not_create_fake_success(self):
        bad=subprocess.CompletedProcess([],1,"","failed")
        with mock.patch("scripts.stage5_race.subprocess.run",return_value=bad):
            self.assertEqual(len(stop_motors({})),4)

if __name__=="__main__":unittest.main()
