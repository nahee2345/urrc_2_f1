"""5단계 실제 3랩 판정 로그: 접촉 미확인/단축/랩 누락을 절대 PASS로 처리하지 않음."""
import json
import tempfile
from pathlib import Path
import unittest
from scripts.stage5_race import parse_verdict,build_commands

class MonacoVerdictTest(unittest.TestCase):
    def write(self,folder,payload):
        f=Path(folder)/"lap.jsonl"
        f.write_text(json.dumps({"type":"final",**payload},ensure_ascii=False)+"\n")
        return f
    def good(self):
        return {"status":"PASS_THREE_LAPS","laps":3,
                "lap_times_seconds":[100,102,104],"collisions":0,
                "rejected_crossings":0,"invalid_reasons":[]}
    def test_only_verified_clean_three_laps_passes(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(parse_verdict(self.write(d,self.good()))["status"],"PASS_THREE_LAPS")
    def test_contact_monitor_absent_refuses(self):
        with tempfile.TemporaryDirectory() as d:
            p=self.good();p["status"]="PASS_LAPS_ONLY"
            self.assertEqual(parse_verdict(self.write(d,p))["status"],"NOT_PASSED")
    def test_rejected_shortcut_refuses(self):
        with tempfile.TemporaryDirectory() as d:
            p=self.good();p["rejected_crossings"]=1
            self.assertEqual(parse_verdict(self.write(d,p))["status"],"NOT_PASSED")
    def test_invalid_reasons_refuse(self):
        with tempfile.TemporaryDirectory() as d:
            p=self.good();p["invalid_reasons"]=["체크포인트 미통과"]
            self.assertEqual(parse_verdict(self.write(d,p))["status"],"NOT_PASSED")
    def test_missing_logs_and_two_laps_refuse(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(parse_verdict(Path(d)/"none.jsonl")["status"],"NOT_PASSED")
            p=self.good();p["laps"]=2
            self.assertEqual(parse_verdict(self.write(d,p))["status"],"NOT_PASSED")
    def test_unverified_calibration_must_not_start(self):
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/"calibration.json"
            f.write_text(json.dumps({"source_sensor":"start_camera",
                "calibrated":False,"rois":[[.1,.1,.2,.2]]*5}))
            with self.assertRaises(ValueError):
                build_commands(f)

if __name__=="__main__":unittest.main()
