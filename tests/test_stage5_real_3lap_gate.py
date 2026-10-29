"""Monaco 진짜 3랩 감사에서는 정적/모의 증거로 PASS해서는 안 됨."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import scripts.stage5_full_lap_audit as audit


class ActualMonacoThreeLapGateTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.root=Path(self.tmp.name)
        patch=mock.patch.object(audit,"DATA",self.root)
        patch.start()
        self.addCleanup(patch.stop)
        self.addCleanup(self.tmp.cleanup)
    def put(self,payload,name):
        path=self.root/name
        path.write_text(json.dumps(payload))
    def test_absent_real_sensor_evidence_never_passes(self):
        result=audit.evaluate(0)
        self.assertEqual(result["status"],"NOT_PASSED")
        self.assertTrue(any(not x["passed"] for x in result["checks"]))
    def test_source_fidelity_alone_does_not_count(self):
        self.put({"status":"PASS_MONACO_SOURCE_FIDELITY","elevation_reference_m":8.4,
                  "source_files_modified":False},"terrain_fidelity.json")
        self.assertEqual(audit.evaluate(0)["status"],"NOT_PASSED")
    def test_synthetic_lap_counter_not_enough(self):
        self.put({"result":{"status":"PASS_THREE_LAPS","laps":3,
                  "collisions":0,"rejected_crossings":0,
                  "invalid_reasons":[],"lap_times_seconds":[100,100,100]}},"stage5_verdict.json")
        self.assertEqual(audit.evaluate(0)["status"],"NOT_PASSED")
    def test_two_laps_even_with_distance_not_enough(self):
        self.put({"result":{"status":"PASS_THREE_LAPS","laps":2,
                  "collisions":0,"rejected_crossings":0,
                  "invalid_reasons":[],"lap_times_seconds":[1200,1200]}},"stage5_verdict.json")
        self.put({"distance_measured_xy_m":2000,"pose_samples":10000},"pose_status.json")
        self.assertEqual(audit.evaluate(0)["status"],"NOT_PASSED")
    def test_stop_after_failed_process_must_not_pass(self):
        self.assertEqual(audit.evaluate(1)["status"],"NOT_PASSED")
    def test_corrupt_lap_evidence_rejected(self):
        (self.root/"lap_events.jsonl").write_text("not-json")
        self.assertEqual(audit.evaluate(0)["status"],"NOT_PASSED")


if __name__=="__main__":unittest.main()
