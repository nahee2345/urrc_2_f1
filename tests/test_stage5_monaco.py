"""5단계: 공식 Monaco 3D 지형·신호등·AI-01 출발/랩 코드 정적 검증."""
from pathlib import Path
import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from stage5.monaco import prepared_package,generate_monaco,inspect_original
from stage3.lap import gates_from_upstream,LapSupervisor
from stage4.safety import CameraDrive
from stage4.lane import LaneEstimate
from vehicle_generator.model import Vehicle
from dataclasses import replace

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/"upstream/urrc_f1_ws"

class MonacoStage5Test(unittest.TestCase):
    def test_source_elevation_width_and_finish(self):
        report=SRC/"src/urrc_track_gazebo/docs/monaco_geometry_report.json"
        if not report.exists():self.skipTest("서브모듈 준비되지 않음")
        d=json.loads(report.read_text())
        self.assertEqual(d["track"],"monaco")
        self.assertGreater(d["elevation_range_m"],8)
        self.assertLess(d["width_min_m"],1.5)
        self.assertAlmostEqual(d["length_3d_m"],667.16,delta=.5)
        finish,cps=gates_from_upstream(SRC,"monaco")
        self.assertGreater(len(cps),50)
        self.assertAlmostEqual(finish["z"],2.1452267296,delta=.1)

    def test_official_monaco_assets_and_vehicle_spawn(self):
        if not (SRC/"src/urrc_track_gazebo/urrc_tracks/v10_assets.py").is_file():
            self.skipTest("서브모듈 준비되지 않음")
        with tempfile.TemporaryDirectory(prefix="stage5_test_") as d:
            output=Path(d)
            result=generate_monaco(SRC,output,"AI-01")
            self.assertEqual(result["fidelity"]["status"],"PASS_MONACO_SOURCE_FIDELITY")
            self.assertGreater(result["fidelity"]["mesh_z_range_m"],6)
            self.assertEqual(result["fidelity"]["signal_lenses"],5)
            self.assertEqual(result["fidelity"]["spectator_cameras"],4)
            root=ET.parse(result["world"]).getroot()
            world=root.find("world")
            self.assertEqual(world.get("name"),"monaco")
            self.assertIsNotNone(world.find("./model[@name='urrc_ai_01']"))
            self.assertEqual(world.findtext("include/uri"),"model://urrc_monaco")
            car=world.find("./model[@name='urrc_ai_01']")
            self.assertEqual(len(car.findall(".//sensor[@type='camera']")),2)
            self.assertGreater(len(car.findall(".//sensor[@type='contact']")),5)
            rgb=[float(x) for x in car.findtext(
                "./link[@name='base_link']/visual[@name='body_visual']/material/diffuse").split()[:3]]
            self.assertTrue(all(v<.05 for v in rgb))
            self.assertGreater(result["checkpoints"],50)
            check=inspect_original(SRC,output/"resources/urrc_track_gazebo",result["world"])
            self.assertEqual(check["status"],"PASS_MONACO_SOURCE_FIDELITY")
            self.assertTrue((output/"resources/urrc_track_gazebo/worlds/monaco_signal_active.sdf").exists())
            # 캐시를 두 번째 생성할 때 원본과 바이트 고정 조건 유지
            reread=generate_monaco(SRC,output,"AI-01")
            self.assertEqual(reread["fidelity"]["original_collision_sha256"],
                             result["fidelity"]["original_collision_sha256"])

    def test_monaco_lap_supervisor_not_auto_complete(self):
        if not (SRC/"src/urrc_track_gazebo/tracks/processed/monaco.json").exists():
            self.skipTest("서브모듈 준비되지 않음")
        finish,cps=gates_from_upstream(SRC,"monaco")
        x=LapSupervisor(finish,cps,target=3)
        self.assertEqual(x.status()["status"],"NOT_PASSED")
        self.assertEqual(x.laps,0)
        self.assertFalse(x.contact_monitor_ready)
        self.assertEqual(x.target,3)

    def test_safe_start_must_be_true_for_monaco(self):
        control=CameraDrive(replace(Vehicle(),name="urrc_ai_01",drive="fwd"),
                             speed_limit_mps=.55)
        control.camera(LaneEstimate(valid=True,error=.12,confidence=.9,samples=5),1.)
        a=control.tick(1.)
        self.assertFalse(a["allowed"])
        self.assertEqual(a["speed_mps"],0)
        control.permission(True,1.1)
        control.camera(LaneEstimate(valid=True,error=.12,confidence=.9,samples=5),1.1)
        b=control.tick(1.1)
        self.assertTrue(b["allowed"])
        self.assertGreater(b["speed_mps"],0)
        control.permission(False,1.12)
        c=control.tick(1.16)
        self.assertFalse(c["allowed"])
        self.assertEqual(c["speed_mps"],0)

if __name__=="__main__":unittest.main()
