"""사용자 오류 회귀 검사: 직접 실행 모듈 경로·10대 원색·Monaco 신호등 진입."""
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET
from vehicle_generator.model import Vehicle,generate
from vehicle_generator.sensors import build
from vehicle_generator.colors import AI_COLORS,color_for_vehicle

ROOT=Path(__file__).resolve().parents[1]

class Stage5UserIssueRegression(unittest.TestCase):
    def test_all_ten_ai_models_have_distinct_high_visibility_colors(self):
        colors=[]
        self.assertEqual(len(AI_COLORS),10)
        for index in range(1,11):
            ai=f"AI-{index:02d}"
            name=f"urrc_ai_{index:02d}"
            with self.subTest(ai=ai):
                root=ET.fromstring(generate(Vehicle(name=name)))
                body=root.find("./model/link[@name='base_link']/visual[@name='body_visual']")
                rgb=tuple(float(x) for x in body.findtext("material/diffuse").split()[:3])
                self.assertEqual(rgb,tuple(round(x,3) for x in color_for_vehicle(name)))
                self.assertGreater(max(rgb),.90)
                self.assertLess(min(rgb),.71)
                colors.append(rgb)
        self.assertEqual(len(set(colors)),10)
    def test_sensor_housing_matches_vehicle_body(self):
        for n in range(1,11):
            name=f"urrc_ai_{n:02d}"
            model,_=build(Vehicle(name=name),["rgb"])
            root=ET.fromstring(model)
            body=root.findtext("./model/link[@name='base_link']/visual[@name='body_visual']/material/diffuse")
            housing=root.findtext("./model/link[@name='base_link']/visual[@name='rgb_housing_visual']/material/diffuse")
            self.assertEqual(body,housing)
    def test_script_direct_execution_no_module_not_found(self):
        # 설치된 ROS 없어도 잘못된 교정 파일을 안내하고 Python import 문제를 내지 않아야 함
        with tempfile.TemporaryDirectory() as d:
            result=subprocess.run([sys.executable,str(ROOT/"scripts/stage5_race.py"),
                "--calibration",str(Path(d)/"missing.json"),"--minutes","1"],
                cwd=d,text=True,capture_output=True,timeout=15)
            self.assertNotEqual(result.returncode,0)
            self.assertNotIn("ModuleNotFoundError",result.stderr)
            self.assertNotIn("Traceback",result.stderr)
            self.assertIn("missing.json",result.stderr)
    def test_standalone_gazebo_has_light_option(self):
        result=subprocess.run([sys.executable,str(ROOT/"scripts/stage5_run_monaco.py"),
            "--help"],cwd=ROOT,text=True,capture_output=True,timeout=15)
        self.assertEqual(result.returncode,0)
        self.assertIn("--lights",result.stdout)
if __name__=="__main__":
    unittest.main()
