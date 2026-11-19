import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
P=Path(__file__).resolve().parents[1]/'scripts'/'inspection_v4.py'
s=importlib.util.spec_from_file_location('inspection_v4',P)
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
class TestV4(unittest.TestCase):
 def test_empty(self):
  with tempfile.TemporaryDirectory() as d:
   r=m.inspect_entries(Path(d))
   self.assertEqual(r['status'],'NO_ENTRANTS')
 def test_camera_topics(self):
  self.assertEqual(len(m.camera_topics('spa')),4)
  self.assertEqual(m.camera_topics('spa')['start'],'/urrc/spa/spectator/start/image')
 def test_camera_missing(self):
  with patch.object(m.shutil,'which',return_value='/usr/bin/gz'),patch.object(m,'run_cmd',return_value=(0,'/urrc/spa/spectator/start/image')):
   r=m.inspect_cameras('spa')
   self.assertEqual(r['status'],'FAIL')
 def test_camera_all(self):
  topics='\n'.join(m.camera_topics('spa').values())
  with patch.object(m.shutil,'which',return_value='/usr/bin/gz'),patch.object(m,'run_cmd',return_value=(0,topics)):
   r=m.inspect_cameras('spa')
   self.assertEqual(r['status'],'PASS')
 def test_vehicle_bad(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d); team=root/'race/entrants/team_01';team.mkdir(parents=True)
   (team/'race_entry.json').write_text(json.dumps({'entry_id':'team_01','ros_package':'team_01_race','algorithm_launch_file':'race.launch.py','vehicle_model_file':'src/car.sdf','speed_topic':'/team_01/speed','steering_topic':'/team_01/steering','lidar_topic':'/team_01/scan'}))
   self.assertEqual(m.inspect_entries(root)['status'],'FAIL')
 def test_model_oversize(self):
  with tempfile.TemporaryDirectory() as d:
   f=Path(d)/'car.sdf';f.write_text('<sdf version="1.10"><model name="car"><link name="base_link"><collision name="big"><geometry><box><size>0.6 0.2 0.1</size></box></geometry></collision></link></model></sdf>')
   errors,_=m.inspect_model(f,'sdf','base_link')
   self.assertTrue(any('규격 초과' in e for e in errors))
if __name__=='__main__':unittest.main()
