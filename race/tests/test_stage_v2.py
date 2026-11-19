import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
P=Path(__file__).resolve().parents[1]/'scripts'/'stage_v2.py'
spec=importlib.util.spec_from_file_location('stage_v2',P)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class TestStageV2(unittest.TestCase):
 def test_track_exclusions(self):
  for _ in range(50):
   self.assertNotEqual(m.choose_track('qualifying',None,[],__import__('random').Random()),'monaco')
   self.assertNotEqual(m.choose_track('final',None,[],__import__('random').Random()),'monza')
  with self.assertRaises(ValueError):m.choose_track('final','monza',[],__import__('random').Random())
 def test_selection(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)
   q={'valid':True,'state':'complete','provisional':False,'results':[{'rank':i,'entry_id':f'team_{i:02d}','status':'finished'} for i in range(1,11)]}
   (p/'qualifying.json').write_text(json.dumps(q))
   self.assertEqual(m.eligible_ids('semi_a',p),['team_01','team_03','team_05','team_07','team_09'])
   self.assertEqual(m.eligible_ids('semi_b',p),['team_02','team_04','team_06','team_08','team_10'])
   for name,ids in [('semi_a',[1,3,5,7,9]),('semi_b',[2,4,6,8,10])]:
    t={'valid':True,'state':'complete','provisional':False,'results':[{'rank':j,'entry_id':f'team_{i:02d}','status':'finished'} for j,i in enumerate(ids,1)]}
    (p/f'{name}.json').write_text(json.dumps(t))
   self.assertEqual(m.eligible_ids('final',p),['team_01','team_02','team_03','team_04','team_05','team_06'])
 def test_invalid_previous(self):
  with tempfile.TemporaryDirectory() as d:
   with self.assertRaises(ValueError):m.eligible_ids('semi_a',Path(d))
if __name__=='__main__':unittest.main()
