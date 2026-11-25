import sys,unittest,tempfile,random,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_v3 as v
class TestV3(unittest.TestCase):
 def test_candidates(self):
  for m in v.assignments():
   self.assertEqual(len(set(m.values())),4)
   self.assertNotEqual(m['qualifying'],'monaco')
   self.assertNotEqual(m['final'],'monza')
   self.assertEqual(set([m['semi_a'],m['semi_b']]),set(v.SEMI_PAIR))
 def test_refresh_different_each_stage(self):
  with tempfile.TemporaryDirectory() as d:
   path=Path(d)/'maps.json'
   one=v.refresh(path,random.Random(1))['maps']
   two=v.refresh(path,random.Random(2))['maps']
   self.assertTrue(all(one[s]!=two[s] for s in v.STAGES))
   self.assertEqual(v.load(path)['maps'],two)
 def test_no_entries(self):
  with tempfile.TemporaryDirectory() as d:self.assertEqual(v.entrant_count(Path(d)),0)
 def test_corrupt(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'maps.json';p.write_text(json.dumps({'maps':dict.fromkeys(v.STAGES,'spa')}))
   with self.assertRaises(ValueError):v.load(p)
if __name__=='__main__':unittest.main()
