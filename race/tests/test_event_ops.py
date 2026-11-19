import importlib.util,sys,tempfile,json,unittest
from pathlib import Path
folder=Path(__file__).resolve().parents[1]/'scripts';sys.path.insert(0,str(folder))
spec=importlib.util.spec_from_file_location('event_ops',folder/'event_ops.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Tests(unittest.TestCase):
 def test_lifecycle(self):
  with tempfile.TemporaryDirectory() as d:
   d=Path(d);draw=d/'draw.json';draw.write_text(json.dumps(dict(qualifying='spa',semi_a='cota',semi_b='baku',final='monaco')))
   order=d/'grid.txt';order.write_text('grid,team\n'+''.join(f'{i},team_{i:02d}\n' for i in range(1,11)))
   s=d/'s';o=m.initiate(s,draw,'qualifying',order);self.assertEqual(o['laps'],2)
   m.transition(s,'start');m.transition(s,'pause');m.transition(s,'dnf',2,'blocking');m.transition(s,'resume')
   o=m.transition(s,'invalidate',reason='gazebo_crash');self.assertEqual(o['dnf']['2']['team'],'team_02')
   new=m.replay(s,d/'retry');self.assertEqual(new['grid'],o['grid']);self.assertEqual(new['dnf'],{})
 def test_changed_replay_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   d=Path(d);draw=d/'draw.json';draw.write_text(json.dumps(dict(qualifying='spa',semi_a='cota',semi_b='baku',final='monaco')))
   order=d/'grid.txt';order.write_text('grid,team\n'+''.join(f'{i},team_{i:02d}\n' for i in range(1,11)))
   s=d/'s';m.initiate(s,draw,'qualifying',order);m.transition(s,'invalidate',reason='server')
   order.write_text(order.read_text()+'# edited')
   with self.assertRaises(ValueError):m.replay(s,d/'retry')
if __name__=='__main__':unittest.main()
