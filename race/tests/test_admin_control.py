import importlib.util
from pathlib import Path
import tempfile
import unittest

FILE=Path(__file__).resolve().parents[1]/'scripts'/'admin_control.py'
spec=importlib.util.spec_from_file_location('admin_control',FILE)
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)

class TestAdmin(unittest.TestCase):
    def test_draw(self):
        with tempfile.TemporaryDirectory() as d:
            for seed in range(100):
                x=a.draw_tracks(Path(d)/f'{seed}.json',seed)
                self.assertNotEqual(x['qualifying'],'monaco')
                self.assertEqual({x['semi_a'],x['semi_b']},{'cota','baku'})
                self.assertEqual(len({x[k] for k in ('qualifying','semi_a','semi_b','final')}),4)
                self.assertNotEqual(x['final'],'monza')
    def test_duplicate_grid(self):
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'grid.csv'
            f.write_text('grid,team\n1,team_01\n1,team_02\n')
            with self.assertRaises(ValueError):a.validate_order(f)
    def test_duplicate_team(self):
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'grid.csv'
            f.write_text('grid,team\n1,team_01\n2,team_01\n')
            with self.assertRaises(ValueError):a.validate_order(f)
    def test_valid_grid(self):
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'grid.csv';f.write_text('grid,team\n1,team_01\n2,team_10\n')
            self.assertEqual(a.validate_order(f,2),{1:'team_01',2:'team_10'})
    def test_fail_safe_pause(self):
        with tempfile.TemporaryDirectory() as d:
            calls=[]
            orig_pause,orig_remove,orig_resume=a.pause_world,a.remove_entity,a.resume_world
            try:
                a.pause_world=lambda world:calls.append('pause')
                def failed(world,team):calls.append('remove');raise RuntimeError('fail')
                a.remove_entity=failed
                a.resume_world=lambda world:calls.append('resume')
                with self.assertRaises(RuntimeError):a.delete_grid('monza',1,{1:'team_01'},Path(d)/'audit.jsonl','blocking')
                self.assertEqual(calls,['pause','remove'])
            finally:a.pause_world,a.remove_entity,a.resume_world=orig_pause,orig_remove,orig_resume

if __name__=='__main__':unittest.main()
