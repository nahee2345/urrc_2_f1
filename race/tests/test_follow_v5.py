import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

P=Path(__file__).resolve().parents[1]/'scripts'/'follow_v5.py'
spec=importlib.util.spec_from_file_location('follow_v5',P)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class Tests(unittest.TestCase):
    def test_order(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'grid.txt';p.write_text('# test\ngrid,team\n1,team_09\n2,team_02\n')
            self.assertEqual(m.read_order(p),{1:'team_09',2:'team_02'})
    def test_duplicate(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'grid.txt';p.write_text('1,team_01\n2,team_01\n')
            with self.assertRaises(ValueError):m.read_order(p)
    def test_view_offsets(self):
        with patch.object(m,'service') as s:
            m.follow('team_01','rear',2.5,1.3,{'/gui/follow','/gui/follow/offset'})
            self.assertIn('x: -2.5',s.call_args_list[0].args[2])
            m.follow('team_01','front',2.5,1.3,{'/gui/follow','/gui/follow/offset'})
            self.assertIn('x: 2.5',s.call_args_list[2].args[2])
    def test_no_gui(self):
        with self.assertRaises(RuntimeError):m.follow('team_01','rear',2,1,set())
    def test_id(self):
        with self.assertRaises(ValueError):m.follow('bad-model','rear',2,1,{'/gui/follow'})
if __name__=='__main__':unittest.main()
