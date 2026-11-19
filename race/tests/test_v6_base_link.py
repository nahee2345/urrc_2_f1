import sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base_link_check_v6 import check

class CenterTest(unittest.TestCase):
    def test_centered(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'model.sdf';p.write_text('<sdf version="1.9"><model name="a"><link name="base_link"><collision name="c"><geometry><box><size>0.5 0.2 0.12</size></box></geometry></collision></link></model></sdf>')
            self.assertEqual(check(p)['status'],'GEOMETRIC_PASS')
    def test_offset(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'model.sdf';p.write_text('<sdf version="1.9"><model name="a"><link name="base_link"><collision name="c"><pose>0.1 0 0</pose><geometry><box><size>0.5 0.2 0.12</size></box></geometry></collision></link></model></sdf>')
            self.assertEqual(check(p)['status'],'FAIL')
    def test_missing(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'model.sdf';p.write_text('<sdf version="1.9"><model name="a"><link name="other"/></model></sdf>')
            self.assertEqual(check(p)['status'],'FAIL')
if __name__=='__main__':unittest.main()
