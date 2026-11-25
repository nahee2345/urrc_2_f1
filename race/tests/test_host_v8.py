import importlib.util
import tempfile
import unittest
from pathlib import Path
HERE=Path(__file__).resolve().parents[1]/'scripts'
spec=importlib.util.spec_from_file_location('hg',HERE/'host_guard_v8.py');hg=importlib.util.module_from_spec(spec);spec.loader.exec_module(hg)
class TestHost(unittest.TestCase):
 def test_metrics(self):
  self.assertGreater(hg.HostGuard.mem_available_mb(),0)
  self.assertGreater(hg.HostGuard.cpu_sample()[0],0)
 def test_error_log(self):
  with tempfile.TemporaryDirectory() as d:
   g=hg.HostGuard(Path(d)/'e.jsonl')
   with self.assertRaises(hg.HostOverload):g.fail('TEST','test')
   self.assertIn('TEST',(Path(d)/'e.jsonl').read_text())
 def test_guard(self):
  with tempfile.TemporaryDirectory() as d:
   g=hg.HostGuard(Path(d)/'e.jsonl',min_mem_mb=0,min_disk_mb=0)
   g.check(now=10);g.check(now=11)
if __name__=='__main__':unittest.main()
