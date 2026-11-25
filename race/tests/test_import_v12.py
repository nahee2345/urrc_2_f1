import json
import sys
from pathlib import Path
import tempfile
import unittest
import zipfile

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import import_downloads_v12 as imp

class TestV12Import(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)/'ws'
        self.dl=Path(self.tmp.name)/'Downloads'
        self.dl.mkdir()
        (self.root/'race/scripts').mkdir(parents=True)
        (self.root/'src/team/team_11').mkdir(parents=True)
        (self.root/'src/team/team_11/keep.txt').write_text('never change')
        (self.root/'race_v12.sh').write_text('')
        # use organizer's actual submission.py from the provided source repo
        import shutil
        src=Path(__file__).resolve().parents[1]/'scripts'/'submission.py'
        if src.is_file():shutil.copy2(src,self.root/'race/scripts/submission.py')
        else:
            self.skipTest('official submission.py unavailable in standalone test folder')
    def submission(self,t,aszip=False,oldid='team01'):
        folder=self.dl/t
        (folder/'models/one').mkdir(parents=True)
        (folder/'models/one/model.sdf').write_text('<sdf/>')
        (folder/'package.xml').write_text('<package><name>entry_' + t + '</name></package>')
        (folder/'race_entry.json').write_text(json.dumps({'entry_id':oldid,'ros_package':'entry_'+t,
          'vehicle_model_file':f'src/{t}/models/one/model.sdf'}))
        if aszip:
            z=self.dl/(t+'.zip')
            with zipfile.ZipFile(z,'w') as archive:
                for p in folder.rglob('*'):
                    if p.is_file():archive.write(p,f'{t}/{p.relative_to(folder)}')
            import shutil
            shutil.rmtree(folder)
    def test_import_both_and_preserve_team11(self):
        self.submission('team_01')
        self.submission('team_02',aszip=True,oldid='team02')
        imp.perform(self.root,self.dl,False)
        self.assertFalse((self.root/'src/team/team_01').exists())
        imp.perform(self.root,self.dl,True)
        self.assertTrue((self.root/'src/team/team_02/models/one/model.sdf').is_file())
        self.assertEqual((self.root/'src/team/team_11/keep.txt').read_text(),'never change')
        x=json.loads((self.root/'src/team/team_01/race_entry.json').read_text())
        self.assertEqual(x['entry_id'],'team_01')
        imp.perform(self.root,self.dl,True)  # repeat safe
    def test_wrong_model_path_autocorrect(self):
        self.submission('team_01')
        self.submission('team_02')
        p=self.dl/'team_01/race_entry.json'
        j=json.loads(p.read_text());j['vehicle_model_file']='src/oldname/models/one/model.sdf'
        p.write_text(json.dumps(j))
        imp.perform(self.root,self.dl,True)
        fixed=json.loads((self.root/'src/team/team_01/race_entry.json').read_text())
        self.assertEqual(fixed['vehicle_model_file'],'src/team_01/models/one/model.sdf')
    def test_invalid_one_does_not_import_any(self):
        self.submission('team_01')
        self.submission('team_02')
        (self.dl/'team_02/race_entry.json').unlink()
        with self.assertRaises(ValueError):imp.perform(self.root,self.dl,True)
        self.assertFalse((self.root/'src/team/team_01').exists())
    def test_source_untouched(self):
        self.submission('team_01')
        self.submission('team_02')
        old=(self.dl/'team_01/race_entry.json').read_bytes()
        imp.perform(self.root,self.dl,True)
        self.assertEqual((self.dl/'team_01/race_entry.json').read_bytes(),old)
    def test_zip_traversal_rejected(self):
        path=self.dl/'evil.zip'
        with zipfile.ZipFile(path,'w') as z:z.writestr('../test','bad')
        with self.assertRaises(ValueError):imp.safe_unzip(path,Path(self.tmp.name)/'stage')

if __name__=='__main__':unittest.main()
