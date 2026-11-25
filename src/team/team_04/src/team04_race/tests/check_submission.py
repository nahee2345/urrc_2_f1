#!/usr/bin/env python3
"""Read-only submission audit, including the final ZIP structure."""
from pathlib import Path
import argparse,json,zipfile,xml.etree.ElementTree as E,re,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from validate_dimensions import bounds

def check(package,archive=None):
 data=json.loads((package/'race_entry.json').read_text());assert data['entry_id']=='team04'
 assert data['ros_package']==E.parse(package/'package.xml').getroot().findtext('name')=='team04_race'
 assert data['vehicle_model_file']=='src/team04_race/models/team04_car/model.sdf'
 assert all(r['legal'] and r['target'] for r in bounds(package/'models/team04_car/model.sdf'))
 launch=(package/'launch/race.launch.py').read_text()
 for arg in ['race_start_mode','race_start_topic','speed_topic','steering_topic','lidar_topic','use_sim_time']:assert arg in launch
 assert "executable='create'" not in launch
 for p in package.rglob('*'):
  assert not p.is_symlink(),str(p)
  if p.is_file() and p.suffix in ['.cpp','.py','.yaml','.json','.xml','.sdf','.txt','.in'] and '__pycache__' not in p.parts:
   text=p.read_text();assert not re.search(r'/' + r'home/[^\s]+',text),str(p)
 if archive:
  with zipfile.ZipFile(archive) as z:
   names=z.namelist();assert 'race_entry.json' in names and 'src/team04_race/package.xml' in names
   assert json.loads(z.read('race_entry.json'))==data
   for n in names:
    assert not set(Path(n).parts)&{'build','install','log','.git','__pycache__','.pytest_cache'}
    assert not n.startswith('/') and '..' not in Path(n).parts
    assert n=='race_entry.json' or n.startswith('src/team04_race/')
 print('PASS: metadata, dimensions, launch contract, portability, ZIP structure' if archive else 'PASS: metadata, dimensions, launch contract, portability')
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--package',type=Path,default=Path(__file__).resolve().parents[1]);a.add_argument('--zip',type=Path);args=a.parse_args();check(args.package,args.zip)
