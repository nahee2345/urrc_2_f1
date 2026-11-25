#!/usr/bin/env python3
"""Read-only release checks plus official validation in a team-local fixture."""
import argparse,hashlib,json,sys,zipfile
from pathlib import Path
from check_submission import check

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True);p.add_argument('--archive',type=Path,required=True);p.add_argument('--artifacts-dir',type=Path);a=p.parse_args();root=a.root.resolve();fixture=a.fixture.resolve();archive=a.archive.resolve();source=root/'src/team_04/team04_race';out=a.artifacts_dir.resolve() if a.artifacts_dir else root/'src/team_04/artifacts/stage4'
 if root/'src/team_04' not in fixture.parents:raise ValueError('fixture must stay inside team_04')
 sys.path.insert(0,str(root/'race/scripts'));import race_manager
 check(source,archive)
 prefix=__import__('ament_index_python.packages',fromlist=['get_package_prefix']).get_package_prefix('team04_race');prefix=Path(prefix)
 assert fixture in prefix.parents,(fixture,prefix)
 entry=fixture/'race/entrants/team04';share=prefix/'share/team04_race';race_manager.validate(fixture)
 files=[]
 with zipfile.ZipFile(archive) as z:
  for name in z.namelist():
   if name.endswith('/'):continue
   data=z.read(name)
   assert (entry/name).read_bytes()==data,name
   if name.startswith('src/team04_race/'):
    relative=Path(name).relative_to('src/team04_race');assert (source/relative).read_bytes()==data,name;files.append(str(relative))
   elif name=='race_entry.json':assert json.loads(data)==json.loads((source/name).read_text())
   else:raise AssertionError(name)
 current={str(f.relative_to(source)) for f in source.rglob('*') if f.is_file() and '__pycache__' not in f.parts}
 assert current==set(files),(current-set(files),set(files)-current)
 required=['race_entry.json','config/autonomy.yaml','config/autonomy_stage2_rollback.yaml','config/bridge.yaml','config/sensors.yaml','config/vehicle.yaml','config/autonomy.rviz','models/team04_car/model.sdf','launch/race.launch.py','docs/STAGE3.md','docs/STAGE4.md','config/autonomy_stage3_rollback.yaml']
 for name in required:assert (share/name).is_file(),name
 for name in ['lib/team04_race/autonomy_04','lib/team04_race/control_04','lib/team04_race/scan_04','lib/libteam04_vehicle_system.so','lib/libteam04_trial_observer.so']:assert (prefix/name).is_file(),name
 for name in ['src/scan.cpp','src/vehicle_system.cpp','models/team04_car/model.sdf','config/sensors.yaml','config/vehicle.yaml']:
  before=json.loads((out/'baseline.json').read_text())['src/team_04/team04_race/'+name];assert hashlib.sha256((entry/'src/team04_race'/name).read_bytes()).hexdigest()==before,name
 report=dict(fresh_zip_build=True,official_validator=True,installed_resources=True,zip_equals_source=True,archive_source_files=len(files),zip_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),entry_launch_arguments=json.loads((share/'race_entry.json').read_text())['launch_arguments'],fresh_install_prefix=str(prefix),vehicle_and_sensor_sources_unchanged=True)
 (out/'relocation_check.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
