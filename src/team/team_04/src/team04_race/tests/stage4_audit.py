#!/usr/bin/env python3
"""Stage4 audit. Previous stage records are read-only inputs."""
import argparse,hashlib,json,zipfile,xml.etree.ElementTree as ET
from pathlib import Path

def audit(root):
 out=root/'src/team_04/artifacts/stage4';base=json.loads((out/'baseline.json').read_text());changes=[];missing=[];protected=0;records=0
 for name,expected in base.items():
  source=name.startswith('src/team_04/team04_race/');allowed=source or name=='src/team_04/REPORT_STAGE3.md';path=root/name
  if not path.is_file():missing.append(name);continue
  if not name.startswith('src/team_04/'):protected+=1
  if name.startswith(('src/team_04/artifacts/stage2/','src/team_04/artifacts/stage3/')):records+=1
  if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:changes.append(dict(path=name,allowed=allowed))
 assert not missing and all(c['allowed'] for c in changes),(missing,changes)
 frozen=out/'baseline/team04_stage3.zip';original=root/'src/team_04/artifacts/stage3/team04_stage3.zip';assert frozen.read_bytes()==original.read_bytes()
 with zipfile.ZipFile(frozen) as archive:
  legacy=archive.read('src/team04_race/src/lidar_planner.cpp')
 assert legacy.replace(b'planLidar(',b'planCenterlineLidar(')==(root/'src/team_04/team04_race/src/centerline_planner.cpp').read_bytes()
 fixed=['src/scan.cpp','src/vehicle_system.cpp','models/team04_car/model.sdf','config/sensors.yaml','config/vehicle.yaml']
 for name in fixed:
  path='src/team_04/team04_race/'+name;assert hashlib.sha256((root/path).read_bytes()).hexdigest()==base[path],path
 original_model=ET.tostring(ET.parse(root/'src/team_04/team04_race/models/team04_car/model.sdf').getroot());models={}
 for path in out.glob('*/instrumented.sdf'):
  tree=ET.parse(path).getroot();model=tree.find('model')
  for child in list(model):
   if child.tag=='plugin' and child.get('filename')=='libteam04_trial_observer.so':model.remove(child)
  models[path.parent.name]=ET.tostring(tree)==original_model
 assert all(models.values())
 previous=set(base);new=[str(p.relative_to(root)) for p in (root/'src/team_04/team04_race').rglob('*') if p.is_file() and '__pycache__' not in p.parts and str(p.relative_to(root)) not in previous]
 outside=[str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and not str(p.relative_to(root)).startswith('src/team_04/') and str(p.relative_to(root)) not in base]
 assert not outside,outside
 report=dict(protected_file_count=protected,previous_artifact_count=records,protected_and_previous_records_unchanged=True,changed=changes,new_source_files=sorted(new),missing=missing,outside_team_new_files=outside,vehicle_sensor_sources_unchanged=True,test_models_original_except_observer=models,stage3_zip_sha256=hashlib.sha256(original.read_bytes()).hexdigest())
 report['legacy_centerline_identical_except_function_symbol']=True
 (out/'preservation_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
 return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();audit(a.root.resolve())
