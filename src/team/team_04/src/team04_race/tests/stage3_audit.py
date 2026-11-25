#!/usr/bin/env python3
"""Compare against the pre-stage3 SHA-256 inventory without touching originals."""
import argparse,hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path

def audit(root):
 out=root/'src/team_04/artifacts/stage3';baseline=json.loads((out/'baseline.json').read_text());changed=[];missing=[];counts={'protected':0,'stage2':0,'source':0}
 for name,expected in baseline.items():
  path=root/name
  group='stage2' if '/artifacts/stage2/' in name else 'source' if name.startswith('src/team_04/team04_race/') else 'protected';counts[group]+=1
  if not path.is_file():missing.append(dict(group=group,path=name))
  elif hashlib.sha256(path.read_bytes()).hexdigest()!=expected:changed.append(dict(group=group,path=name))
 old_sources={n for n in baseline if n.startswith('src/team_04/team04_race/')};current_sources={str(p.relative_to(root)) for p in (root/'src/team_04/team04_race').rglob('*') if p.is_file() and '__pycache__' not in p.parts}
 frozen=out/'baseline_stage2_source.zip';stage2=root/'src/team_04/artifacts/stage2/team04_stage2.zip'
 protected_new=[str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and not p.relative_to(root).as_posix().startswith('src/team_04/') and str(p.relative_to(root)) not in baseline]
 report=dict(counts=counts,changed=changed,missing=missing,new_source_files=sorted(current_sources-old_sources),outside_team_new_files=protected_new,stage2_zip_sha256=hashlib.sha256(stage2.read_bytes()).hexdigest(),frozen_copy_identical=frozen.read_bytes()==stage2.read_bytes())
 report['protected_and_stage2_unchanged']=not any(r['group']!='source' for r in changed+missing) and not protected_new and report['frozen_copy_identical']
 for name in ['src/scan.cpp','src/vehicle_system.cpp','models/team04_car/model.sdf','config/sensors.yaml','config/vehicle.yaml']:
  path='src/team_04/team04_race/'+name
  assert hashlib.sha256((root/path).read_bytes()).hexdigest()==baseline[path],path
 report['vehicle_sensor_sources_unchanged']=True
 original=ET.tostring(ET.parse(root/'src/team_04/team04_race/models/team04_car/model.sdf').getroot());instruments={}
 for path in out.glob('*/instrumented.sdf'):
  tree=ET.parse(path).getroot();model=tree.find('model')
  for child in list(model):
   if child.tag=='plugin' and child.get('filename')=='libteam04_trial_observer.so':model.remove(child)
  instruments[path.parent.name]=ET.tostring(tree)==original
  assert instruments[path.parent.name],path
 report['test_models_match_original_except_observer']=instruments
 (out/'preservation_audit.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2));assert report['protected_and_stage2_unchanged']
 return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();audit(a.root.resolve())
