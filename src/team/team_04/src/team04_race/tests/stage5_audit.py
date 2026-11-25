#!/usr/bin/env python3
"""Read-only baseline audit; writes only new stage5 evidence."""
from pathlib import Path
import argparse,hashlib,json,xml.etree.ElementTree as ET

def canonical(node):return (node.tag,tuple(sorted(node.attrib.items())),(node.text or '').strip(),tuple(canonical(c) for c in node))
def actor_physics(path):
 t=ET.parse(path).getroot();m=t.find('model');m.attrib.pop('name',None)
 for p in list(m.findall('plugin')):m.remove(p)
 for l in m.findall('link'):
  for s in list(l.findall('sensor')):l.remove(s)
 return canonical(t)
def audit(root):
 out=root/'src/team_04/artifacts/stage5';base=json.loads((out/'baseline.json').read_text());changed=[];missing=[];protected=0;previous=0
 for n,h in base.items():
  p=root/n
  if not p.is_file():missing.append(n);continue
  source=n.startswith('src/team_04/team04_race/')
  if hashlib.sha256(p.read_bytes()).hexdigest()!=h:changed.append(dict(path=n,allowed=source))
  protected+=int(not n.startswith('src/team_04/'));previous+=int(n.startswith('src/team_04/artifacts/'))
 assert not missing and all(c['allowed'] for c in changed),(missing,changed)
 fixed=['src/vehicle_system.cpp','src/scan.cpp','src/control.cpp','models/team04_car/model.sdf','config/vehicle.yaml','config/sensors.yaml']
 for n in fixed:
  name='src/team_04/team04_race/'+n;assert hashlib.sha256((root/name).read_bytes()).hexdigest()==base[name],name
 archive=root/'src/team_04/artifacts/stage4/final_release/team04_stage4.zip';assert archive.read_bytes()==(out/'team04_stage4_frozen.zip').read_bytes()
 original_model=root/'src/team_04/team04_race/models/team04_car/model.sdf';original=actor_physics(original_model)
 actors={str(p.relative_to(out)):actor_physics(p)==original for p in out.glob('stage5-*/team04_test_actor_*.sdf')}
 assert all(actors.values()),actors
 outside=[str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and not str(p.relative_to(root)).startswith('src/team_04/') and str(p.relative_to(root)) not in base]
 assert not outside,outside
 report=dict(protected_files=protected,previous_artifact_files=previous,old_records_and_protected_unchanged=True,unchanged_vehicle_sensor_and_control=True,stage4_zip_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),changed_source_files=changed,actor_physics_equal_original=actors,missing=missing,outside_team_new_files=outside)
 (out/'preservation_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();audit(a.root.resolve())
