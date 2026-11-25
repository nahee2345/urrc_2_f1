#!/usr/bin/env python3
"""Metadata-only spawn preparation. Never supplies pose to navigation."""
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[4]
team=root/'src/team/team_10'
entry=json.loads((team/'race_entry.json').read_text())
model=(team/'models/team10_car/model.sdf').resolve()
# This source layout normalizes to src/team_10; verify the submission contract.
assert entry['entry_id']=='team_10'
assert entry['vehicle_model_file']=='src/team_10/models/team10_car/model.sdf'
sdf=ET.parse(model).getroot()
assert sdf.findtext('.//plugin/topic')=='/model/team_10/cmd_vel'
wheel=sdf.find("./model/link[@name='rear_left_wheel']")
contact_offset=-float(wheel.findtext('pose').split()[2])+float(wheel.findtext('collision/geometry/cylinder/radius'))
assert abs(contact_offset-entry['spawn_z_offset_m'])<1e-6
metadata=json.loads((root/'src/urrc_track_gazebo/tracks/processed/monaco.json').read_text())
slot=next(x for x in metadata['grid'] if x['position']==1)
print('grid:',slot, 'model:',model, 'contact offset:',contact_offset,flush=True)
payload='sdf_filename: '+json.dumps(str(model))+' name: "team_10" allow_renaming: false pose { position { x: %s y: %s z: %s } orientation { z: %s w: %s } }'
import math,re
# Metadata field names are intentionally checked instead of a hardcoded spawn pose.
x,y,z=slot['x'],slot['y'],slot['road_z']
yaw=slot['yaw']
req=payload%(x,y,z+contact_offset,math.sin(yaw/2),math.cos(yaw/2))
result=subprocess.run(['gz','service','-s','/world/monaco/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True)
print(result.stdout,result.stderr)
if result.returncode or 'data: true' not in result.stdout: raise SystemExit('spawn failed')

telemetry=root/'build_f1/telemetry/libteam10-contact-telemetry.so'
if not telemetry.exists():raise SystemExit('Build telemetry first: bash src/team/team_10/tools/build_contact_recorder.sh')
state=subprocess.run(['gz','model','-m','team_10'],capture_output=True,text=True,timeout=10)
match=re.search(r'Model: \[(\d+)\]',state.stdout)
if not match:raise SystemExit('Cannot identify spawned F1 entity for telemetry')
req='entity { id: '+match.group(1)+' type: MODEL } plugins { filename: '+json.dumps(str(telemetry))+' name: "team10::ContactTelemetry" }'
result=subprocess.run(['gz','service','-s','/world/monaco/entity/system/add','--reqtype','gz.msgs.EntityPlugin_V','--reptype','gz.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True)
print('contact telemetry:',result.stdout,result.stderr,flush=True)
if result.returncode or 'data: true' not in result.stdout:raise SystemExit('contact telemetry failed')
