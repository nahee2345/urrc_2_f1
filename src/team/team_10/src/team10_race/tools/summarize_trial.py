#!/usr/bin/env python3
"""Offline movement evidence; does not publish or control anything."""
import json,math,sys
from pathlib import Path
path=Path(sys.argv[1] if len(sys.argv)>1 else 'validation_f1/chain.jsonl')
rows=[json.loads(x) for x in path.read_text().splitlines()]
rows=[r for r in rows if 'ground_truth_measurement_only' in r['values']]
if not rows:raise SystemExit('No actual vehicle poses recorded yet')
origin=rows[0]['values']['ground_truth_measurement_only']
last=rows[-1]['values']['ground_truth_measurement_only']
forward=(last[0]-origin[0])*math.cos(origin[3])+(last[1]-origin[1])*math.sin(origin[3])
released=next((r for r in rows if r['values'].get('/team_10/camera_gate_10')),None)
pre=[r for r in rows if released and r['wall']<released['wall']-.2]
zero=all(r['values'].get('/team_10/speed_10',0)==0 and r['values'].get('/team_10/steering_10',0)==0 and r['values'].get('/team_10/gz_cmd_vel_10',[0,0])==[0,0] for r in pre)
print(json.dumps(dict(source=str(path),initial_pose=origin,latest_pose=last,forward_m=forward,
    observed_camera_release=released is not None,pre_release_outputs_zero=zero if pre else None,
    measured_forward_2m=bool(released and forward>=2.0)),indent=2))
