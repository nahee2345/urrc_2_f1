#!/usr/bin/env bash
# Start a read-only browser viewer; never launch Gazebo, spawn cars or send lights.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [[ -f race/install/setup.bash ]]; then
  set +u; source race/install/setup.bash; set -u
fi
# The operator session selects the track and stage, never CLI guesses.
eval "$(python3 - <<'PY2'
import json, pathlib, os, shlex
p=pathlib.Path('race/results/operator_v26_active.json')
if not p.exists(): raise SystemExit('ERROR: 공식 경기가 실행 중이지 않습니다 (operator session missing)')
d=json.loads(p.read_text());pid=d.get('pid')
try:os.kill(int(pid),0)
except (OSError,ValueError,TypeError):raise SystemExit('ERROR: 공식 경기 운영 프로세스 미실행')
for k,v in [('URRC_SPECTATOR_STAGE',d.get('stage')),('URRC_SPECTATOR_TRACK',d.get('track'))]:
 if not isinstance(v,str):raise SystemExit('ERROR: 공식 경기 정보 누락 '+k)
 print('export '+k+'='+shlex.quote(v))
PY2
)"
# Gazebo transport must share the official race partition.
if [[ -z "${GZ_PARTITION:-}" ]]; then
  GZ_PARTITION="$(python3 - <<'PY2'
from pathlib import Path
import os
matches=set()
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  command=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
  if 'gz sim' not in command:continue
  env=(p/'environ').read_bytes().split(b'\0')
  matches.update(x.split(b'=',1)[1].decode() for x in env if x.startswith(b'GZ_PARTITION=urrc_race_'))
 except (OSError,PermissionError):continue
if len(matches)!=1:raise SystemExit(f'ERROR: 공식 Gazebo partition 확인 실패: {sorted(matches)}. GZ_PARTITION 직접 설정 필요')
print(next(iter(matches)))
PY2
)"
fi
export GZ_PARTITION
export GZ_SIM_RESOURCE_PATH="$ROOT/src/urrc_track_gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
WEB="$ROOT/race/spectator_3d/web"
if [[ ! -f "$WEB/node_modules/three/build/three.module.js" ]]; then
  echo '[관전] Three.js 설치 (최초 1회)'
  (cd "$WEB" && npm ci --no-audit --no-fund)
fi
# Ensure all required map files exist; create web copies without touching official worlds.
/usr/bin/python3 "$ROOT/race/spectator_3d/assets.py"
exec /usr/bin/python3 -u "$ROOT/race/spectator_3d/server.py"
