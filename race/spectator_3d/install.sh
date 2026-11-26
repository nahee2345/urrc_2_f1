#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT="$HOME/urrc_2_f1_next"
[[ ! -e "$ROOT" ]] || { echo "중단: $ROOT 이미 존재함 (덮어쓰지 않음)"; exit 2; }
for c in git npm gz colcon; do command -v "$c" >/dev/null || { echo "필요: $c"; exit 2; }; done
/usr/bin/python3 -c 'import aiohttp,numpy; from gz.transport13 import Node; from gz.msgs10.pose_v_pb2 import Pose_V' || { echo 'python3-aiohttp python3-numpy 및 Gazebo Harmonic Python bindings 필요'; exit 2; }
TMP=$(mktemp -d); trap 'rm -rf -- "$TMP"' EXIT
git clone --depth 1 --single-branch --branch main https://github.com/nahee2345/urrc_2.git "$TMP/practice"
git clone --depth 1 --filter=blob:none --sparse --branch main https://github.com/nahee2345/urrc_2_f1.git "$TMP/team-source"
git -C "$TMP/team-source" sparse-checkout set src/team/team_08
mkdir "$ROOT"
mkdir -p "$ROOT/src/team"
cp -a "$TMP/practice/src/urrc_track_gazebo" "$ROOT/src/"
cp -a "$TMP/team-source/src/team/team_08" "$ROOT/src/team/"
{ git -C "$TMP/practice" rev-parse HEAD; git -C "$TMP/team-source" rev-parse HEAD; } > "$ROOT/source_commits.txt"
cp -a "$HERE" "$ROOT/spectator"
cd "$ROOT/spectator/web"
npm install --no-audit --no-fund --ignore-scripts three@0.170.0
cd "$ROOT"
set +u; source /opt/ros/jazzy/setup.bash; set -u
colcon build --base-paths src/team/team_08 --packages-select team08_race --merge-install
python3 - <<'PY'
import sys
from pathlib import Path
p=Path('src/urrc_track_gazebo').resolve();sys.path.insert(0,str(p))
from urrc_tracks.v10_assets import enhance_track
if not (p/'models/urrc_monza/meshes/road_textured.obj').is_file() or not (p/'models/urrc_monza/meshes/asphalt_ground.obj').is_file(): enhance_track(p,'monza')
PY
/usr/bin/python3 spectator/assets.py
printf '\n설치 완료: %s\n' "$ROOT"
