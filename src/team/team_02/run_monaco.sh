#!/usr/bin/env bash
# Execute only in the existing URRC operator workspace. No new workspace.
set -euo pipefail
TEAM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$TEAM_DIR/../../.." && pwd)"
[ -f "$ROOT/race_v24.sh" ] || { echo "Official URRC workspace required: $ROOT" >&2; exit 2; }
cd "$ROOT"
set +u
source /opt/ros/jazzy/setup.bash
set -u
echo "[TEAM02] register and build native replay plugin"
bash race_v24.sh sync
rm -rf -- "$ROOT/race/build/team02_race"
bash race_v24.sh prepare
set +u
source "$ROOT/race/install/setup.bash"
set -u
LIB="$ROOT/race/install/lib/libteam02_pose_02.so"
[ -f "$LIB" ] || { echo "Gazebo TEAM02 plugin missing: $LIB" >&2; exit 2; }
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/race/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
python3 - <<'PY'
from pathlib import Path
import sys
root=Path.cwd()
sys.path.insert(0,str(root/"race/scripts"))
from preview_visual_v19 import ensure_assets
ensure_assets(root,"monaco")
PY
echo "[TEAM02] Monaco vehicles grid 1; input READY on terminal after GUI loads"
exec bash race_v24.sh rehearse vehicles --track monaco --team team_02 --seconds 300 --start-mode signal
