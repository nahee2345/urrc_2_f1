#!/usr/bin/env bash
# Execute only in the existing URRC operator workspace. No new workspace.
set -euo pipefail
TEAM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$TEAM_DIR/../../.." && pwd)"
[ -f "$ROOT/race_v24.sh" ] || { echo "Official URRC workspace required: $ROOT" >&2; exit 2; }
cd "$ROOT"
source /opt/ros/jazzy/setup.bash
echo "[TEAM08] register and build native replay plugin"
bash race_v24.sh sync
bash race_v24.sh prepare
source "$ROOT/race/install/setup.bash"
LIB="$ROOT/race/install/team08_race/lib/libteam08_pose.so"
[ -f "$LIB" ] || { echo "Gazebo TEAM08 plugin missing: $LIB" >&2; exit 2; }
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/race/install/team08_race/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
python3 - <<'PY'
from pathlib import Path
import sys
root=Path.cwd()
sys.path.insert(0,str(root/"race/scripts"))
from preview_visual_v19 import ensure_assets
ensure_assets(root,"monaco")
PY
echo "[TEAM08] Monaco vehicles grid 1; input READY on terminal after GUI loads"
exec bash race_v24.sh rehearse vehicles --track monaco --team team_08 --seconds 300 --start-mode camera
