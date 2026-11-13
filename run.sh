#!/usr/bin/env bash
set -Eeuo pipefail
# Single command: bash run.sh monza 1
TRACK="${1:-monza}"
LANE="${2:-1}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *) echo "Unknown track" >&2; exit 2;; esac
case "$LANE" in 1|2|3) ;; *) echo "Lane must be 1, 2 or 3" >&2; exit 2;; esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PKG="$ROOT/src/urrc_track_gazebo"
OFFICIAL="${URRC_OFFICIAL_WS:-$HOME/urrc_f1_ws}"
LOG="$ROOT/log/runs/${TRACK}_lane${LANE}/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG"
export GZ_PARTITION="${GZ_PARTITION:-urrc_team11_fixed_$(id -u)}"
export GZ_SIM_RESOURCE_PATH="$PKG/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export PYTHONUNBUFFERED=1
set +u
source /opt/ros/jazzy/setup.bash
set -u
ROS_PYTHON=/usr/bin/python3
if ! "$ROS_PYTHON" -c 'import rclpy; import nav_msgs.msg; import geometry_msgs.msg'; then
  echo "ROS 2 Jazzy Python unavailable" >&2; exit 3
fi
if ! command -v gz >/dev/null || ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 ||
   ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "Gazebo, ros_gz_bridge or ros_gz_sim missing" >&2; exit 3
fi
echo "[1/7] Import or verify six official tracks, signals and meshes"
if ! "$ROS_PYTHON" import_6_tracks.py --official "$OFFICIAL" --verify-only >"$LOG/import_check.log" 2>&1; then
  "$ROS_PYTHON" import_6_tracks.py --official "$OFFICIAL" >"$LOG/import.log" 2>&1 || {
    tail -40 "$LOG/import.log";exit 4;
  }
fi
echo "[2/7] Generate six tracks x three fixed coordinate routes"
"$ROS_PYTHON" generate_routes.py >"$LOG/routes.log" 2>&1 || {
  tail -50 "$LOG/routes.log";exit 4;
}
WORLD_PID="" BRIDGE_PID="" DRIVER_PID=""
GATE="$LOG/lights_out.ready"
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$DRIVER_PID" ]]; then kill -TERM "$DRIVER_PID" 2>/dev/null || true; fi
  if [[ -n "$BRIDGE_PID" ]]; then kill -TERM "$BRIDGE_PID" 2>/dev/null || true; fi
  if [[ -n "$WORLD_PID" ]]; then
    kill -INT "$WORLD_PID" 2>/dev/null || true
    sleep .5
    kill -TERM "$WORLD_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
if gz service -l 2>/dev/null | grep -Fxq "/world/$TRACK/create"; then
  echo "Existing $TRACK Gazebo world detected. Close previous instance." >&2;exit 4
fi
echo "[3/7] Open $TRACK race world and GUI"
gz sim -r -v 3 --gui-config "$PKG/worlds/${TRACK}_grid.gui.config" \
  "$PKG/worlds/${TRACK}_race.sdf" >"$LOG/gazebo.log" 2>&1 &
WORLD_PID=$!
READY=0
for _ in $(seq 1 90); do
  if ! kill -0 "$WORLD_PID" 2>/dev/null; then
    tail -50 "$LOG/gazebo.log";exit 5
  fi
  if gz service -l 2>/dev/null | grep -Fxq "/world/$TRACK/create"; then READY=1;break;fi
  sleep 1
done
if [[ "$READY" -ne 1 ]]; then echo "Gazebo service timeout" >&2;exit 5;fi
echo "[4/7] Spawn vehicle at exact official grid 1 centre"
read -r GX GY GZ GYAW < <("$ROS_PYTHON" - "$TRACK" "$PKG" <<'PY'
import json,sys
from pathlib import Path
track,pkg=sys.argv[1:]
meta=json.loads((Path(pkg)/"tracks"/"processed"/f"{track}.json").read_text())
g=next(p for p in meta["grid"] if int(p["position"])==1)
print(g["x"],g["y"],float(g["road_z"])+.09,g["yaw"])
PY
)
ros2 run ros_gz_sim create --world "$TRACK" \
  --file "$ROOT/src/team_11/models/team11_yellow_car/model.sdf" \
  --name team_11 --x "$GX" --y "$GY" --z "$GZ" -Y "$GYAW" \
  >"$LOG/spawn.log" 2>&1 || { tail -45 "$LOG/spawn.log";exit 6; }
echo "[5/7] Bridge true Gazebo model odometry and steering"
ros2 run ros_gz_bridge parameter_bridge \
  "/model/team_11/cmd_vel_11@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/model/team_11/odometry_11@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  >"$LOG/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then tail -45 "$LOG/bridge.log";exit 6;fi
echo "[6/7] Ready fixed lane $LANE (no lane changes)"
"$ROS_PYTHON" fixed_route_drive.py --track "$TRACK" --lane "$LANE" \
  --start-file "$GATE" >"$LOG/drive.log" 2>&1 &
DRIVER_PID=$!
sleep 2
if ! kill -0 "$DRIVER_PID" 2>/dev/null; then tail -45 "$LOG/drive.log";exit 7;fi
echo "[7/7] Official five-red signal; START only when all lamps turn off"
if ! timeout -s INT -k 3 110 "$ROS_PYTHON" \
  "$PKG/scripts/traffic_lights_v10.py" --track "$TRACK" --timeout 80 \
  >"$LOG/lights.log" 2>&1; then
  echo "FAIL: signal not confirmed; driving disabled" >&2
  tail -50 "$LOG/lights.log" >&2;exit 8
fi
if ! grep -q 'LIGHTS OUT' "$LOG/lights.log"; then
  echo "FAIL: no all-off confirmation" >&2;exit 8
fi
touch "$GATE"
echo "START: $TRACK, lane $LANE, initial grid 1, 2m/s cap"
set +e
wait "$DRIVER_PID"
RESULT=$?
set -e
DRIVER_PID=""
cat "$ROOT/log/fixed_routes/${TRACK}_lane${LANE}.json" 2>/dev/null || true
echo "RUN FINISHED: $LOG (exit $RESULT)"
exit "$RESULT"
