#!/usr/bin/env bash
set -Eeuo pipefail
# ONE COMMAND: bash run.sh monza 1 [speed_mps] or bash run.sh battle monza
# Optional 3-car S-curve/overtake/defense trial. Existing fixed-lane runs unchanged.
if [[ "${1:-}" == "battle" ]]; then
  shift
  exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_battle.sh" "${1:-monza}"
fi
# 1=right 2=centre 3=left; all cars spawn at official grid-1 centre.
TRACK="${1:-monza}"
LANE="${2:-1}"
SPEED="${3:-10}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *)
  echo "TRACK: monza spa monaco cota hungaroring baku" >&2; exit 2 ;;
esac
case "$LANE" in 1|2|3) ;; *)
  echo "LANE: 1=right 2=centre 3=left" >&2; exit 2 ;;
esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PKG="$ROOT/src/urrc_track_gazebo"
OFFICIAL="${URRC_OFFICIAL_WS:-$HOME/urrc_f1_ws}"
LOG="$ROOT/log/kinematic/${TRACK}_lane${LANE}/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG"
export GZ_PARTITION="${GZ_PARTITION:-urrc_team11_replay_$(id -u)}"
export PYTHONUNBUFFERED=1
export GZ_SIM_RESOURCE_PATH="$PKG/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

if ! command -v gz >/dev/null || ! command -v cmake >/dev/null; then
  echo "Gazebo Harmonic and cmake are required" >&2; exit 3
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "ros_gz_sim missing (ROS2 Jazzy)" >&2; exit 3
fi

echo "[1/6] Import six official maps (read-only source), five-light logic"
if ! /usr/bin/python3 import_6_tracks.py --official "$OFFICIAL" --verify-only \
    >"$LOG/import_check.log" 2>&1; then
  /usr/bin/python3 import_6_tracks.py --official "$OFFICIAL" \
    >"$LOG/import.log" 2>&1 || { tail -50 "$LOG/import.log"; exit 4; }
fi
echo "[2/6] Prepare all 18 fixed coordinate routes"
/usr/bin/python3 generate_routes.py >"$LOG/routes.log" 2>&1 || {
  tail -50 "$LOG/routes.log";exit 4;
}

echo "[3/6] Compile the original pace-car-style SetWorldPoseCmd plugin"
cmake -S "$ROOT/kinematic" -B "$ROOT/kinematic/build" \
  -DCMAKE_BUILD_TYPE=Release >"$LOG/cmake.log" 2>&1 || {
    tail -70 "$LOG/cmake.log";exit 5;
  }
cmake --build "$ROOT/kinematic/build" --parallel 2 >>"$LOG/cmake.log" 2>&1 || {
  tail -70 "$LOG/cmake.log";exit 5;
}
PLUGIN="$ROOT/kinematic/build/libteam11_fixed_replay.so"
test -f "$PLUGIN" || { echo "Plugin library not built" >&2;exit 5; }
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(dirname "$PLUGIN")${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

echo "[4/6] Prepare exact grid-1 spawn and lane-$LANE one-lap replay"
OUT="$LOG/prepared"
/usr/bin/python3 "$ROOT/kinematic/prepare.py" \
  --track "$TRACK" --lane "$LANE" --speed "$SPEED" \
  --plugin "$PLUGIN" --output "$OUT" >"$LOG/prepare.log" 2>&1 || {
    tail -70 "$LOG/prepare.log";exit 6;
  }
read -r GX GY GZ GYAW < <(/usr/bin/python3 - "$OUT/manifest.json" <<'PY'
import json,sys
data=json.load(open(sys.argv[1]))
print(*data["grid_xyz_yaw"])
PY
)
export GZ_SIM_RESOURCE_PATH="$OUT/resources/urrc_track_gazebo/models:$GZ_SIM_RESOURCE_PATH"
WORLD="$OUT/resources/urrc_track_gazebo/worlds/${TRACK}_fixed.sdf"
LIGHTS="$OUT/resources/urrc_track_gazebo/scripts/traffic_lights_v10.py"
MODEL="$OUT/replay_vehicle.sdf"
TOPIC="/urrc/$TRACK/fixed/start"
WORLD_PID=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$WORLD_PID" ]]; then
    kill -INT "$WORLD_PID" 2>/dev/null || true
    sleep 1
    kill -TERM "$WORLD_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
if gz service -l 2>/dev/null | grep -Fxq "/world/$TRACK/create"; then
  echo "An existing $TRACK world is running; close it before replay" >&2
  exit 7
fi

echo "[5/6] Open official-copy $TRACK world with coordinate replay plugin"
gz sim -r -v 3 --gui-config "$PKG/worlds/${TRACK}_grid.gui.config" \
  "$WORLD" >"$LOG/gazebo.log" 2>&1 &
WORLD_PID=$!
READY=0
for _ in $(seq 1 100); do
  if ! kill -0 "$WORLD_PID" 2>/dev/null; then
    tail -60 "$LOG/gazebo.log";exit 7
  fi
  if gz topic -l 2>/dev/null | grep -Fxq "$TOPIC/ready"; then
    READY=1;break
  fi
  sleep 1
done
if [[ "$READY" -ne 1 ]]; then
  echo "FAIL: kinematic plugin never advertised readiness" >&2
  tail -65 "$LOG/gazebo.log";exit 7
fi

ros2 run ros_gz_sim create --world "$TRACK" \
  --file "$MODEL" --name team_11 \
  --x "$GX" --y "$GY" --z "$GZ" -Y "$GYAW" \
  >"$LOG/spawn.log" 2>&1 || {
    tail -65 "$LOG/spawn.log";exit 8;
  }
echo "[6/6] Five sequential red lamps; do not drive before all-off"
if ! timeout -s INT -k 3 110 /usr/bin/python3 "$LIGHTS" \
   --track "$TRACK" --timeout 80 >"$LOG/lights.log" 2>&1; then
  echo "FAIL: original light script; car stays stationary" >&2
  tail -60 "$LOG/lights.log";exit 9
fi
if ! grep -q '^LIGHTS OUT .*model_remove=single_request' "$LOG/lights.log"; then
  echo "FAIL: all-off service confirmation missing" >&2;exit 9
fi
for _ in 1 2 3; do
  if ! gz topic -t "$TOPIC" -m gz.msgs.Boolean -p 'data: true' \
    >"$LOG/start.log" 2>&1; then
    tail -40 "$LOG/start.log";exit 10
  fi
  sleep .1
done
echo "START: $TRACK, fixed lane $LANE, grid 1, $SPEED m/s"
echo "Direct Gazebo SetWorldPoseCmd coordinate replay; no steering, lane changes, or physics control"
echo "Gazebo running. One-lap completion will be reported."
SEEN=0
# For batch mode, completion automatically closes this run and advances
# to the next track. Interactive mode keeps the Gazebo window visible.
# Safety timeout prevents a stuck Gazebo from blocking all 18 runs.
MAX_REPLAY_WALL_S=${URRC_REPLAY_TIMEOUT_S:-300}
STARTED_AT=$(date +%s)
while kill -0 "$WORLD_PID" 2>/dev/null; do
  if [[ "$SEEN" == 0 ]] && grep -q '\[FIXED ROUTE\] PASS_LAP' "$LOG/gazebo.log"; then
    grep '\[FIXED ROUTE\] PASS_LAP' "$LOG/gazebo.log" | tail -1
    SEEN=1
    if [[ "${URRC_BATCH:-0}" == 1 ]]; then
      echo "BATCH PASS $TRACK lane $LANE; closing world for next test"
      break
    fi
  fi
  if [[ "${URRC_BATCH:-0}" == 1 ]] && (( $(date +%s) - STARTED_AT > MAX_REPLAY_WALL_S )); then
    echo "BATCH FAIL: no confirmed lap within ${MAX_REPLAY_WALL_S}s; $LOG/gazebo.log" >&2
    exit 12
  fi
  sleep 1
done
if [[ "$SEEN" == 0 ]]; then
  echo "FAIL: Gazebo stopped without a confirmed lap; inspect $LOG/gazebo.log" >&2
  exit 11
fi
echo "PASS: $TRACK lane $LANE at $SPEED m/s; logs: $LOG"
