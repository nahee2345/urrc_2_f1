#!/usr/bin/env bash
set -Eeuo pipefail
# Stage 7: official track + vision-only five red lights then all-off.
TRACK="${1:-monza}"
GRID="${2:-1}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *) echo "Invalid track" >&2; exit 2;; esac
if ! [[ "$GRID" =~ ^([1-9]|10|11)$ ]]; then echo "Grid 1..11 only" >&2; exit 2; fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TEAM="$ROOT/src/team_11"
OFFICIAL="${URRC_OFFICIAL_WS:-$HOME/urrc_f1_ws}"
PKG="$OFFICIAL/src/urrc_track_gazebo"
if [[ ! -f "$PKG/scripts/traffic_lights_v10.py" ]]; then echo "Official traffic light script not found: $PKG" >&2; exit 2; fi
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
set +u
source /opt/ros/jazzy/setup.bash
set -u
LOG="$ROOT/log/team11_stage7"
mkdir -p "$LOG"
export TEAM11_CAMERA_DIAG_DIR="$LOG"
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  echo "[TEAM11] Stopping stage7 processes"
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap "exit 130" INT TERM
cd "$ROOT"
echo "[1/8] Build team_11"
colcon build --packages-select team_11 --symlink-install >"$LOG/build.log" 2>&1 || { tail -80 "$LOG/build.log"; exit 1; }
set +u
source "$ROOT/install/setup.bash"
set -u
echo "[2/8] Open official track WITHOUT automatic light sequence"
# Separate stage5 world runner opens the original race SDF but never starts traffic_lights_v10.
bash "$TEAM/tests/stage5_world.sh" "$TRACK" >"$LOG/world.log" 2>&1 &
WORLD_PID=$!; PIDS+=("$WORLD_PID")
READY=0
for i in $(seq 1 90); do
  if ! kill -0 "$WORLD_PID" 2>/dev/null; then tail -60 "$LOG/world.log"; exit 1; fi
  if gz service -l 2>/dev/null | grep -Fxq "/world/$TRACK/create"; then READY=1; break; fi
  sleep 1
done
if (( READY == 0 )); then echo "Gazebo create service timeout" >&2; exit 1; fi
echo "[3/8] Spawn car in official grid $GRID"
python3 "$TEAM/tests/stage5_spawn.py" --track "$TRACK" --grid "$GRID" --official "$OFFICIAL" >"$LOG/spawn.log" 2>&1 || { cat "$LOG/spawn.log"; exit 1; }
echo "[4/8] Start ROS bridges for control and RGB camera"
ros2 run ros_gz_bridge parameter_bridge "/model/team_11/cmd_vel_11@geometry_msgs/msg/Twist]gz.msgs.Twist" "/camera_11@sensor_msgs/msg/Image[gz.msgs.Image" >"$LOG/bridge.log" 2>&1 &
BRIDGE_PID=$!; PIDS+=("$BRIDGE_PID")
sleep 3
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then echo "Bridge failed" >&2; cat "$LOG/bridge.log"; exit 1; fi
echo "[5/8] Start camera detector, gated vehicle adapter and one-second test driver"
ros2 run team_11 team11_signal_camera >"$LOG/camera.log" 2>&1 &
CAMERA_PID=$!; PIDS+=("$CAMERA_PID")
ros2 run team_11 team11_stage5_adapter >"$LOG/adapter.log" 2>&1 &
ADAPTER_PID=$!; PIDS+=("$ADAPTER_PID")
ros2 run team_11 team11_stage7_drive >"$LOG/drive.log" 2>&1 &
DRIVE_PID=$!; PIDS+=("$DRIVE_PID")
sleep 4
for pid in "$CAMERA_PID" "$ADAPTER_PID" "$DRIVE_PID"; do
  if ! kill -0 "$pid" 2>/dev/null; then echo "ROS node failed" >&2; tail -50 "$LOG/camera.log" "$LOG/adapter.log" "$LOG/drive.log"; exit 1; fi
done
echo "[6/8] Camera and adapter are ready. All controls are currently STOP."
echo "[TEAM11] Camera diagnostics before lights:"
tail -8 "$LOG/camera.log" || true
if ! grep -q "DIAG frames=" "$LOG/camera.log"; then
  echo "[TEAM11] FAIL: Camera is not providing frames; do not start signals." >&2
  echo "[TEAM11] Check $LOG/bridge.log and /camera_11 QoS." >&2
  exit 4
fi
echo "[7/8] Trigger OFFICIAL five-light sequence (4+ ON -> ALL OFF)"
python3 "$PKG/scripts/traffic_lights_v10.py" --track "$TRACK" --timeout 35 >"$LOG/lights.log" 2>&1 || { echo "Official lights failed" >&2; tail -60 "$LOG/lights.log"; exit 1; }
echo "[8/8] Observe camera-gated one-second motion"
sleep 3
if grep -q "FINISHED: One-second pulse complete" "$LOG/drive.log"; then
  echo "[TEAM11] PASS: camera gate allowed a completed 1-second forward pulse"
else
  echo "[TEAM11] NOT VERIFIED: detector did not confirm start; vehicle should remain stopped"
  tail -30 "$LOG/camera.log" || true
  echo "[TEAM11] Official sequence duration and errors:"
  tail -20 "$LOG/lights.log" || true
  tail -15 "$LOG/drive.log" || true
fi
echo "[TEAM11] Logs: $LOG"
echo "[TEAM11] Onboard camera: $LOG/camera_before.png"
echo "[TEAM11] Red candidate: $LOG/camera_red_candidate.png (when observed)"
echo "[TEAM11] Press Ctrl+C when finished viewing the Gazebo window."
wait "$WORLD_PID" || true
