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
LOG="${TEAM11_STAGE7_LOG:-$ROOT/log/team11_stage7}"
mkdir -p "$LOG"
if [[ "${TEAM11_STAGE7_CAPTURE_IMAGES:-1}" == "1" ]]; then
  export TEAM11_CAMERA_DIAG_DIR="$LOG"
else
  unset TEAM11_CAMERA_DIAG_DIR
fi
rm -f "$LOG/result.txt"
# Avoid confusing a previous passing attempt with a new failed run.
: >"$LOG/camera.log"
: >"$LOG/drive.log"
: >"$LOG/lights.log"
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  rm -f "$LOG/signal_capture.active"
  echo "[TEAM11] Stopping stage7 processes"
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap "exit 130" INT TERM
cd "$ROOT"
echo "[1/8] Build team_11"
if [[ "${TEAM11_STAGE7_SKIP_BUILD:-0}" != "1" ]]; then
  colcon build --packages-select team_11 --symlink-install >"$LOG/build.log" 2>&1 || { tail -80 "$LOG/build.log"; exit 1; }
else
  echo "[TEAM11] Reusing prior team_11 build"
fi
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
echo "[2/8] Prepare camera detector, gated adapter and test driver before vehicle spawn"
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
echo "[3/8] Spawn car in official grid $GRID AFTER ROS nodes are running"
python3 "$TEAM/tests/stage5_spawn.py" --track "$TRACK" --grid "$GRID" --official "$OFFICIAL" >"$LOG/spawn.log" 2>&1 || { cat "$LOG/spawn.log"; exit 1; }
TOPIC_READY=0
for i in $(seq 1 50); do
  if gz topic -l 2>/dev/null | grep -Fxq "/camera_11"; then TOPIC_READY=1; break; fi
  sleep .4
done
if [[ "$TOPIC_READY" -ne 1 ]]; then
  echo "[TEAM11] FAIL: Gazebo camera topic missing after spawn" >&2
  gz topic -l >"$LOG/gz_topics.log" 2>&1 || true
  tail -40 "$LOG/spawn.log" "$LOG/world.log" >&2 || true
  exit 4
fi
echo "[4/8] Attach camera bridge after vehicle and Gazebo camera topic exist"
ros2 run ros_gz_bridge parameter_bridge "/model/team_11/cmd_vel_11@geometry_msgs/msg/Twist]gz.msgs.Twist" "/camera_11@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/camera_zoom_11@sensor_msgs/msg/Image[gz.msgs.Image" >"$LOG/bridge.log" 2>&1 &
BRIDGE_PID=$!; PIDS+=("$BRIDGE_PID")
sleep 3
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then echo "Bridge failed" >&2; cat "$LOG/bridge.log"; exit 1; fi
echo "[4/8] Wait for onboard camera frames after spawning. Controls stay STOP."
echo "[TEAM11] Waiting for first real camera image after vehicle spawn"
CAMERA_READY=0
for i in $(seq 1 90); do
  if grep -q "FRAME_READY source=" "$LOG/camera.log"; then CAMERA_READY=1; break; fi
  if ! kill -0 "$WORLD_PID" 2>/dev/null || ! kill -0 "$CAMERA_PID" 2>/dev/null; then break; fi
  sleep .5
done
if [[ "$CAMERA_READY" -ne 1 ]]; then
  echo "[TEAM11] FAIL: Camera did not produce a frame after spawn; lights will not run." >&2
  tail -40 "$LOG/camera.log" "$LOG/bridge.log" "$LOG/world.log" >&2 || true
  exit 4
fi
echo "[TEAM11] Camera ready; signal sequence may start"
echo "[5/8] Trigger OFFICIAL five-light sequence (4+ ON -> ALL OFF)"
rm -f "$LOG"/signal_*.png
if [[ "${TEAM11_STAGE7_CAPTURE_IMAGES:-1}" == "1" ]]; then
  touch "$LOG/signal_capture.active"
fi
python3 "$PKG/scripts/traffic_lights_v10.py" --track "$TRACK" --timeout 35 >"$LOG/lights.log" 2>&1 || { echo "Official lights failed" >&2; tail -60 "$LOG/lights.log"; exit 1; }
rm -f "$LOG/signal_capture.active"
echo "[6/8] Observe camera-gated one-second motion"
sleep 3
if grep -q "START: 4+ red lamps observed, now ALL OFF" "$LOG/camera.log" &&
   grep -q "FINISHED: One-second pulse complete" "$LOG/drive.log" &&
   grep -q "LIGHTS OUT" "$LOG/lights.log" &&
   ! grep -q "STOP: Camera start permission lost" "$LOG/drive.log"; then
  echo "PASS" > "$LOG/result.txt"
  echo "[TEAM11] PASS: camera detected red lights then off, followed by 1-second forward pulse"
else
  echo "FAIL" > "$LOG/result.txt"
  echo "[TEAM11] NOT VERIFIED: detector did not confirm start; vehicle should remain stopped"
  tail -30 "$LOG/camera.log" || true
  echo "[TEAM11] Official sequence duration and errors:"
  tail -20 "$LOG/lights.log" || true
  tail -15 "$LOG/drive.log" || true
fi
echo "[TEAM11] Logs: $LOG"
echo "[TEAM11] Onboard camera: $LOG/camera_before.png"
echo "[TEAM11] Actual lamp-on sequence: $LOG/signal_00.png .. signal_*.png"
echo "[TEAM11] Press Ctrl+C when finished viewing the Gazebo window."
if [[ "${TEAM11_STAGE7_AUTO_EXIT:-0}" == "1" ]]; then
  [[ "$(cat "$LOG/result.txt")" == "PASS" ]] || exit 7
  exit 0
fi
wait "$WORLD_PID" || true
