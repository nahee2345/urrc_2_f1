#!/usr/bin/env bash
set -Eeuo pipefail
# 11-grid batch: keep ONE official world and bridge alive, replace ONLY team_11 vehicle.
TRACK="${1:-monza}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *) echo "Unsupported track" >&2; exit 2;; esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TEAM="$ROOT/src/team_11"
OFFICIAL="${URRC_OFFICIAL_WS:-$HOME/urrc_f1_ws}"
PKG="$OFFICIAL/src/urrc_track_gazebo"
LOGROOT="$ROOT/log/team11_stage7_all_grids/$TRACK"
mkdir -p "$LOGROOT"
SUMMARY="$LOGROOT/summary.csv"
printf 'track,grid,result,detail\n' >"$SUMMARY"
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$ROOT"
echo "[TEAM11] Building once; one Gazebo world reused for all 11 grids"
colcon build --packages-select team_11 --symlink-install >"$LOGROOT/build.log" 2>&1 || { tail -40 "$LOGROOT/build.log"; exit 2; }
set +u
source "$ROOT/install/setup.bash"
set -u
if gz service -l 2>/dev/null | grep -Fxq "/world/$TRACK/create"; then
  echo "[TEAM11] An existing $TRACK world is running. Close Gazebo before batch testing." >&2
  exit 2
fi
WORLD_PID=""
BRIDGE_PID=""
GRID_PIDS=()
stop_grid() {
  for pid in "${GRID_PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${GRID_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then kill -KILL "$pid" 2>/dev/null || true; fi
    wait "$pid" 2>/dev/null || true
  done
  GRID_PIDS=()
}
remove_car() {
  # Official race manager uses type: MODEL, not numeric type: 2.
  # Gazebo removal is asynchronous; check the Boolean reply and allow physics
  # updates before creating another entity with exactly the same name.
  local answer=""
  answer="$(gz service -s "/world/$TRACK/remove" --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean --timeout 5000 \
    --req 'name: "team_11" type: MODEL' 2>&1)" || true
  printf '%s\n' "$answer" >>"$LOGROOT/remove.log"
  sleep 2
}
cleanup() {
  trap - EXIT INT TERM
  stop_grid
  remove_car
  if [[ -n "$BRIDGE_PID" ]]; then kill -TERM "$BRIDGE_PID" 2>/dev/null || true; fi
  if [[ -n "$WORLD_PID" ]]; then
    kill -INT "$WORLD_PID" 2>/dev/null || true
    sleep 1
    kill -TERM "$WORLD_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
echo "[TEAM11] Opening official world $TRACK"
bash "$TEAM/tests/stage5_world.sh" "$TRACK" >"$LOGROOT/world.log" 2>&1 &
WORLD_PID=$!
world_ready=0
for i in $(seq 1 90); do
  if ! kill -0 "$WORLD_PID" 2>/dev/null; then
    echo "[TEAM11] World stopped prematurely" >&2
    tail -30 "$LOGROOT/world.log"
    exit 2
  fi
  if gz service -l 2>/dev/null | grep -Fxq "/world/$TRACK/create"; then
    world_ready=1
    break
  fi
  sleep 1
done
if [[ "$world_ready" -ne 1 ]]; then echo "[TEAM11] World service timeout" >&2; exit 2; fi
ros2 run ros_gz_bridge parameter_bridge \
  "/model/team_11/cmd_vel_11@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/camera_11@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/camera_zoom_11@sensor_msgs/msg/Image[gz.msgs.Image" \
  >"$LOGROOT/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  tail -30 "$LOGROOT/bridge.log"; exit 2
fi
passed=0
failed=0
for GRID in $(seq 1 11); do
  echo
  echo "===== $TRACK GRID $GRID / 11 ====="
  DIR="$LOGROOT/grid_$(printf '%02d' "$GRID")"
  mkdir -p "$DIR"
  : >"$DIR/camera.log"
  : >"$DIR/drive.log"
  : >"$DIR/lights.log"
  stop_grid
  remove_car
  sleep 1
  spawned=0
  for attempt in 1 2 3; do
    if python3 "$TEAM/tests/stage5_spawn.py" --official "$OFFICIAL" --track "$TRACK" --grid "$GRID" >"$DIR/spawn_attempt_${attempt}.log" 2>&1; then
      spawned=1
      cp "$DIR/spawn_attempt_${attempt}.log" "$DIR/spawn.log"
      break
    fi
    echo "[TEAM11] Grid $GRID spawn attempt $attempt failed; checking removal"
    cat "$DIR/spawn_attempt_${attempt}.log" >&2
    remove_car
    sleep 2
  done
  if [[ "$spawned" -ne 1 ]]; then
    echo "[TEAM11] Grid $GRID spawn FAIL after 3 attempts"
    printf '%s,%d,FAIL,spawn_after_3_attempts\n' "$TRACK" "$GRID" >>"$SUMMARY"
    failed=$((failed+1))
    continue
  fi
  # Reset the camera light gate for every grid, without restarting Gazebo.
  TEAM11_CAMERA_DIAG_DIR="" ros2 run team_11 team11_signal_camera >"$DIR/camera.log" 2>&1 &
  GRID_PIDS+=("$!")
  ros2 run team_11 team11_stage5_adapter >"$DIR/adapter.log" 2>&1 &
  GRID_PIDS+=("$!")
  ros2 run team_11 team11_stage7_drive >"$DIR/drive.log" 2>&1 &
  GRID_PIDS+=("$!")
  camera_ready=0
  for i in $(seq 1 40); do
    if grep -q "DIAG frames=" "$DIR/camera.log" 2>/dev/null; then camera_ready=1; break; fi
    sleep .5
  done
  if [[ "$camera_ready" -ne 1 ]]; then
    echo "[TEAM11] Grid $GRID camera timeout"
    printf '%s,%d,FAIL,no_camera_frames\n' "$TRACK" "$GRID" >>"$SUMMARY"
    failed=$((failed+1))
    continue
  fi
  echo "[TEAM11] Camera ready; triggering official five red lights"
  if ! timeout -s INT -k 5s 70s python3 "$PKG/scripts/traffic_lights_v10.py" --track "$TRACK" --timeout 35 >"$DIR/lights.log" 2>&1; then
    echo "[TEAM11] Grid $GRID official lights failed or timed out"
    printf '%s,%d,FAIL,official_lights\n' "$TRACK" "$GRID" >>"$SUMMARY"
    failed=$((failed+1))
    continue
  fi
  sleep 3
  if grep -q "START: 4+ red lamps observed, now ALL OFF" "$DIR/camera.log" &&
     grep -q "FINISHED: One-second pulse complete" "$DIR/drive.log" &&
     grep -q "LIGHTS OUT" "$DIR/lights.log" &&
     ! grep -q "STOP: Camera start permission lost" "$DIR/drive.log"; then
    printf '%s,%d,PASS,camera_lights_out_forward\n' "$TRACK" "$GRID" >>"$SUMMARY"
    echo "[TEAM11] GRID $GRID PASS"
    passed=$((passed+1))
  else
    max_lamps="$(grep -o 'peak_lamps=[0-9]*' "$DIR/camera.log" | tail -1 || true)"
    reason="unknown"
    if ! grep -q "ARMED source=" "$DIR/camera.log"; then
      reason="no_stable_four_red"
    elif ! grep -q "START: 4+ red lamps observed, now ALL OFF" "$DIR/camera.log"; then
      reason="red_seen_but_no_all_off"
    elif grep -q "STOP: Camera start permission lost" "$DIR/drive.log"; then
      reason="permission_lost"
    elif ! grep -q "FINISHED: One-second pulse complete" "$DIR/drive.log"; then
      reason="forward_not_finished"
    else
      reason="unconfirmed_light_sequence"
    fi
    echo "[TEAM11] GRID $GRID FAIL: $reason ${max_lamps:-no_camera_diagnostics}"
    printf '%s,%d,FAIL,%s_%s\n' "$TRACK" "$GRID" "$reason" "${max_lamps:-unknown}" >>"$SUMMARY"
    failed=$((failed+1))
  fi
done
stop_grid
remove_car
echo
echo "[TEAM11] Results: PASS $passed / 11, FAIL $failed / 11"
echo "[TEAM11] Summary: $SUMMARY"
cat "$SUMMARY"
if [[ "$failed" -gt 0 ]]; then exit 1; fi
