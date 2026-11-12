#!/usr/bin/env bash
set -Eeuo pipefail
# One-command stage-5 integrated local test. No edits to official map files by TEAM11.
TRACK="${1:-monza}"
GRID="${2:-1}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *) echo "Invalid track: $TRACK" >&2; exit 2;; esac
if ! [[ "$GRID" =~ ^([1-9]|10|11)$ ]]; then echo "Grid must be 1..11" >&2; exit 2; fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TEAM="$ROOT/src/team_11"
OFFICIAL="${URRC_OFFICIAL_WS:-$HOME/urrc_f1_ws}"
if [[ ! -f "$OFFICIAL/run_track.sh" ]]; then echo "Missing official launcher: $OFFICIAL/run_track.sh" >&2; exit 2; fi
if [[ ! -f "/opt/ros/jazzy/setup.bash" ]]; then echo "ROS 2 Jazzy not installed" >&2; exit 2; fi
for cmd in gz ros2 colcon python3; do command -v "$cmd" >/dev/null || { echo "Missing: $cmd" >&2; exit 2; }; done
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
source /opt/ros/jazzy/setup.bash
LOG="$ROOT/log/team11_stage5"
mkdir -p "$LOG"
PIDS=()
cleanup() {
  echo "[TEAM11] Stopping test processes"
  for pid in "${PIDS[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
echo "[1/6] Build team_11"
cd "$ROOT"
colcon build --packages-select team_11 --symlink-install >"$LOG/build.log" 2>&1 || { cat "$LOG/build.log"; exit 1; }
source "$ROOT/install/setup.bash"
echo "[2/6] Open official $TRACK (official asset preparation is handled by its own launcher)"
bash "$OFFICIAL/run_track.sh" "$TRACK" --view grid >"$LOG/world.log" 2>&1 &
WORLD_PID=$!; PIDS+=("$WORLD_PID")
echo "[TEAM11] Waiting for Gazebo /world/$TRACK/create"
READY=0
for i in $(seq 1 90); do
  if ! kill -0 "$WORLD_PID" 2>/dev/null; then echo "World stopped; see $LOG/world.log" >&2; tail -40 "$LOG/world.log"; exit 1; fi
  if gz service -l 2>/dev/null | grep -Fxq "/world/$TRACK/create"; then READY=1; break; fi
  sleep 1
done
if (( READY == 0 )); then echo "Gazebo create service unavailable; see $LOG/world.log" >&2; exit 1; fi
echo "[3/6] Start ROS to Gazebo bridge"
ros2 run ros_gz_bridge parameter_bridge "/model/team_11/cmd_vel_11@geometry_msgs/msg/Twist]gz.msgs.Twist" >"$LOG/bridge.log" 2>&1 &
PIDS+=("$!")
sleep 2
echo "[4/6] Spawn at official grid $GRID; local +X points toward starting signal"
python3 "$TEAM/tests/stage5_spawn.py" --official "$OFFICIAL" --track "$TRACK" --grid "$GRID" >"$LOG/spawn.log" 2>&1 || { cat "$LOG/spawn.log"; exit 1; }
sleep 2
echo "[5/6] Start manual test safety adapter"
ros2 run team_11 team11_stage5_adapter --ros-args -p test_bypass_gate:=true >"$LOG/adapter.log" 2>&1 &
ADAPTER_PID=$!; PIDS+=("$ADAPTER_PID")
sleep 2
if ! kill -0 "$ADAPTER_PID" 2>/dev/null; then echo "Adapter failed" >&2; cat "$LOG/adapter.log"; exit 1; fi
echo "[6/6] Test forward movement for 1 second (then STOP)"
ros2 run team_11 team11_stage5_manual --sequence straight --seconds 1 >"$LOG/drive.log" 2>&1 || { cat "$LOG/drive.log"; exit 1; }
echo "[TEAM11] Test completed. White lamps should move toward start signal; red lamps mark rear."
echo "[TEAM11] View GUI and logs: $LOG"
echo "[TEAM11] Press Ctrl+C to stop the test."
wait "$WORLD_PID" || true
