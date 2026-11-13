#!/usr/bin/env bash
set -Eeuo pipefail
# Practice-only physical lap; official race runner and start gate untouched.
TRACK="${1:-monza}"
GRID="${2:-1}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *) echo "Invalid track" >&2; exit 2;; esac
[[ "$GRID" =~ ^([1-9]|10|11)$ ]] || { echo "Invalid grid 1..11" >&2; exit 2; }
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TEAM="$ROOT/src/team_11"
OFFICIAL="${URRC_OFFICIAL_WS:-$HOME/urrc_f1_ws}"
LOG="$ROOT/log/team11_physical_lap/${TRACK}_grid_$(printf '%02d' "$GRID")"
mkdir -p "$LOG"
export TEAM11_PRACTICE_LIGHTWEIGHT=1
export TEAM11_PRACTICE_DIR="$LOG/practice"
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
export PYTHONUNBUFFERED=1
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$ROOT"
# ROS 2 Jazzy on Ubuntu 24.04 installs rclpy for the system Python 3.12.
# Avoid venv/pyenv/conda python that cannot see /opt/ros/jazzy packages.
ROS_PYTHON=/usr/bin/python3
if [[ ! -x "$ROS_PYTHON" ]]; then
  echo "[TEAM11] FAIL: /usr/bin/python3 not installed" >&2
  exit 2
fi
if ! "$ROS_PYTHON" -c 'import rclpy; import nav_msgs.msg; import geometry_msgs.msg' 2>"$LOG/ros_python_check.log"; then
  echo "[TEAM11] FAIL: ROS Jazzy Python imports unavailable using $ROS_PYTHON" >&2
  echo "[TEAM11] Check: source /opt/ros/jazzy/setup.bash; /usr/bin/python3 -c 'import rclpy'" >&2
  cat "$LOG/ros_python_check.log" >&2
  exit 2
fi
echo "[TEAM11] ROS Python OK: $ROS_PYTHON"
if gz service -l 2>/dev/null | grep -Fxq "/world/$TRACK/create"; then
  echo "Another $TRACK world is running; close it before practice" >&2
  exit 2
fi
echo "[TEAM11] Build and generate three lanes from official data"
colcon build --packages-select team_11 --symlink-install >"$LOG/build.log" 2>&1 || {
  tail -60 "$LOG/build.log"; exit 2;
}
set +u
source "$ROOT/install/setup.bash"
set -u
PYTHONPATH="$TEAM" python3 "$TEAM/routes.py" \
  --track-package "$OFFICIAL" --output "$LOG/routes" >"$LOG/routes.log" 2>&1 || {
    tail -40 "$LOG/routes.log"; exit 3;
  }
world_pid=""
bridge_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$bridge_pid" ]]; then kill -TERM "$bridge_pid" 2>/dev/null || true; fi
  if [[ -n "$world_pid" ]]; then
    kill -INT "$world_pid" 2>/dev/null || true
    sleep 1
    kill -TERM "$world_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
echo "[TEAM11] Start isolated lightweight world: no lights or spectator camera"
bash "$TEAM/tests/stage5_world.sh" "$TRACK" >"$LOG/world.log" 2>&1 &
world_pid=$!
ready=0
for i in $(seq 1 90); do
  if ! kill -0 "$world_pid" 2>/dev/null; then
    tail -60 "$LOG/world.log"; exit 4
  fi
  if gz service -l 2>/dev/null | grep -Fxq "/world/$TRACK/create"; then ready=1; break; fi
  sleep 1
done
if [[ "$ready" -ne 1 ]]; then echo "[TEAM11] FAIL: Gazebo world unavailable"; tail -40 "$LOG/world.log"; exit 4; fi
echo "[TEAM11] Spawn grid $GRID"
python3 "$TEAM/tests/stage5_spawn.py" --official "$OFFICIAL" --track "$TRACK" --grid "$GRID" \
  >"$LOG/spawn.log" 2>&1 || { tail -40 "$LOG/spawn.log"; exit 5; }
echo "[TEAM11] Bridge real Ackermann odometry and drive command"
ros2 run ros_gz_bridge parameter_bridge \
  "/model/team_11/cmd_vel_11@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/model/team_11/odometry_11@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  >"$LOG/bridge.log" 2>&1 &
bridge_pid=$!
sleep 2
if ! kill -0 "$bridge_pid" 2>/dev/null; then tail -60 "$LOG/bridge.log"; exit 6; fi
if [[ "${TEAM11_PRACTICE_FOLLOW:-1}" == "1" ]]; then
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
    --timeout 1500 --req 'data: "team_11"' >"$LOG/follow.log" 2>&1 || true
fi
echo "[TEAM11] Physical lap trial started; no start signal required."
set +e
PYTHONPATH="$TEAM" "$ROS_PYTHON" "$TEAM/tests/physical_lap_trial.py" \
  --track "$TRACK" --grid "$GRID" --routes "$LOG/routes" \
  --timeout "${TEAM11_LAP_TIMEOUT:-180}" --output "$LOG/result.json" \
  2>&1 | tee "$LOG/trial.log"
result="${PIPESTATUS[0]}"
set -e
echo "[TEAM11] Result: $LOG/result.json"
exit "$result"
