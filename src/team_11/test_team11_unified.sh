#!/usr/bin/env bash
# Run all standalone TEAM11 unified logic tests inside urrc_2_f1.
set -euo pipefail
team_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_root="$(cd "$team_root/../.." && pwd)"

echo "[TEAM11] Python syntax"
python3 -m compileall -q "$team_root/team11_race" "$team_root/three_lane"
echo "[TEAM11] Offline path, strategy, camera and LiDAR tests"
python3 -m unittest discover -s "$team_root/three_lane" -p 'test_*.py' -v

if [[ "${1:-}" == "--ros" ]]; then
  if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy was not found at /opt/ros/jazzy/setup.bash" >&2
    exit 2
  fi
  # ROS 2 / ament setup files assume unset environment variables are allowed.
  # Bash nounset (-u) must be disabled temporarily while sourcing them.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
  cd "$ws_root"
  colcon build --base-paths src/team_11 --packages-select team11_race --symlink-install
  set +u
  # shellcheck disable=SC1091
  source "$ws_root/install/setup.bash"
  set -u
  ros2 pkg prefix team11_race
  echo "[TEAM11] ROS 2 build PASS"
fi

echo "[TEAM11] Tests finished. Offline PASS is not Gazebo lap completion."
