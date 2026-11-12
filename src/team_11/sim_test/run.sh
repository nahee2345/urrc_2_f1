#!/usr/bin/env bash
# Self-contained TEAM11 ROS + Gazebo test using bundled map CSV only.
set -euo pipefail
team_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ws_root="$(cd "$team_root/../.." && pwd)"
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS Jazzy installation not found" >&2
  exit 2
fi
if ! command -v gz >/dev/null; then
  echo "Gazebo Harmonic 'gz' executable not found" >&2
  exit 2
fi
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
exec python3 "$team_root/sim_test/run_replay.py" "$@"
