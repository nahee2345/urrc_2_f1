#!/usr/bin/env bash
set -eo pipefail
TEAM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
role="${1:-competition}"
if [[ $# -gt 0 ]]; then shift; fi
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$TEAM_ROOT/install/local_setup.bash" ]]; then bash "$TEAM_ROOT/build.sh"; fi
source "$TEAM_ROOT/install/local_setup.bash"
case "$role" in
  1|map) exec python3 -m team_01_race.practice 1 --team-root "$TEAM_ROOT" "$@" ;;
  2) exec python3 -m team_01_race.practice 2 --team-root "$TEAM_ROOT" "$@" ;;
  3) exec python3 -m team_01_race.practice 3 --team-root "$TEAM_ROOT" "$@" ;;
  competition|all) exec ros2 launch team_01_race competition.launch.py "$@" ;;
  sensors) exec ros2 launch team_01_race sensors.launch.py "$@" ;;
  autonomy) exec ros2 launch team_01_race race.launch.py bridge_sensors:=false require_official_start:=false "$@" ;;
  4|5|6|7|8|9) exec python3 -m team_01_race.practice "$role" --team-root "$TEAM_ROOT" "$@" ;;
  *) echo "사용법: bash run.sh {1 [monza|monaco|hungaroring|random]|2|3|competition|sensors|autonomy|4|5|6|7|8|9}" >&2; exit 2 ;;
esac
