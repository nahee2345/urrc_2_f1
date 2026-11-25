#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
if [ ! -f install/local_setup.bash ]; then
  bash build.sh
fi
source install/local_setup.bash
exec ros2 launch team05_race race.launch.py "$@"
