#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")"
colcon build --base-paths src
