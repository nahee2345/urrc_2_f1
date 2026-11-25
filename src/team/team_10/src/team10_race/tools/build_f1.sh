#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
set +u
source /opt/ros/jazzy/setup.bash
set -u
bash src/team/team_10/tools/build_contact_recorder.sh
colcon --log-base log_f1 build --base-paths src/team/team_10 --build-base build_f1 --install-base install_f1 --symlink-install
