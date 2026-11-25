#!/usr/bin/env bash
set -eo pipefail

cd ~/urrc_2
source /opt/ros/jazzy/setup.bash
source ~/team05_runtime/install/setup.bash

bash race_v24.sh check

exec bash race_v24.sh rehearse qualifying \
  --team team_05 \
  --track monaco \
  --seconds 2400 \
  --start-mode signal
