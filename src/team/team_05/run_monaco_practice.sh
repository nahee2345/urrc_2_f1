#!/usr/bin/env bash
set -eo pipefail

team_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="/opt/ros/jazzy/setup.bash"
overlay_setup="$HOME/team05_runtime/install/setup.bash"

if [[ ! -r "$ros_setup" ]]; then
    echo "ROS 2 Jazzy setup not found: $ros_setup" >&2
    exit 1
fi

source "$ros_setup"
if [[ -r "$overlay_setup" ]]; then
    source "$overlay_setup"
fi

export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
exec python3 -u "$team_dir/monaco_practice.py"
