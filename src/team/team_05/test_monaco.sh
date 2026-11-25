#!/usr/bin/env bash
set -e

cd ~/urrc_2

source /opt/ros/jazzy/setup.bash
source ~/track_runtime/install_v2/setup.bash
source ~/team05_runtime/install/local_setup.bash

export GZ_PARTITION="urrc_practice_$(id -u)"

echo "===== GAZEBO CONNECTION ====="

if ! gz service -l | grep -Fx '/world/monaco/create'; then
    echo "ERROR: MONACO NOT CONNECTED"
    exit 1
fi

echo "===== START SIGNAL CHECK ====="

grep -nE 'start_sequence|std_msgs|race_start|signal|traffic_light' \
    race/scripts/race_manager.py | head -n 100

echo "===== VEHICLE SPAWN ====="

if gz topic -l | grep -E '^/model/team_05/(cmd_vel|odometry)$'; then
    echo "TEAM05 ALREADY EXISTS"
else
    ros2 run ros_gz_sim create \
      -world monaco \
      -name team_05 \
      -file "$HOME/urrc_2/src/team/team_05/src/team05_race/models/car/model.sdf" \
      -x 0.454020 \
      -y -2.007622 \
      -z 2.101087 \
      -Y 1.554473
fi

echo "===== SENSOR TOPICS ====="

gz topic -l | grep -E 'team_05|signal|light' || true

echo "===== AUTONOMOUS CONTROLLER ====="
echo "WAITING FOR OFFICIAL START SIGNAL"

ros2 launch team05_race race.launch.py \
  race_start_mode:=signal \
  use_sim_time:=false \
  max_speed:=0.55
