# TEAM 11 — race driving prototype (stage 2)

Official map source: [urrc_f1_ws](https://github.com/nahee2345/urrc_f1_ws). NEVER modify official track worlds, meshes, grids, signals or other teams' packages.

## Implemented
- routes.py: 3 lanes (1 right / 2 center / 3 left) for each of six official centerline CSVs; <= boundary width estimates and clearance guard.
- strategy.py: map-indexing; pure pursuit steering limited to +/-30 degrees and steering rate <=90 deg/s; curvature-dependent speed cap and ~3s lookahead; front vehicle TTC <=5s passing trigger; nearby-free-lane checking; 1->3 via lane 2 with minimum 2 seconds; rear threat <=3s defensive positioning; no lane changes in sharp corners; speed/steering slew and emergency map-loss stop. Opponent timestamps, lane and longitudinal gap evaluated in a common map frame.
- tests: synthetic geometry and tactical scenarios.
- sync_team11.sh: clone or fast-forward update the local working copy, stopping on local changes.

## NOT YET COMPLETE / unsafe to race
- ROS2 subscriptions, odometry, camera detection, traffic lights, Gazebo vehicle model/spawn and command bridges are still missing. **This is not a driveable ROS2/Gazebo package yet.**
- Generated routes are not committed; generate using the installed official source data.
- Tactical path arbitration remains a prototype. Do NOT interpret these tests as 6-map successful driving. Grid-start autodetection, true collision prediction, jerk-limited 5th-degree lane changes, passing clearance proof, physical lateral friction, real rear-vehicle motion tracking, and full-speed validation still need implementation.
- Centerline road widths are estimated, NOT surveyed wall boundaries. A >=0.5 m gap to actual collision meshes must be separately verified.
- Defence must not cause deliberate contact or violate race rules. Safety overrides competitive position.

## Local checkout or update
From another directory, first time:
```bash
git clone https://github.com/nahee2345/urrc_2_f1.git ~/urrc_2_f1
```
Every later update:
```bash
cd ~/urrc_2_f1 && bash src/team_11/sync_team11.sh
```
This script only updates this Git checkout and does not delete unsaved work. You must run it after changes are pushed; there is no unattended automatic update.

## Generate 18 paths locally
```bash
cd ~/urrc_2_f1
python3 src/team_11/routes.py --track-package ~/urrc_f1_ws/src/urrc_track_gazebo --output src/team_11/generated_routes
```
Run portable stdlib tests:
```bash
PYTHONPATH=src/team_11 python3 -m unittest discover -s src/team_11 -p 'test_*.py'
```


## ROS 2 controller prototype (requires external vehicle interface)
```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
colcon build --packages-select team_11
source install/setup.bash
ros2 run team_11 team11_controller --ros-args -p use_sim_time:=true -p track:=monza -p spawn_grid:=1 -p routes_root:="$HOME/urrc_2_f1/src/team_11/generated_routes"
```
Requires fresh odometry `/odom_11`, `/start_allowed_11` from a camera-signal detector, and optionally opponent odometry topics explicitly named through `opponent_topics`. Publishes `/cmd_vel_11` Twist with `linear.x` = speed and **`angular.z` = steering angle radians (NOT angular velocity)**. This command needs a verified actuator adapter for the actual vehicle. The simulator and camera-start detector are not yet provided. `spawn_grid` is manually configured here; automatic slot inference still remains.
