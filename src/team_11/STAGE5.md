# TEAM 11 — Stage 5 local official-world driving test

Changes are in the team_11 package only. This test NEVER changes original official map files.
The stage5 manual driver is intentionally NOT autonomous and never starts the race orchestrator.

## A — Get code and run unit tests
```bash
cd ~/urrc_2_f1
git pull --ff-only origin main
PYTHONPATH=src/team_11 python3 -m unittest discover -s src/team_11 -p 'test_*.py' -v
source /opt/ros/jazzy/setup.bash
colcon build --packages-select team_11 --symlink-install
```

## B — Terminal 1: official Monza
```bash
cd ~/urrc_2_f1
bash src/team_11/tests/stage5_world.sh monza
```
If the official workspace differs from ~/urrc_f1_ws, set URRC_OFFICIAL_WS before launching.
Close previous Gazebo windows first. This is the official SDF loaded without modifications.

## C — Terminal 2: spawn on official first grid
```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 src/team_11/tests/stage5_spawn.py --track monza --grid 1
```
Run only once per freshly opened world.

## D — Terminal 3: ROS to Gazebo command bridge
```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge '/model/team_11/cmd_vel_11@geometry_msgs/msg/Twist]gz.msgs.Twist'
```

## E — Terminal 4: control adapter (test-only bypass)
```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run team_11 team11_stage5_adapter --ros-args -p test_bypass_gate:=true
```

## F — Terminal 5: cautious motion check
```bash
cd ~/urrc_2_f1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run team_11 team11_stage5_manual --sequence straight --seconds 1
```
Only if the track immediately ahead is clear, test `left`, `right`, `reverse` individually using the same command with `--sequence` changed.
Manual adapter stops when input stale for 0.4 seconds and clamps speed to +/-2 m/s.
Right-positive steering from URRC is converted to left-positive Gazebo yaw rate.
Never run test_bypass_gate:=true during an official competition.

## Diagnostic
```bash
gz topic -l | grep -E 'scan_11|camera_11|odometry_11|cmd_vel_11'
ros2 topic list | grep '_11'
```

## Limitations
- Manual driving and vehicle plugin behavior require user-side Gazebo validation.
- This stage does not yet implement start signal vision or autonomous navigation.
- Competition rules forbid using pre-recorded map coordinates and other vehicle/world pose for navigation. Existing strategy prototype is not competition legal.
- If official map visuals are missing, do not substitute a custom map; identify missing official assets separately.
