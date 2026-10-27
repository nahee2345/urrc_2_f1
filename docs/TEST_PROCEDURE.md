# Stage 0/1 test procedure

## Scope
Only nahee2345/urrc_2_f1 is modified. urrc_f1_ws and urrc_2 are read-only. The official upstream commit is a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290 and is pinned as a submodule. Its official race policies are not modified.

## Stage 0: checkout and source integrity
Run on Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic:

~~~bash
git clone --recurse-submodules https://github.com/nahee2345/urrc_2_f1.git
cd urrc_2_f1
git submodule update --init --recursive
python3 scripts/check_upstream.py
bash upstream/urrc_f1_ws/test_v10.sh
~~~

The last command is an upstream static v10.8 check, not a physics run.

## Stage 1: local tests and model generation
~~~bash
cd urrc_2_f1
python3 -m unittest discover -s tests -v
python3 scripts/build_vehicle.py --config config/stage1_rwd.json --output generated/urrc_stage1_rwd/model.sdf
gz sdf -p generated/urrc_stage1_rwd/model.sdf > /tmp/urrc_stage1_expanded.sdf
~~~

The generated model has 7 links, 6 revolute joints and 5 collision shapes. Two wheel controllers are driven by capped forces; two position PID controllers steer front wheels. No sensor is mounted in Stage 1. FWD, RWD, and AWD model generation are tested by Python tests.

## Stage 1: original Monza manual physics inspection (not yet executed)

Terminal A:

~~~bash
cd urrc_2_f1
source /opt/ros/jazzy/setup.bash
bash upstream/urrc_f1_ws/run_v10.sh monza
~~~

Terminal B, after Gazebo loads:

~~~bash
cd urrc_2_f1
source /opt/ros/jazzy/setup.bash
python3 scripts/build_vehicle.py
ros2 run ros_gz_sim create --world monza --file generated/urrc_stage1_rwd/model.sdf --name urrc_stage1_rwd --x 0.3786723464 --y -2.0560353977 --z 1.2435265615 -Y 1.4573844161
~~~

After the wheels settle onto the original road, send a low-speed target to each RWD wheel. Joint +Y spin must be **positive** for vehicle +X forward movement (verified by headless Gazebo).

~~~bash
gz topic -t /model/urrc_stage1_rwd/drive/rear_left -m gz.msgs.Double -p 'data: 3.0'
gz topic -t /model/urrc_stage1_rwd/drive/rear_right -m gz.msgs.Double -p 'data: 3.0'
~~~

Stopping commands, sent to BOTH motors:

~~~bash
gz topic -t /model/urrc_stage1_rwd/drive/rear_left -m gz.msgs.Double -p 'data: 0.0'
gz topic -t /model/urrc_stage1_rwd/drive/rear_right -m gz.msgs.Double -p 'data: 0.0'
~~~

To inspect the front-steer PID behavior (do not attempt while speeding):

~~~bash
gz topic -t /model/urrc_stage1_rwd/steer/left -m gz.msgs.Double -p 'data: 0.1'
gz topic -t /model/urrc_stage1_rwd/steer/right -m gz.msgs.Double -p 'data: 0.1'
~~~

Check four wheel-ground contacts, body stability, actual acceleration via torque and friction, steering response and braking. Zero motor speed targets do not guarantee instantaneous braking; allow the physics to settle. Save error logs, simulator video, and joint states. Mark a physics PASS only after the actual simulated car demonstrably moves and turns without penetration or exploding dynamics.

The model is not an official race package. Do not use it to claim a three-lap AI PASS.

## Acceptance states
- Stage 0 source SHA and official contract: upstream lock COMMITTED, direct checkout verification PENDING.
- Stage 1 Python unit tests: PASS (12/12 locally and on GitHub Actions).
- Stage 1 Gazebo SDF and straight-line physics: SUCCESS via Ubuntu 24.04 GitHub Actions. The local assistant container lacks Gazebo. Latest braking and steering automated tests are also in GitHub Actions; check the newest completed run and the STAGE_1_HARDENING report.
- Stage 2 and later: NOT STARTED.
