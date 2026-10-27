# URRC Stage 0/1 results — 2026-10-27

## Repository write boundary
Allowed repository: https://github.com/nahee2345/urrc_2_f1
Read-only source 1: https://github.com/nahee2345/urrc_f1_ws at a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290.
Read-only source 2: https://github.com/nahee2345/urrc_2 at 6a18d72ef3fb2f99229f39b9ad52e7f821ad376d.
No write requests sent to either source repository.

## Stage 0

- Pinned exact upstream v10.8 commit as a Git submodule under upstream/urrc_f1_ws.
- Added .gitmodules, docs/source_lock.json, and scripts/check_upstream.py.
- Locked official six tracks, 0.55 x 0.25 x 0.18 m dimensions, five red lights, 2-lap qualification and v10.8 scripts.
- Official upstream rules and world files are unchanged. This experiment independently permits different driving sensors, a 3-lap solo test, and visual red-lights-out detection.
- Upstream clone and v10.8 runtime static checks are PENDING in the isolated runtime; GitHub read inspection and pinning were completed.

## Stage 1

- Python standard-library SDF 1.9 generator with dynamic 4-wheel car and real inertial bodies.
- Geometries: chassis and four wheels have collisions; front steering knuckles are dynamic links.
- Four physical wheel joints and two steering joints.
- Forces: two or four selectable powered wheels (FWD/RWD/AWD). Gazebo JointController uses force commands; front steering uses PID torques and joint bounds.
- Ackermann steering map honors race convention +RIGHT, and clamps the inner front wheel to 35 degrees.
- Generation settings are in config/stage1_rwd.json, with python3 scripts/build_vehicle.py.

## Generated RWD design output

| Item | Computed value |
| --- | --- |
| Conservative length | 0.460 m |
| Conservative width, including steering sweep | 0.221546 m |
| Conservative height | 0.095 m |
| Mass | 1.74 kg |
| Wheelbase | 0.340 m |
| Wheel radius | 0.040 m |
| Physical links / joints / collisions | 7 / 6 / 5 |

Note: Stage 3 sensor mounts change mass and full envelope, so sizes require reevaluation.

## Actually executed local checks

~~~text
test_invalid_commands ... ok
test_size_limit_failure ... ok
test_size_margin_and_spawn ... ok
test_source_contract_mock_fixture ... ok
test_steering_race_right_sign_and_ackermann ... ok
test_three_drives_xml_physics ... ok

Ran 6 tests in 0.009s
OK
~~~

Python compileall succeeded. Model generator executed, XML parsing succeeded (7 links, 6 joints, 5 collision geometries).

## NOT EXECUTED / NOT CLAIMED

- Actual gz sdf semantic validation, Gazebo loading, wheel-road contacts, friction-based movement, cornering, and stopping: NOT RUN.
- Source submodule checkout and direct upstream verification on Ubuntu: NOT RUN.
- ROS 2 Jazzy integration and official race submission: NOT RUN.
- AI start-camera, lap counting, collision monitor, three-lap drive: NOT IMPLEMENTED / NOT RUN.

Reason: the assistant container has no Gazebo executable or network access for cloning GitHub; repository source was inspected and modified through the authenticated GitHub connector.

**Overall:** Stage 0 infrastructure committed (upstream checkout verification pending). Stage 1 Python static checks PASS; full physical Stage 1 acceptance PENDING.
