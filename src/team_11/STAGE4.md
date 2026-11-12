# Stage 4 — yellow TEAM 11 vehicle

## Specs and constraints
- Gazebo Harmonic model: `models/team11_yellow_car/model.sdf`.
- Canonical link: `base_link`, located at the geometric mid-point of the model's *designed outer envelope*.
- Envelope: length <=0.55 m, width <=0.25 m, height <=0.18 m.
- Body yellow, four wheels, front steering (±30 degrees), rear drive.
- Center of designed wheel envelope at local Z=0, lowest point at Z=-0.09 m.
- **Official spawn_z_offset_m must be 0.09**, not 0.04, unless actual Gazebo contact verification dictates a further correction.
- Camera is for start recognition only, LiDAR is the only permitted driving sensor under the official race specification.
- Original maps, official signal control and world assets are never altered.

## Verify static model

```bash
cd ~/urrc_2_f1
python3 -m unittest src/team_11/test_model.py -v
gz sdf -p src/team_11/models/team11_yellow_car/model.sdf >/tmp/team11_expanded.sdf
```

## Gazebo model-only trial
On a machine with Gazebo Harmonic:

```bash
cd ~/urrc_2_f1
gz sim -r -v 3 src/team_11/tests/model_test_world.sdf
```

Look for yellow chassis, four wheels with ground contact, no explosion/collision drift, and check topics:

```bash
gz topic -l | grep -E 'scan_11|camera_11|odometry_11|cmd_vel_11'
```

The model-only world is a temporary flat test map, not the official race circuit. **Do not launch it at the same time as an official race world**.

## Important race rules found in official urrc_f1_ws
Official `race/README_KO.md` forbids stored track coordinate use for driving decisions and forbids GPS, IMU, world pose, and other teams' information. It allows only onboard LiDAR for driving decisions and camera for start signal recognition. The earlier `routes.py` and `strategy.py` modules are research prototypes and **not suitable for formal competition** without changing the approach to LiDAR-only lane inference and opponent tracking. This limitation must be resolved before stage 13.
