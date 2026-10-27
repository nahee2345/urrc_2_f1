# URRC AI individual experiments — Stage 0 and Stage 1

**Only this repository may be modified.** Upstream `urrc_f1_ws` and `urrc_2` are read-only sources.

- **Stage 0:** pin and verify the `urrc_f1_ws` v10.8 source using an immutable Git submodule.
- **Stage 1:** create an SDF 1.9 dynamic four-wheel vehicle with FWD/RWD/AWD configurations, individual front steering joints, force-limited motors, collision and inertial parameters.
- **Status:** Stage 1 has 12 passing Python tests. GitHub Actions (Ubuntu 24.04 + Gazebo Harmonic) has **PASS_FLAT_PHYSICS** for forward motion, stopping response and left steering. The measured CI result is in `reports/stage1_flat_physics_ci_20261027.json`. Official Monza racing and three-lap completion are **not yet verified**. See `reports/STAGE_1_HARDENING.md` and GitHub Actions.

## Download

```bash
git clone --recurse-submodules https://github.com/nahee2345/urrc_2_f1.git
cd urrc_2_f1
git submodule update --init --recursive
```

## Stage 0: read-only upstream verification

```bash
python3 scripts/check_upstream.py
```

## Stage 1: model generation and unit tests

```bash
python3 -m unittest discover -s tests -v
python3 scripts/build_vehicle.py
python3 scripts/stage1_smoke.py --prepare
# On Ubuntu with Gazebo Harmonic:
python3 scripts/stage1_smoke.py --run
```

Generated SDF: `generated/urrc_stage1_rwd/model.sdf`.

`urrc_f1_ws` officially permits LiDAR for driving and camera for lights only. This project's experiment-specific changes (camera/depth for driving and solo 3-lap PASS) are **overlays**, not changes to official upstream rules. Stage 1 includes no sensors or autonomous driving; these are later stages.

Authoritative upstream lock: `docs/source_lock.json`.

## Stage 2 — multi-sensor generation (10 vehicles)

All 10 experimental vehicle designs are generated from the SAME validated physical 4-wheel chassis, each with a dedicated start-light camera and an independent selection of RGB, depth, stereo, 2D LiDAR or 3D LiDAR driving sensors.

~~~bash
cd ~/urrc_2_f1
git pull --ff-only
git submodule update --init --recursive
python3 -m unittest discover -s tests -v
python3 scripts/build_sensors.py --ai all
python3 scripts/stage2_sensor_smoke.py --run --ai AI-05
cat generated/stage2_sensor_result.json
~~~

Each vehicle SDF and its exact sensor topics / overall dimensions are saved under `generated/stage2/AI-XX/`. ROS 2 Jazzy bridge YAML can be generated from a vehicle manifest:

~~~bash
python3 scripts/bridge_sensors.py --manifest generated/stage2/AI-05/manifest.json \
  --output generated/stage2/AI-05/ros_bridge.yaml
~~~

Documentation: [Stage 2 Korean guide](docs/STAGE2_GUIDE.md), [Stage 2 test report](reports/STAGE_2.md). GitHub Actions builds/validates all ten SDF files and runs actual Gazebo rendering sensors. A successful sensor stream does **not** mean ROS bridging, actual camera traffic-light recognition, autonomous racing, or three clean laps have passed.
