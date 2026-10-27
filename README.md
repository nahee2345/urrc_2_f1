# URRC AI individual experiments — Stage 0 and Stage 1

**Only this repository may be modified.** Upstream `urrc_f1_ws` and `urrc_2` are read-only sources.

- **Stage 0:** pin and verify the `urrc_f1_ws` v10.8 source using an immutable Git submodule.
- **Stage 1:** create an SDF 1.9 dynamic four-wheel vehicle with FWD/RWD/AWD configurations, individual front steering joints, force-limited motors, collision and inertial parameters.
- **Status:** Python static/unit tests passed locally; Gazebo runtime physics and three-lap race are **NOT YET VERIFIED**. See `reports/STAGE_0_1.md`.

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
```

Generated SDF: `generated/urrc_stage1_rwd/model.sdf`.

`urrc_f1_ws` officially permits LiDAR for driving and camera for lights only. This project's experiment-specific changes (camera/depth for driving and solo 3-lap PASS) are **overlays**, not changes to official upstream rules. Stage 1 includes no sensors or autonomous driving; these are later stages.

Authoritative upstream lock: `docs/source_lock.json`.
