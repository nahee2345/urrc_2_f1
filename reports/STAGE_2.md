# Stage 2 status — 2026-10-27

## Source and write protection
Writable: nahee2345/urrc_2_f1 only.
Immutable source module: urrc_f1_ws@a23c7eec41b98b67ec3a3a2a46f9d7ed957f8290.
Other upstream urrc_2 unchanged.

## Stage 1 prerequisite
User-supplied actual Ubuntu result: PASS_FLAT_PHYSICS, 12 / 12 unit tests.
User measurement: forward +X 1.439996m; 3s post-command brake drift 0.237083m; left turn +Y 0.245785m; stable height approximately 0.04m.

## Implementation
- Stage2 SDF generation from original dynamic stage1 4-wheel model, no world replacements.
- 10 AI configuration profiles with distinct allowed driving sensor combinations.
- Common dedicated start camera, forbidden in `driving` profile list.
- Cameras: real SDF `camera` and `depth_camera`. Stereo: two independent cameras.
- GPU LiDAR: real SDF `gpu_lidar`, both one-vertical-beam 2D and 16-beam 3D.
- Per-sensor collision and visual housings, composite chassis COM and full inertia tensor.
- Motor/steering model and SDF physics from stage1 retained.
- Explicit model namespace topic isolation and optional ROS 2 GZ_TO_ROS bridge configuration.

## Tests and evidence
- Structural tests: Python unittest suite, 10 model manifests, SDF XML parse.
- Gazebo SDF parser: each generated AI model checked in GitHub Actions when completed.
- Rendering tests: real Gazebo RGB and 3D LiDAR stream payloads through Ogre2, in separate non-Monza world, if supported by runner graphics.
- Static CI URL: https://github.com/nahee2345/urrc_2_f1/actions/runs/40448610922
- Exact runtime verdict to be updated from GitHub Actions logs. A workflow with failures is not PASS.

## Known remaining tasks
- Physical 3D lidar PointCloudPacked -> ROS PointCloud2 bridge requires independent validation.
- Start camera light-out detection/classification and race-start authorization not implemented.
- Monza original track, obstacles and wall contact not tested here.
- No learning/controller implementation or 3 consecutive race laps.


## Verified runtime evidence — Stage 2 actual sensors

**GitHub Actions 5-configuration run:** https://github.com/nahee2345/urrc_2_f1/actions/runs/40449012977

**Result:** PASS_SENSOR_STREAMS for AI-03, AI-04, AI-05, AI-08, and AI-10. Every enabled sensor in those five models published an actual nonempty message during the Gazebo Harmonic Ogre2 headless trial.

| AI | What was measured | Actual result |
| --- | --- | --- |
| AI-03 | Start camera; both stereo camera image streams | PASS |
| AI-04 | Start camera; depth image R_FLOAT32 | PASS |
| AI-05 | Start camera; RGB image; 3D LaserScan; 3D PointCloudPacked | PASS |
| AI-08 | Start camera; RGB image; 2D LaserScan; 3D LaserScan; 3D PointCloudPacked | PASS |
| AI-10 | Start camera; RGB image; depth image; 3D LaserScan; 3D PointCloudPacked | PASS |

Machine-readable full raw results (including topic name, advertised/received flags, and message byte counts) are archived in reports/stage2_ci_5cases_20261027.json.

**Full 10-AI CI run:** https://github.com/nahee2345/urrc_2_f1/actions/runs/40449401194. Its 25 Python unit tests, all 10 sensor vehicle models, and SDF static checks passed. Await authoritative *completed* Gazebo results for per-AI runtime classification; do not invent PASS for a running job.

**Stage 1 regression:** https://github.com/nahee2345/urrc_2_f1/actions/runs/40449067712 (both static and physical test PASS).

## Actual sensor specifications and constraints
- Every model carries the start-only camera; its video data are actual, but it does not yet classify five red lights.
- Stereo is two separate physical camera sensor poses with distinct image topics, not an assumed monocular depth model.
- Sensor housings have visual and collision geometries; mass, full inertia tensor, and sensor center of mass are integrated with the dynamic chassis.
- 10 generated vehicles: overall envelope approximately 0.477m long × 0.22155m wide × 0.115–0.138m high, masses 1.792–1.927kg.
- FWD/RWD/AWD drive variants remain supported.
- Gazebo PointCloudPacked stream has been confirmed independently of LaserScan (AI-05, AI-08, AI-10).
- Pure Python sensor and bridge mapping tests are PASS; actual ROS 2 Jazzy bridge end-to-end test, AI perception, original Monza race, wall collision and three-lap clean run remain pending.
