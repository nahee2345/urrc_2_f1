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
