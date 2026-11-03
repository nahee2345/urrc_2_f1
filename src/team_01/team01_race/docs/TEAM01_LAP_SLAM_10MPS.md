# TEAM 01 autonomous racing architecture (10 m/s target)

## What changed
- Split the legacy 796-line monolith into gate, road, geometry, and controller files.
- Split the ROS node defaults and diagnostics so each new Python file is below 400 lines.
- Retain camera-controlled launch, forward LiDAR, RGB-D road segmentation, and fail-closed checks.
- Remove fixed corner OUT-IN-OUT offsets that could induce artificial S-shaped steering.
- Use rate- and acceleration-limited steering, with continued corridor validation.
- Add persistent PASS -> MERGE lateral trajectories with quintic smoothstep.
- Build a first-lap centerline from **RTAB-Map SLAM map->base_link TF**, not Gazebo world poses.
- Enter mapped operation only after a camera start-line candidate and geometric map closure.
- Second lap uses map-derived forward targets only when live sensed road boundaries agree.
- Forward visibility and braking envelope limit speed; first-lap cap 5 m/s, mapped cap 10 m/s.

## SLAM requirement
This repository consumes the TF transform `team01_map -> base_link` from an external
SLAM subsystem. Start RTAB-Map with calibrated RGB-D and/or LiDAR odometry,
correct sensor extrinsics and a loop-closure-enabled map frame. If TF is absent,
map completion and mapped-lap overtaking remain disabled by design.
The saved file `~/.local/state/urrc_team01/learning/slam_lap1_map.json`
is a *provisional* geometric loop. Check RTAB-Map loop closure graph separately.
No known map is preloaded at startup. Map samples are never treated as proof
that a corridor is free when the live camera and LiDAR disagree.

## Validation handoff
1. ROS2 Jazzy build and unit tests, then physical Gazebo launch and camera signal start.
2. 1 lap: inspect local road boundary continuity, TF age and map samples.
3. Check true lap completion/RTAB-Map loop closure independently of dead reckoning.
4. Lap 2: confirm map target within live sensed road, stable steering and obstacle tracking.
5. Verify SHIFT, PASS, MERGE against stopped and moving cars, including rear stale sensor.
6. Test 3, 5, 8, 10 m/s; 10 m/s is a *target*, not a demonstrated pass.
7. Measure steering command/actual, yaw-rate, path oscillation, clearance, stopping distance.

## Known limits
- The integrated map path is a lightly smoothed **learned centerline**, not yet
  minimum-curvature global optimization.
- The opponent detector is still short-range depth clustering; multiple cars and
  advanced occupancy prediction require subsequent integration and testing.
- Map loop confidence is currently geometric; check SLAM back-end loop-closure
  evidence before treating the map as fully certified.
- No Gazebo lap completion or 10 m/s stability claim is made by this commit.

## Included RTAB-Map ROS 2 launcher
- Install: `sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-tf2-ros`.
- `race.launch.py` automatically enables the SLAM subsystem only if
  `rtabmap_slam` is installed; override with `enable_slam:=false` for diagnostics.
- `team01_rgbd_relay` pairs registered 424x240 RGB/depth frames and provides
  camera intrinsics/extrinsics, while the LiDAR scan TF bridge declares its mount.
- ICP Odometry provides continuous TF; RTAB-Map combines RGB-D imagery and
  LiDAR for map loop closure and publishes `team01_map -> base_link` through TF.
- Missing TF, poor loop closure, or map/live-corridor disagreement blocks
  mapped 10 m/s mode. Run-time calibration and TF chain require Gazebo tests.

## 2026-11-03 developer response to independent QA run #41571452247

- P0: Fixed the test expecting `DEFAULTS` to be embedded in `node.py`; it now reads `team01_race.defaults` and verifies mapping=5 m/s, mapped=10 m/s and 20 m/s still disabled.
- P0: Clarified `defend` as a **non-steering lane-hold flag** for close rear traffic, independent of SLAM. Passing remains gated on a valid map.
- P0: Corrected adaptive trajectory checking: the candidate is validated across the full varying-curvature rollout, whereas the rate-limited first wheel command is checked only for the bounded interval until replanning. Retained 2.5 m constant-arc validation for nonadaptive plans; did NOT remove curb or collision safety checks.
- CI: Added the same RTAB-Map and tf2 ROS Jazzy dependencies to the primary Monza workflow that independent QA already installs.
- **Request to verifier:** report the `Motion.reason` for any `BRAKE`, especially `limited steering outside visible road`, distinguish 1st-lap motion from mapped-lap speed, capture `rtabmap` TF and loop closure evidence.
- No Gazebo 1-lap, SLAM loop-closure, or actual 10 m/s PASS claimed by these edits.

## Gradual straight-line capture / anti-oscillation update

- `reference_path.py`: quintic Hermite acquisition path starts at the
  vehicle's present position and heading, then smoothly enters the selected
  sensed road center/offset over approximately 3–8.5 m.
- The reference keeps the observed road curvature and never acts as a
  drivable-area authorization; the original measured bounds remain mandatory.
- `curved_path.py` accepts this reference. The controller uses one continuous
  adaptive path, removing arbitrary ±0.11 m choices that could flip each tick.
- `steering.py`: target low-pass, steering rate/acceleration limit and
  braking-rate arrival stop target-angle overshoot.
- In verified unobstructed straight sections, wheel command is limited to
  ±6 degrees during capture; curve/obstacle/edge cases are exempt.
- `node.py` logs `join_length` and `join_error`.
- Test `tests/test_v15_gradual_capture.py` defines gradual-join invariants.
- **Verification handoff:** Record commanded steering vs. yaw, path switching,
  peak lateral error, sign reversals per 10 m, `Motion.reason` for BRAKE,
  and actual distance/lap count in Gazebo. This change is not an actual
  10 m/s or full-lap PASS.
