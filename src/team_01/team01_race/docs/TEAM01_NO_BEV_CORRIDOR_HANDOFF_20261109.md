# TEAM01 2026-11-09 — BEV replacement with measured ego-frame corridor

STATUS: DEVELOPMENT ONLY. User requested NO TESTS or QA until separately ordered.

- Remove OccupancyGrid bird's-eye raster publisher and BEV helper modules.
- Publish /team01/debug/corridor as JSON String containing only currently measured
  base_link road boundaries, observed forward stations, usable lane width,
  nearest observed on-road obstacle, and near/middle/far road regions.
- Missing/stale sensor input or missing measured bounds gives valid=false;
  unobserved space is never labelled free.
- Existing RGB-D/LiDAR RoadCorridor and local path/collision planner remain
  the driving input; BEV was diagnostic, not a motion authorization source.
- Add camera_frame_state (NO_FRAMES, STALE, BASELINE_PENDING, READY)
  to existing startup diagnostics without bypassing camera startup safety.
- Require measured boundaries in ROI diagnostics rather than guessed width.
- Keep shadow MPC diagnostic only. Never enable 8/10/20 m/s by this patch.
- Preserve original Monza/world/submodule, 0.3m race boundary and QA scripts.

QA TO RUN ONLY AFTER USER COMMAND:
1. Offline Python regression, import, source size <=400 lines.
2. ROS2 /team01/debug/corridor topic and real sensor frame freshness.
3. Camera readiness, RED 4→0, start gate, Gazebo wheel and odometry.
4. Five-metre straight, first corner, official Monza lap.
5. RGB-D/scan time alignment, SLAM TF/loop closure, IMU/joint sign latency.

No actual fixes to 0m START_CALIBRATION_FAILED or complete lap are claimed.
