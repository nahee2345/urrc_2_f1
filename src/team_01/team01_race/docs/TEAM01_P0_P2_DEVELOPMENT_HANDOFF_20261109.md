# TEAM01 P0-P2 integrated development handoff — 2026-11-09

STATUS: Code changes committed to a development branch. NO UNIT TESTS, NO GAZEBO, NO CI and NO LAP VALIDATION were executed at user request. Original Monza and QA scripts unchanged. Official Monza one-lap PASS remains NOT ACHIEVED.

| Priority | Development changes | Deferred independent QA |
|---|---|---|
| P0-1 Startup | camera frame states NO_FRAMES / STALE / BASELINE_PENDING / READY; preserve fail-closed camera gate | ROS 2 bridge, RGB frames, RED 4→0, permission and start |
| P0-2 Drivetrain | pair adapter speed and steering commands within 120ms; new passive drive_chain diagnostics | actual wheel movement and continuous 5m |
| P0-3 Steering | reject NaN/Inf steering intent; keep same-side gradual steering and 3-frame opposite-turn confirmation | S-oscillation and actual IMU / steering delay |
| P0-4 Corner | remove bird-eye grid; use measured ego-frame RGB-D/LiDAR corridor | first corner, curb clearance and offtrack |
| P0-5 Speed | cap overtaking HOLD state; keep TTC, stopping envelope and basic stage cap | measured brake distance and Monza lap |
| P1-1 Sensor registration | 50ms road RGB/Depth ROS timestamp guard; existing 25ms SLAM synchronization | actual sensor stamps, QoS and TF |
| P1-2 SLAM | revoke loop closure permission after implausible odometry position jump | RTAB-Map loop closure and covariance |
| P1-3 2nd lap | reject reverse learned-map tangent and unsafe lookahead; live road bounds still override map | verified mapped lap |
| P1-4 Speed stages | preserve QA approval gates for 5→8→10m/s and disabled 20m/s | approval for each higher stage |
| P2-1 Overtake/rejoin | re-evaluate side corridor during SHIFT, PASS and MERGE, hold on obstruction | dynamic traffic, object tracking and safe rejoin |
| P2-2 Other tracks | retain original 6-track runner and track-independent measured road corridor | independently test Spa, Monaco, COTA, Hungaroring, Baku |
| P2-3 Concurrent race | preserve vehicle-local safety, rear gap and per-agent command topics | 10 vehicles, mutual traffic interactions, collisions |

BEV replacement: eliminate local BEV OccupancyGrid generator; publish /team01/debug/corridor with base_link measured right/left edges, usable width, nearest detected in-road obstacle and near/middle/far observations. Unknown or stale corridors remain invalid. This is diagnostic; actual controller uses measured RGB-D and LiDAR RoadCorridor rather than the deleted BEV grid.

Boundary conditions: no world/map/SDF changes, no 7s fallback in normal camera race mode, no enabling of unapproved high speeds, no modifications to independent QA workflow, no preloaded track racing line.

Important: These patches are unverified development changes, not proof of solving START_CALIBRATION_FAILED or 0m motion. True multi-car prediction (DBSCAN/IMM/CTRV), minimum-time racing lines and MPCC/MPPI remain incomplete or not deployed. Do not claim 10-car safe operation or generalized performance.

When the user explicitly orders testing: fixed-SHA offline tests and size gate; ROS2 topics and launch; camera bridge and readiness; adapter/drive_chain, 5m straight; actual first corner and official Monza lap; RGB-D+LiDAR+IMU timing and RTAB-Map closure; 2nd lap; 8/10m/s stages; dynamic passing; five other tracks; ten concurrent vehicles.
