# TEAM01 integrated hardening state / test hold — 2026-11-09

Development branch: dev/ego-corridor-no-bev-20261109.
Do not merge to main or trigger tests until user explicitly requests verification.
No ROS2/Gazebo run, independent QA, real 5m drive, corner or lap PASS is claimed.

## P0 implemented changes
- Camera state diagnostic, readiness status, no forced launch on regular camera mode.
- Controller-originated start authorization heartbeat added. Adapter requires that
  authorization to be fresh, alongside operator/no-signal gate, both speed and
  steering commands fresh and paired within 120ms.
- Passive drive_chain now distinguishes missing/stale controller command,
  adapter diagnostics, odometry, and a command without motion.
- Steering intent rejects nonfinite target, gradual same-side turns preserved.
- Previous curb and collision safety, braking envelope and ego-frame measured
  corridor remain. Passing HOLD now brakes to zero, pending QA.
- No automatic speed-stage escalation; unapproved 8/10/20m/s remain locked.

## P1 implemented changes
- RGB/Depth road frames must be within configured 50ms sensor stamp tolerance,
  in addition to previous delivery-time sanity checks.
- SLAM map use requires prior loop closure and current quality; VO
  discontinuity or uncertain observation revokes previous closure evidence.
- Learned map target must agree with forward direction and live road bounds.
- Original multi-rate reference path and physical IMU/joint correction approval
  gates preserved. New feedback calibration remains disabled by default.

## P2 implemented changes and limitations
- Fifth-order shift samples collision-checked against measured road envelope;
  side, forward lead and merge lanes are rechecked on each control update.
- Closing lead gaps can force HOLD; HOLD commands zero instead of carrying an
  unverified trajectory. This is NOT full multi-agent predicted collision
  avoidance, ID-based opponent tracking or safe racing certification.
- Existing run_v10.sh still offers Monza/Spa/Monaco/COTA/Hungaroring/Baku.
  No race track assets, world files, friction values or official 0.3m rule changed.
- Ten-vehicle independent namespacing/spawning/coordination and real-world
  multi-car performance are NOT established. These require engineering work
  beyond local control logic and an authorized independent QA.

## Deferred verification requirements
1. Freeze exact code SHA; offline regression, >= legacy test count and 400 lines.
2. ROS2/Gazebo start pipeline: RGB camera ready, RED→OFF, gate, cmd watchdog.
3. Controller → adapter → Gazebo wheel/steering joints → actual odometry;
   5m uninterrupted movement and sign/latency evidence.
4. Real first corner, curvature hysteresis, curb margin, collision and brake.
5. Official unchanged Monza 1 lap; map TF, RTAB closure and actual second lap.
6. QA-approved 8m/s and 10m/s in separate runs; no unsafe 20m/s activation.
7. Moving opponents, passing and merging; five other tracks individually.
8. Ten vehicles concurrently with independent agent IDs/ROS topics,
   measured separation, official disqualifications and reproducible bags.

No verification has been run by the development assistant.
