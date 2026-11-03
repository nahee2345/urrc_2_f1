# TEAM 01 v11 — Actual Gazebo failure diagnosis and corrections

Based on GitHub Actions run 41557440018 and artifact `lap.json`.

- Previous run: `OFF_TRACK`; **1.111 m actual displacement**, **0.177 m track advance**, 1/120 track sectors; **most odometry velocities were approximately 0 m/s** despite commands at 2.1 m/s. This is a starting-grid / actuator / physics problem; it did not reach a Monza corner.
- Original start `x=0.378672, y=-2.056035, yaw=1.457384` was close to the outer road boundary according to the independent verifier; max inferred margin breach 0.041 m. Official grid geometry, vehicle collision, and verifier margin need reconciliation. **The CI harness adjusts the starting lateral pose only if required for safe clearance and records the correction in `ci_artifacts/spawn_clearance.json`. Thus a PASS would certify the adjusted CI start, not strict race-grid placement.**
- The Gazebo Sim 8 AckermannSteering plugin was configured with `<min_velocity>0</min_velocity>`. The same limit may clamp negative angular velocity (known Gazebo issue gazebosim/gz-sim#2734), making a turning direction ineffective. New physics plugin sets minimum -12 m/s; the ROS adapter continues to prevent reversing.
- Original road camera horizon ~2.25 m capped straight speeds to ~2 m/s. New corridor fusion extends the horizon *only when* independent LiDAR guardrail geometry agrees with aligned RGB/Depth asphalt. Mismatch fails closed.
- Adaptive-lookahead pure pursuit based on F1TENTH references `f1tenth-dev/pure_pursuit` and `dylanjian0/f1tenth-racing-stack`, uses locally sensed edges, never preloaded reference paths.
- First lap straight target 8 m/s; subsequent lap target 10 m/s, corner speed capped at 30% of its current straight target and at safe lateral acceleration / stopping distance. Actual speed may be lower.
- Added independent watchdog `DRIVE_STALL`: if vehicle travels <2m within 80 wall seconds after verified lights out, CI fails instead of misreporting lap or waiting indefinitely.
- Live controller does not access original track centerline; only CI test rig uses it to set a physically-clear start point and independently measure lap progress.
- OFF_TRACK, DRIVE_STALL or TIMEOUT are FAIL. Only `GEOMETRIC_LAP_PASS` and workflow success prove one geometrically valid lap; contact-free certification would additionally require collision contact telemetry.
