# Stage5: LiDAR traffic perception and bounded local passing

This extension retains the stage4 centerline/racing planners and the vehicle model. The frozen stage4 submission is `artifacts/stage5/team04_stage4_frozen.zip`; the original stage4 archive and all earlier raw results remain untouched. New runs, builds and reports belong to `artifacts/stage5`.

## Modes and inputs

`autonomy_traffic_mode:=solo_racing` keeps the original racing decision when no interior object is present. Detection, tracking, prediction and following/stop protection remain active. `traffic_racing` additionally considers passing and is capped at 1.0 m/s by default (`traffic_max_speed`, validated bounds enforced by the node). The solo ceiling remains 4.5 m/s. Do not interpret this as validation of high-speed traffic racing.

```bash
ros2 launch team04_race race.launch.py driving_mode:=autonomous \
  autonomy_traffic_mode:=solo_racing
ros2 launch team04_race race.launch.py driving_mode:=autonomous \
  autonomy_traffic_mode:=traffic_racing autonomy_max_speed:=1.0
```

The direct launch still defaults to manual. Autonomy consumes only its own PointCloud2, start Bool and clock. Prior own speed/steering commands are internal motion predictors. Actor poses, encoder, IMU, map and track names are never control inputs.

## Perception and tracking

The existing measured ground plane and raw obstacle points are reused. A 12 cm spatial grid clusters neighboring returns within 17 cm. Components need at least three returns and 2 cm XY extent; a single point is never a confirmed vehicle. Measured wall contacts, long facades and objects outside the polygon of consecutive actual paired cross-sections (with current straight-wall support for the +/-0.8m side-sector gap only) are rejected as vehicle candidates, while raw static collision protection remains active. Sparse compact returns remain unknown occupied regions, with a larger hidden-size envelope. Longitudinal fragments with overlapping lateral support are coalesced only when their full union fits a0.60x0.30m forward body; the complete union remains occupied. Side-by-side envelopes remain separate.

A one-to-one gated nearest-neighbor association and alpha-beta filter retain up to16 object IDs. Timestamped range rate estimates closing speed. Command-based SE(2) motion is supplemented at a0.1 feature weight by current straight wall features only when both walls have sufficient extent, low fitting residual and consistent tangents. Forward displacement along parallel walls is unobservable and still uses the command model. Curved/insufficient walls use the conservative command fallback. This is not GPS/encoder/IMU odometry.

The tracked longitudinal anchor is the nearest observed X face, rather than a bbox center that moves when another body panel becomes visible. A measured planar face can support a bounded heading hypothesis. Every visible anchor receives an isotropic0.64m hidden-body pad (larger than the maximum permitted body diagonal), plus the observed extents, fitting/range uncertainty and future growth; it is not the small visible point cluster. Old IDs expire after one second. Unresolved retired occupancy is held separately until actual rays through that region show clearance. Missing detections do not mean free space.

## Planning, state and protection

All ordinary stage4 safety gates remain. Object returns are separated only for current boundary extraction and then restored to the raw full-body collision certificate. Neither a wall nor a path is extrapolated through unseen space.

Passing extends the existing candidate sampler with3–5m smooth lateral transitions and a held side. The remaining transition contracts with accumulated own commanded travel, so replanning does not repeatedly restart the lane change. A1.5m minimum transition and the existing curvature/steering/stopping gates still apply. Traffic manoeuvres certify a measured local interval of at most4m and replan at each scan; the unchanged stopping and lookahead budgets must fit inside this interval. Solo racing retains its10m interval. Candidates must pass measured wall checks, every raw obstacle point and future inflated object-occupancy SAT tests. The original stopping budget, reaction reserve, sensor timeout, steering sign/rate and mechanical limits remain.

States are RACING, TARGET_DETECTED, FOLLOWING, OVERTAKE_PLANNING, OVERTAKE_LEFT/RIGHT, SIDE_BY_SIDE, RETURN_TO_RACING_LINE, ABORT_OVERTAKE and EMERGENCY_BRAKING. A confirmed association, enough returns, low measured curvature and an approach interval precede a pass. The chosen side is latched. Brief occlusion permits prediction without acceleration; longer loss aborts. The return gate requires an observed complete opponent envelope behind the ego rear, an observed or identity-confirmed crossing into the rear sector, commanded progress and a collision-certified return path. Completion and post-return normal driving are checked independently by the test observer.

TTC uses an inflated longitudinal gap and a conservative closing estimate. It is a constant-velocity reference; lateral footprint prediction and physical stopping distance are additional constraints. It is not an oracle of future opponent maneuvers.

## Physical sensing limitation observed in simulation

The frozen front camera masks shallow forward downward LiDAR rays. In the actual stopped-car run, the -1degree rays within +/-4degrees returned at about0.186m on own geometry. A centered low opponent can therefore be invisible until close range; side-visible opponents have different visibility. This cannot be solved by claiming that missing points are free. Physical geometry and the16x720/20Hz sensor are unchanged. Current traffic results must be read with this limitation; no blanket twelve-car collision-free guarantee is made.

## Validation and output

Use explicit `--base-paths src/team_04/team04_race` with stage5 build/install/log directories. Earlier extracted packages are immutable and would otherwise be discovered as duplicates.

`tests/traffic_trial.py` runs an isolated GZ partition and ROS domain96. It creates only team-local test copies of physical actors, with original masses, collisions, joints, force limits and watchdog. Actor sensor publishers are removed from test copies to avoid ego-topic contamination. Test actor commands use dedicated `/team04/test/...` topics. The original world, official spawner source, lights and timing code are unchanged. Ground-truth actor controllers and evaluators run only in the test harness.

`tests/analyze_stage5.py` reports actor detection/association, nearest-footprint range error, closing-speed error, constant-velocity TTC bias, full-footprint gap, contacts, offroad estimates and strict completed-pass criteria. Range-qualified recall includes occlusion and is not an unconditional visibility oracle. Contact observations are accumulated every physics step. Sim and wall times are separate. The final `REPORT_STAGE5.md` and acceptance files distinguish actual success, safe stop/abort, failure and unexecuted cases.

Debug: `/team04/debug/traffic_state` JSON, `/team04/debug/tracked_vehicles` boxes/IDs, conservative predicted occupancy at0/1/2s and state/risk/TTC text, plus existing boundaries, racing candidates, selected path, target and speed profile. No truth topic is fed back into these control modules. GUI validation is separate from publishing these data.

## Rollback and release scope

`config/autonomy_stage4_rollback.yaml` preserves the prior solo numerical profile. It does not remove object safety from stage5. For the exact prior code, extract the untouched frozen stage4 ZIP into a new team-local workspace and build there; never overlay it on this source tree. The stage5 ZIP is a source submission, not a certification that every traffic acceptance criterion passed. Consult REPORT_STAGE5.md and artifacts/stage5/acceptance.json for actual results.

The first final candidate exposed a solo false positive at the first Monza turn: nearest-section normal projection included a curved-wall endpoint outside the measured corridor. Revision2 requires actual polygon containment, keeps raw obstacle checks, and revalidates the failed raw cloud. The original candidate ZIP/results remain in release_v1_preserved and relocation_ws.

Revision3 preserves side-sector detections across the small origin gap between front and reversed rear paired traces. Only currently fitted straight walls may classify this near-origin gap; this does not synthesize boundary points or extend any path. Revision2 Q02 was intentionally interrupted after this traffic regression was found, rather than spending a full long solo trial on the superseded candidate.
