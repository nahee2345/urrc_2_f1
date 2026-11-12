# TEAM 11 — URRC F1 autonomous race car

## Source of truth
- Official map source: `nahee2345/urrc_f1_ws/src/urrc_track_gazebo`
- **Never edit official track worlds, meshes or signal assets.**
- Six tracks: monza, spa, monaco, cota, hungaroring, baku.
- ROS 2 Jazzy / Gazebo Harmonic target; each TEAM-owned ROS topic ends in `_11`.
- The standalone participant package will be `src/team_11`; the official track package is an external runtime dependency.
- This revision is **stage 1: route generation + contract**. It is NOT a running autonomous vehicle yet.

## Lane geometry
- Three lanes from the **driver's right to left**: 1 (right), 2 (middle), 3 (left).
- Car dimensions: desired 0.55 x 0.25 x 0.18 m, subject to official event maximum.
- Clearance measured from car's outer side to the road boundary/wall: >=0.50 m.
- Given width W and car width B: maximum center offset = W/2 - B/2 - 0.50.
- Three lane centers = [-offset, 0, +offset] in left-positive coordinates, with offset not exceeding the geometric maximum.
- If the calculated offset is insufficient for three **distinct, safely usable** lanes, mark the segment as narrow: no passing/defensive lateral move there. Do not silently violate wall clearance.
- Road width is a **provided estimate**, not surveyed lane/wall geometry. Confirm physical collision clearance in Gazebo.
- Source CSV has `x_m,y_m,z_m,yaw_rad,road_width_m`. Files close by connecting last row back to first.
- One set of three lane path files per track; do not render path visuals into the official world.

## Driving state machine
1. WAIT_SIGNAL -> JOIN -> CRUISE -> PASS / DEFEND -> CRUISE. CORNER constraints override PASS/DEFEND.
2. Spawn grid automatically inferred from nearest official grid pose (only if distance and orientation are plausible). Odd -> lane 1; even -> lane 3. Use a smooth, curvature-bounded merge.
3. PASS: only when a forward car exists, time-to-contact <=5 s, available next lane is free and straight/mild bend. From lane 1 to 3 (or inverse) occupy lane 2 for ~2 s **provided it remains safe**. Return/complete once other car is behind with longitudinal gap.
4. Accelerate moderately only within tire/friction and collision constraints. Never accelerate into an occupied gap.
5. DEFEND: only with no forward car and rear-car overtake/encounter prediction <=3 s. Choose a safe and reachable path to cover likely passing lane. No rapid changes or contact risk. Disable in sharp corners. After being passed in a corner, consider a normal pass on the next straight.
6. CORNER: detect from track curvature and steering demand; pre-brake to feasible corner speed about 3 s before entry; release restriction at corner exit. Respect steering angle/rate, yaw rate, and lateral acceleration bounds.
7. SIGNAL: visual cameras must see >=4 of five sequential red lamps illuminated, followed by all lamps extinguished. Fallback only if visual signal unreliable and a neighbouring car shows **sustained forward movement**; never confuse jitter with a race start. Default to stopped on uncertainty.
8. No change to official race timing, traffic light orchestration, or map rendering.

## Portability requirements
- TEAM package uses no absolute personal paths.
- Discover sibling `urrc_track_gazebo` in the same ROS workspace or via `ament_index_python` / `URRC_TRACK_PACKAGE_DIR`.
- Map selected from active Gazebo world or explicit parameter; verify name against six allowed tracks.
- Every TEAM-created ROS topic uses suffix `_11`. Frame IDs are namespaced to avoid collisions (for example `team_11/base_link`); actual required `base_link` link is geometrically at the vehicle's 3D centre.
- Validate at least start lights, all 11 grid camera views, the route and map match, launch portability, and 3 laps with traffic before calling it PASS.

## Current stage 1 deliverables
- `routes.py` — imports external official centerline and exports 18 route CSVs, 6 metadata JSON files.
- `test_routes.py` — geometric invariants on synthetic loops.
- The ROS2/Gazebo car model, camera stack, route tracker, perception, decision state machine and actual integration are **not implemented in this commit**.
