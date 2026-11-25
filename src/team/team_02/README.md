# TEAM02 — native coordinate replay (Gazebo Harmonic)

TEAM02 now runs its existing **direct-pose coordinate method** rather than
the Python Ackermann path follower. This change is limited to this team folder.
The official operator still owns the world, spawning, READY, five-red
lights, race gate and lap timing.

## Runtime

- The **active** motion system is `pose_plugin/Team02Pose.cc`, built as
  `libteam02_pose_02.so` with ROS 2 ament_cmake + gz-sim8.
- The saved prior implementation is `legacy/BattleReplay.cc`. It is a
  **reference archive**, not the active plugin. The active code reuses its
  interpolation, easing, yaw blending and baseline speed rules, but is
  **not an exact copy of its full 1100-line three-car strategy**.
- `team02_race/control.py` handles camera red-to-dark and the organizer's
  `/race/start/team_02`; no Python wheel steering, LiDAR path following,
  or Ackermann drive plugin is used.
- `team_01` replaces the old reference script's `practice_s_09` role;
  `team_02` replaces `practice_rear_02`. Both are observed from Gazebo
  world pose only and **never moved by TEAM02**. Without either opponent,
  TEAM02 runs solo on saved coordinates.
- No new workspace is needed. This folder is placed at
  `<official-workspace>/src/team/team_02`.

## Motion and heading

At spawn, the vehicle's real grid pose is projected onto one of the six
bundled centerlines; the original lateral starting offset is retained.
Each physics update moves along the stored s/x/y/z/yaw records, interpolating
between coordinates and applying lateral offsets. Body yaw is blended
towards the measured displacement tangent each step. This includes the
diagonal heading of a smooth lane change: the car is **not held at a fixed
heading while sliding sideways**.

Based on the original BattleReplay: 10 m/s nominal cruise, 9 m/s tight
corner target, +3 m/s² acceleration, -5 m/s² braking, 15 m/s overall speed
limit, 2.5 s fifth-order lateral motion. Observed real front traffic can
slow the car or cause a gradual lane change. Actual mean speed is measured
by Gazebo; it is not guaranteed to be exactly 10 m/s.

This is **kinematic pose replay**, not physically steered Ackermann
driving. The original full multi-car scenario replay is not automatically
validated by the new runtime, and Gazebo physical collisions/tire force
behaviour cannot be inferred from replay alone.

## Apply to the existing official workspace

The official commit and TEAM02 source live in **different GitHub repos**.
To update TEAM02 without reverting the official workspace files:

```bash
cd ~/urrc_2_f1
git fetch origin main
git restore --source=origin/main --worktree -- src/team/team_02
bash src/team/team_02/run_monaco.sh
```

The run script performs official registration/build, loads the native
Gazebo plugin search path, creates missing Monaco visual assets, then
starts Monaco **1st grid** rehearsal. Type READY in the terminal after
the track appears.

**No Gazebo build/launch/lap completion has been executed by the assistant.**
The user must provide actual compilation and rehearsal output before
claiming PASS.

## TEAM02 verified-baseline three-lap rehearsal (renamed)

The exact C++ `kinematic/BattleReplay.cc` three-car racing strategy has been
copied to `src/team/team_02/legacy/BattleReplay08.cc` (plugin alias
`urrc::BattleReplay_02`). The original `team_11` program remains
unchanged for regression comparison. This TEAM02 variant renames:

| Old | TEAM02 scenario |
| --- | --- |
| `team_11` | `team_02` |
| `practice_s_09` | `team_01` |
| `practice_rear_11` | `team_02` |

The Gazebo plugin library is `libteam02_battle_replay_02.so`.
The private start topic is `/team02/battle_start_02` and readiness topic
is `/team02/battle_ready_02`.

Run in the existing `~/urrc_2_f1` after fetching main:

```bash
cd ~/urrc_2_f1
git pull --ff-only origin main
bash run.sh battle08 monaco 2 1 race3
# After the previous run finishes:
bash run.sh battle08 monaco 3 3 race3
```

This is the original **three scripted vehicle scenario**, now using TEAM02
identifiers: the entire scenario plugin controls all three models'
`SetWorldPoseCmd`. The vehicles are **not** separate autonomous entrants
in this rehearsal. The ROS2 official `race.launch.py` continues to use
the separate `pose_plugin/Team02Pose.cc` controller and must not be
described as having the complete proven strategy until it is ported there
and independently tested. The renamed rehearsal has not yet been run
on a real ROS2+Gazebo machine.

## TEAM02 start policy (2026-11-18)

TEAM02 **does not use a camera**, including for start detection.
The controller subscribes only to the organizer's
`/race/start/team_02` (ROS 2 Bool) and bridges that value to Gazebo
`/team02/replay_go_02`. The operator must publish TRUE **only after all
five start lights have switched off**. There is no timer, neighbor-vehicle,
or image-triggered release. When FALSE, `Team02Pose_02` continuously
commands the original spawn pose and publishes zero speed; check actual
Gazebo prestart displacement in a live test before declaring a zero-motion
guarantee. The scripted `battle08` baseline uses its independent Gazebo
lights-out start and remains unchanged.

The current official `urrc_f1_ws` v21/v24 rehearsal enforces a per-team
camera-frame readiness check even in signal mode. TEAM02 has no camera,
so **that operator-side readiness policy is incompatible** and must be
adjusted/approved by the organizer before the official rehearsal can pass.
Do not re-enable a camera simply to bypass the rule.
