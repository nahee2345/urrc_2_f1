# TEAM08 independent race package

This folder is the ONLY folder to copy to a host URRC workspace:

```
<workspace>/src/team/team_08/
```

Supported source-map names: monza, spa, monaco, cota, hungaroring, baku.
The six embedded maps each include three lane CSVs, a centerline, start grids
and track metadata. Do not move only the Python module out of this folder.

## Runtime requirements

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic and the ROS/Gazebo bridge.
The host must use compatible team submission and race-start interfaces.
The organizer owns world startup, light sequencing, operator READY and timing.
TEAM08 does NOT modify any external world or other team.

The root `race_entry.json` identifies the package as `team_08`.
The official intake normalizes the single folder into a ROS package
under its `src/` directory and builds `team08_race`.

Private topics are `/team08/*_08`. The organizer-owned gate is
`/race/start/team_08`; vehicle name is `team_08`.
See `launch/race.launch.py` for the current bridge and node topology.

## Driving and fallback

The running controller in `team08_race/` uses the embedded maps and
the car's own Gazebo pose and LiDAR, not the battle replay plugin.
It detects the world and start grid, chooses the nearest lane and uses
the original team's corner-centering, overtaking and rear-defense planners.
If the grid is unknown but the world is known, it can select a lane
matching both vehicle position and forward heading. While searching
for a valid route AFTER the authorized start, it may drive straight
at 2 m/s for a maximum of 1.5 s, only with fresh forward LiDAR
and at least 2 m forward clearance. Otherwise it stops.
No stale pose, opposite-direction route or unknown map is accepted.

The camera-based five-light transition is used with the official
race gate. The official race gate remains mandatory.

## Existing race3 algorithm retained

`legacy/BattleReplay.cc` is an archived TEAM08-named copy of
the latest Monaco three-car battle strategy from `urrc_2_f1`
(commit c1bde58). It preserves the original reference strategy
but is NOT a ROS controller and is NOT compiled or executed here.
Its SetWorldPoseCmd 3-lap results are not proof of physical steering
or successful autonomous 3-lap Gazebo runs.

## Check after git pull

```bash
cd ~/urrc_2_f1
python3 -m compileall -q src/team/team_08
python3 tools/check_team08_portable.py
# On the official host, use its official intake check and sync commands.
```

Real ROS2 + Gazebo multicar driving and interoperability with each
host deployment are still unverified.
