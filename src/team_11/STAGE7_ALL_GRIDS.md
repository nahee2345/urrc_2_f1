# TEAM 11 — 11 starting-grid camera validation

## Root cause of prior failure

The attached summary.csv shows monza grid 1 and grid 2 both **FAIL exit_124**.
Exit code 124 is the outer `timeout 160s` watchdog, **not proof that a
specific grid's camera missed the red signal**. The previous runner started a
separate full Gazebo GUI world for every grid and timed out.

## Changes
- Build once and open ONE official Gazebo world. Each slot only removes and
  respawns the TEAM 11 car at an official grid coordinate in the LOCAL
  verification launcher. Those coordinates are NEVER used for racing decisions.
- Restart camera start gate, adapter, and one-second trial driver for each slot.
- Keep one ROS↔Gazebo bridge alive and bridge both camera topics.
- Wide view (/camera_11, existing) covers close grids.
- Narrow view (/camera_zoom_11, new 0.65 rad 640×360) covers tiny distant lamps;
  physically mounted at the same onboard camera position.
- Each camera has an independent fail-closed lamp-on then all-off state machine.
  Neither camera is used for steering, road detection or vehicle localization.
- Apply a per-light-sequence timeout. Keep one separate camera, drive, signal,
  and spawn log for every grid.
- `summary.csv` is written after EACH slot, so Ctrl+C preserves finished
  results. FAIL reasons include no camera frames, failed official lights,
  and peak lamp counts.

## One-command validation

Close any existing Gazebo window, then:

```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && PYTHONPATH=src/team_11 python3 -m unittest discover -s src/team_11 -p 'test_*.py' -v && bash src/team_11/tests/run_stage7_all_grids.sh monza
```

For a failed slot, read:
```bash
cat ~/urrc_2_f1/log/team11_stage7_all_grids/monza/summary.csv
tail -40 ~/urrc_2_f1/log/team11_stage7_all_grids/monza/grid_02/camera.log
tail -30 ~/urrc_2_f1/log/team11_stage7_all_grids/monza/grid_02/console.log
```

The GitHub changes are not a substitute for executing the simulation on the
operator PC. Each grid is only considered actually verified after the above
integration test reports PASS for that specific grid. Official race runner
integration and operator race_start permission remain a separate required step.
