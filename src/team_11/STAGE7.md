# TEAM 11 Stage 7: official five-red-light camera start

The one-command test **must not move** before the camera sees at least four illuminated red lamps followed by all lamps going dark. Darkness at first observation never enables the car.

## Run
```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && PYTHONPATH=src/team_11 python3 -m unittest discover -s src/team_11 -p "test_*.py" -v && bash src/team_11/tests/run_stage7.sh monza 1
```

Official Monza can be replaced by spa, monaco, cota, hungaroring or baku; grid 1..11.

## Implementation
- Official race world and official traffic_lights_v10.py used without any TEAM11 map edits.
- Unlike run_stage5.sh, this runner opens the official SDF with stage5_world.sh so the official traffic-light sequence is launched **after** the ROS camera detector and adapter are ready.
- Team-owned camera /camera_11 -> ROS Image bridge, pixel clustering detector -> /start_allowed_11 Bool.
- Require >=4 bright red candidate lamps for >=2 frames, then zero for >=3 frames within two seconds. Do not recognize a green lamp.
- Adapter gates both inputs with a 0.35s authorization watchdog; manual bypass disabled.
- 1-second 0.65m/s test driver only publishes after camera permission, followed by STOP.
- Camera sensor is used only for starting; no world pose, official timing topics or stored map waypoints are read.

## Diagnostics
```bash
cat ~/urrc_2_f1/log/team11_stage7/lights.log
cat ~/urrc_2_f1/log/team11_stage7/camera.log
cat ~/urrc_2_f1/log/team11_stage7/drive.log
```

**Not yet confirmed in Gazebo.** RGB thresholds, resolution, gantry view and visual lighting must be checked on every official grid. If the camera does not see four lamps, the intended safe behavior is NO START.

For formal official races, camera start also requires official /race/start/team_11 Bool true, with false as emergency stop. This separate runner only validates the camera portion; integration with official organizer launch remains.
