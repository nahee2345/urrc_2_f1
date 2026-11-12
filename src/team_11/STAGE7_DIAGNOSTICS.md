# TEAM 11 stage 7 camera detection — diagnostic fix

- Changed `/camera_11` subscriber to ROS 2 `qos_profile_sensor_data` (BEST_EFFORT), compatible with typical sensor/ros_gz_bridge data publishers.
- `camera.log` logs received frames, image encoding, dimensions and maximum recognized red-lamp count.
- The runner now confirms at least one image diagnostic before triggering the official lights. If images are unavailable, it stops safely.
- The official `traffic_lights_v10.py` deliberately completes from first red to all-off in roughly **3.35–5.05 seconds** (strict allowable 3–6 seconds), so the perceived quick sequence is by original design. No official signal timing has been modified.
- Pixel thresholds and physical camera view may still prevent detection. `peak_lamps=0` with positive frames means vision calibration is needed; no camera frames means bridge/QoS/stream failure.

Update and test:
```bash
cd ~/urrc_2_f1 && git pull --ff-only origin main && PYTHONPATH=src/team_11 python3 -m unittest discover -s src/team_11 -p 'test_*.py' -v && bash src/team_11/tests/run_stage7.sh monza 1
```

After the test, inspect:
```bash
tail -40 ~/urrc_2_f1/log/team11_stage7/camera.log
tail -25 ~/urrc_2_f1/log/team11_stage7/lights.log
tail -25 ~/urrc_2_f1/log/team11_stage7/bridge.log
```

Never bypass start for a formal race and do not change upstream race signal duration without organizer approval.