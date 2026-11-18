#!/usr/bin/env python3
"""Repair camera bridges in existing independent Monaco practice entrants.
Preserve maps, models, controllers, team_08 and official race files.
"""
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[2]
for n in ("01", "02"):
    team = f"team_{n}"
    folder = root / "src" / "team" / team
    launch = folder / "launch" / "race.launch.py"
    model = folder / "models" / "practice_car" / "model.sdf"
    if not launch.is_file() or not model.is_file():
        raise SystemExit(f"{team} missing; run official sync/practice-team generator first")
    topic = f"/team{n}/front_camera_{n}/image"
    if topic not in model.read_text():
        raise SystemExit(f"{team} camera model topic not found: {topic}")
    content = launch.read_text()
    bridge = f'"{topic}@sensor_msgs/msg/Image[gz.msgs.Image"'
    if bridge not in content:
        anchor = f'"/team{n}/front_scan_{n}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"'
        if anchor not in content:
            raise SystemExit(f"{team} expected lidar bridge missing")
        content = content.replace(anchor, anchor + ",\n                " + bridge)
    if '"use_sim_time"' not in content:
        anchor = f'name="team{n}_practice_control",output="screen")'
        if anchor not in content:
            raise SystemExit(f"{team} expected control node missing")
        content = content.replace(anchor, f'name="team{n}_practice_control",output="screen",parameters=[{{"use_sim_time": True}}])')
    if "/race/start/" not in content:
        content = "# Official start subscription: /race/start/team_" + n + "\n" + content
    compile(content,str(launch),"exec")
    launch.write_text(content)
    print(f"{team}: camera ROS bridge and preflight launch metadata OK")
print("No official operator or team_08 driving logic changed.")
