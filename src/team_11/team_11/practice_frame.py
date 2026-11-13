"""Coordinate transform for standalone practice Gazebo Ackermann odometry.

The first odometry sample of a spawned model is model-local (0, 0, 0).
Official start grid provides its world x, y and heading. No upstream edits.
"""
import json
import math
from pathlib import Path

def wrap_angle(angle):
    return math.atan2(math.sin(angle),math.cos(angle))

def local_to_world(x,y,spawn_x,spawn_y,spawn_yaw):
    c,s=math.cos(spawn_yaw),math.sin(spawn_yaw)
    return (spawn_x+c*x-s*y,spawn_y+s*x+c*y)

def read_spawn(official,track,grid):
    package=Path(official)
    if not (package/"tracks"/"processed").is_dir():
        package=package/"src"/"urrc_track_gazebo"
    data=json.loads((package/"tracks"/"processed"/(track+".json")).read_text())
    points=[p for p in data["grid"] if int(p["position"])==grid]
    if len(points)!=1:
        raise ValueError("Grid entry missing or duplicated: "+str(grid))
    p=points[0]
    return (float(p["x"]),float(p["y"]),float(p["yaw"]))
