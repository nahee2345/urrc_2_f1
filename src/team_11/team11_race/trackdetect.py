"""Six-map identification using this team's own world-topic list or initial pose."""
from __future__ import annotations
import json
import math
from pathlib import Path
import re
import subprocess

TRACKS = ("monza", "spa", "monaco", "cota", "hungaroring", "baku")
WORLD_TOPIC = re.compile(r"^/world/([a-z0-9_]+)(?:/|$)")


def map_assets(share: Path, name: str) -> Path:
    if name not in TRACKS:
        raise ValueError("Unrecognized live map: " + repr(name))
    directory = share / "maps" / name
    for required in ("centerline.csv", "track.json", "lane_0.csv", "lane_1.csv", "lane_2.csv"):
        if not (directory / required).is_file():
            raise FileNotFoundError(directory / required)
    return directory


def detect_from_topic_list(output: str) -> str | None:
    worlds = {match.group(1) for line in output.splitlines()
              if (match := WORLD_TOPIC.match(line.strip())) and match.group(1) in TRACKS}
    if len(worlds) > 1:
        raise ValueError("More than one active supported Gazebo world: " + str(sorted(worlds)))
    return next(iter(worlds), None)


def discover_world() -> str | None:
    """Read Gazebo public topic names; never touch the organizer source/world."""
    try:
        result = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True,
                                timeout=2.0, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return detect_from_topic_list(result.stdout) if result.returncode == 0 else None


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a-b), math.cos(a-b))


def matching_grid(assets: Path, xyz_yaw, max_distance=.75, max_yaw=.35):
    """Identify start slot, fail closed when no matching painted grid exists."""
    x,y,z,yaw=xyz_yaw
    data=json.loads((assets/"track.json").read_text(encoding="utf-8"))
    matches=[]
    for g in data["grid"][:12]:
        distance=math.hypot(x-float(g["x"]),y-float(g["y"]))
        delta=abs(angle_diff(yaw,float(g["yaw"])))
        height=abs(z-(float(g["road_z"])+.04))
        if distance<=max_distance and delta<=max_yaw and height<=.45:
            matches.append((distance+delta+height,g["position"]))
    if not matches:
        raise ValueError("TEAM11 spawn not located at an expected painted grid")
    matches.sort()
    if len(matches)>1 and matches[1][0]-matches[0][0]<.03:
        raise ValueError("Ambiguous TEAM11 spawn grid")
    return matches[0][1]


def match_map_from_spawn(share: Path, xyz_yaw) -> tuple[str,int] | None:
    found=[]
    for track in TRACKS:
        try:
            slot=matching_grid(map_assets(share,track),xyz_yaw,
                               max_distance=.16,max_yaw=.12)
        except (ValueError,FileNotFoundError):
            continue
        found.append((track,slot))
    if len(found)>1:
        raise ValueError("Spawn matches multiple official maps; refuse movement")
    return found[0] if found else None
