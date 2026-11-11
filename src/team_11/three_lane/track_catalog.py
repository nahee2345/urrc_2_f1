"""Offline precomputed TEAM11 practice tracks; NOT permitted in the official LiDAR-only race."""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
MAPS = HERE / "maps"
TRACKS = ("monza", "spa", "monaco", "cota", "hungaroring", "baku")


def map_dir(track: str) -> Path:
    if track not in TRACKS:
        raise ValueError(f"Unknown TEAM11 map: {track!r}")
    return MAPS / track


def metadata(track: str) -> dict:
    return json.loads((map_dir(track) / "track.json").read_text(encoding="utf-8"))


def map_file(track: str, filename: str) -> Path:
    if filename not in ("centerline.csv", "lane_0.csv", "lane_1.csv", "lane_2.csv", "track.json"):
        raise ValueError(f"Unrecognized track asset {filename!r}")
    path = map_dir(track) / filename
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def identify_world(world_file: str | Path) -> str:
    path = Path(world_file)
    root = ET.parse(path).getroot()
    world = root if root.tag == "world" else root.find("world")
    name = world.get("name", "") if world is not None else ""
    if name not in TRACKS:
        raise ValueError(f"World {name!r} is not one of the six bundled tracks")
    return name


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def identify_spawn(x: float, y: float, yaw: float, *, tolerance_m=0.12,
                   tolerance_rad=0.08) -> str:
    """Identify an unchanged official painted grid pose, never by XY alone.

    Raise rather than guessing on changed or ambiguous maps. Spawn pose is
    available from simulation only; this mode is practice-only.
    """
    matches = []
    for name in TRACKS:
        for grid in metadata(name)["grid"]:
            distance = math.hypot(x - grid["x"], y - grid["y"])
            delta = abs(_wrap(yaw - grid["yaw"]))
            if distance <= tolerance_m and delta <= tolerance_rad:
                matches.append((distance / tolerance_m + delta / tolerance_rad, name))
    if not matches:
        raise ValueError("Spawn pose does not match a known grid: refuse map selection")
    matches.sort()
    if len(matches) > 1 and matches[0][1] != matches[1][1] and matches[1][0] - matches[0][0] < 0.5:
        raise ValueError("Ambiguous spawn pose: explicit world name is required")
    return matches[0][1]


def choose(track: str, *, world_file=None, spawn=None) -> str:
    if track != "auto":
        map_dir(track)
        return track
    if world_file is not None:
        return identify_world(world_file)
    if spawn is not None:
        return identify_spawn(*spawn)
    raise ValueError("track=auto requires --world or --spawn; guessing is unsafe")


def find_source() -> Path:
    """Find assets from this repository submodule or when TEAM11 is copied into host."""
    root = HERE.parents[2]
    choices = [root / "upstream" / "urrc_f1_ws", root]
    extra = os.environ.get("URRC_WORLD_SOURCE")
    if extra:
        choices.insert(0, Path(extra).expanduser())
    for folder in choices:
        if (folder / "src/urrc_track_gazebo/tracks/processed/monza_centerline.csv").is_file():
            return folder.resolve()
    raise FileNotFoundError("Official world assets missing; initialize submodule or provide --source")


def _git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\\0" + data).hexdigest()


def validate_bundle(track: str, source=None) -> dict:
    """Reject altered source maps and malformed stored paths before motion."""
    meta = metadata(track)
    original = map_file(track, "centerline.csv")
    data = original.read_bytes()
    if _git_blob_sha(data) != meta["source_centerline_sha"]:
        raise ValueError(f"{track}: embedded centerline hash mismatch")
    with original.open(newline="", encoding="utf-8") as fh:
        original_rows = list(csv.DictReader(fh))
    if len(original_rows) != meta["centerline_points"] or len(original_rows) < 100:
        raise ValueError(f"{track}: invalid centerline length")
    space = float(meta["route_spacing_m"])
    if not (0.2 < space < 2.0):
        raise ValueError(f"{track}: unsafe road spacing")
    for lane in range(3):
        with map_file(track, f"lane_{lane}.csv").open(newline="", encoding="utf-8") as fh:
            lane_rows = list(csv.DictReader(fh))
        if len(lane_rows) != len(original_rows):
            raise ValueError(f"{track}: route {lane} differs in sample count")
        previous_s = -1.
        for i in range(len(lane_rows)):
            p, q = original_rows[i], lane_rows[i]
            s = float(q["distance_m"])
            if s < previous_s - 1e-6:
                raise ValueError(f"{track}: route {lane} distance is not monotonic")
            previous_s = s
            yaw = float(p["yaw_rad"])
            off = (1-lane)*space
            xx = float(p["x_m"]) - off*math.sin(yaw)
            yy = float(p["y_m"]) + off*math.cos(yaw)
            if abs(float(q["x_m"])-xx) > 2e-6 or abs(float(q["y_m"])-yy) > 2e-6:
                raise ValueError(f"{track}: route {lane} geometry mismatch")
    if source is not None:
        external = (Path(source) / "src/urrc_track_gazebo/tracks/processed" /
                    f"{track}_centerline.csv")
        if not external.is_file():
            raise FileNotFoundError(external)
        if _git_blob_sha(external.read_bytes()) != meta["source_centerline_sha"]:
            raise ValueError(f"{track}: official source revision does not match stored route")
    return {"track":track, "samples":len(original_rows), "spacing_m":space}
