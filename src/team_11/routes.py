#!/usr/bin/env python3
"""Generate right-to-left race lines from the official URRC centerline files.

No modification of official track data. Python standard library only.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import os
from pathlib import Path

TRACKS = ("monza", "spa", "monaco", "cota", "hungaroring", "baku")
CAR_WIDTH_M = 0.25
WALL_CLEARANCE_M = 0.50
MIN_LANE_SEPARATION_M = 0.15

def track_package(path: str | None = None) -> Path:
    if path:
        p = Path(path).expanduser().resolve()
        if (p / "tracks" / "processed").is_dir():
            return p
        if (p / "src" / "urrc_track_gazebo" / "tracks" / "processed").is_dir():
            return p / "src" / "urrc_track_gazebo"
        raise FileNotFoundError("Missing tracks/processed in " + str(p))
    env = os.environ.get("URRC_TRACK_PACKAGE_DIR")
    if env:
        return track_package(env)
    here = Path(__file__).resolve()
    for base in here.parents:
        for p in (base / "src" / "urrc_track_gazebo", base / "urrc_track_gazebo"):
            if (p / "tracks" / "processed").is_dir():
                return p
    try:
        from ament_index_python.packages import get_package_share_directory
        return track_package(get_package_share_directory("urrc_track_gazebo"))
    except (ImportError, LookupError, FileNotFoundError):
        pass
    raise FileNotFoundError("Set URRC_TRACK_PACKAGE_DIR to the official urrc_track_gazebo package root")

def read_centerline(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        data = list(csv.DictReader(stream))
    if len(data) < 3:
        raise ValueError("Centerline needs >=3 points")
    out = []
    for r in data:
        pt = {k: float(r[k]) for k in ("x_m", "y_m", "z_m", "yaw_rad", "road_width_m")}
        if not all(math.isfinite(v) for v in pt.values()):
            raise ValueError("Nonfinite track point")
        if pt["road_width_m"] <= 0:
            raise ValueError("Invalid road width")
        out.append(pt)
    return out

def generate(centerline, car_width=CAR_WIDTH_M, clearance=WALL_CLEARANCE_M):
    lanes = {1: [], 2: [], 3: []}
    narrow_points = 0
    for p in centerline:
        # Positive offset is driver's left; right-hand lane gets a negative offset.
        maximum = max(0.0, p["road_width_m"] / 2 - car_width / 2 - clearance)
        narrow = maximum < MIN_LANE_SEPARATION_M
        if narrow:
            narrow_points += 1
        yaw = p["yaw_rad"]
        left_x, left_y = -math.sin(yaw), math.cos(yaw)
        for number, offset in ((1, -maximum), (2, 0.0), (3, maximum)):
            lanes[number].append({
                "x_m": p["x_m"] + left_x * offset,
                "y_m": p["y_m"] + left_y * offset,
                "z_m": p["z_m"],
                "yaw_rad": yaw,
                "road_width_m": p["road_width_m"],
                "lane_offset_m": offset,
                "safe_to_change_lane": not narrow,
            })
    return lanes, narrow_points

def export_tracks(package: Path, output: Path):
    output.mkdir(parents=True, exist_ok=True)
    summary = {}
    for track in TRACKS:
        source = package / "tracks" / "processed" / (track + "_centerline.csv")
        if not source.is_file():
            raise FileNotFoundError(source)
        points = read_centerline(source)
        lanes, narrow = generate(points)
        dst = output / track
        dst.mkdir(exist_ok=True)
        for number, rows in lanes.items():
            with (dst / ("lane_" + str(number) + ".csv")).open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        meta = {"track": track, "source": source.name, "samples": len(points),
                "lanes": 3, "narrow_samples": narrow,
                "car_width_m": CAR_WIDTH_M, "wall_clearance_m": WALL_CLEARANCE_M,
                "offset_sign": "positive is left",
                "lane_order": "1=right,2=center,3=left",
                "loop": True, "source_width_quality": "estimated; verify collision geometry"}
        (dst / "metadata.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
        summary[track] = meta
    return summary

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--track-package", help="path to official urrc_track_gazebo")
    parser.add_argument("--output", default=str(Path(__file__).resolve().parent / "generated_routes"))
    args = parser.parse_args()
    summary = export_tracks(track_package(args.track_package), Path(args.output).resolve())
    print(json.dumps(summary, indent=2))

if __name__ == "__main__":
    main()
