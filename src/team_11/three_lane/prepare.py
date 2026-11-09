#!/usr/bin/env python3
"""Generate an isolated Gazebo practice world with three routes and a stationary blocker."""
from __future__ import annotations
import argparse
import csv
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from core import (TRACKS, load_centerline, export_routes, closest_lane,
                  lane_spacing)

def prepare(source, destination, track, plugin_file, pace_plugin_file):
    source = Path(source).expanduser().resolve()
    destination = Path(destination).expanduser().resolve()
    pace = source / "race/pace_cars"
    sys.path.insert(0, str(pace))
    from routes import add_car, generate
    # Existing pace tool copies all source assets into destination.
    meta = generate(source, track, destination, str(pace_plugin_file))
    world_file = Path(meta["world_file"])
    doc = ET.parse(world_file)
    world = doc.getroot().find("world")
    if world is None:
        raise RuntimeError("Missing Gazebo world")
    csv_file = source / "src/urrc_track_gazebo/tracks/processed" / (track + "_centerline.csv")
    points = load_centerline(csv_file)
    spacing = export_routes(points, destination / "team11_routes")
    grid = json.loads((source / "src/urrc_track_gazebo/tracks/processed" / (track + ".json")).read_text())["grid"][1]
    initial_lane = closest_lane(points, grid["x"], grid["y"])
    # Place one parked obstacle 40m ahead on the selected lane.
    idx = min(range(len(points)), key=lambda i: abs(points[i].s - 40.0))
    p = points[idx]
    offset = (spacing, 0.0, -spacing)[initial_lane]
    blocker = {
        "position": 99, "x": p.x-offset*math.sin(p.yaw),
        "y": p.y+offset*math.cos(p.yaw), "road_z": p.z,
        "yaw": p.yaw,
    }
    obstacle_name = add_car(world, blocker, "0.85 0.65 0.07 1")
    # The blocker is stationary. Existing pace cars remain as visible moving obstacles.
    runner = {"position": 98, "x": grid["x"], "y": grid["y"],
              "road_z": grid["road_z"], "yaw": grid["yaw"]}
    ego_name = add_car(world, runner, "0.05 0.85 0.3 1")
    # Existing pace plugin still starts on its original topic.
    ctrl = ET.SubElement(world, "plugin", filename=str(Path(plugin_file).resolve()),
                         name="urrc::Team11ThreeLane")
    for tag, val in (
        ("start_topic", f"/urrc/{track}/pace/start"),
        ("ego_name", ego_name),
        ("obstacle_name", obstacle_name),
        ("route_prefix", str((destination / "team11_routes").resolve())),
        ("initial_lane", str(initial_lane)),
        ("spacing_m", str(spacing)),
        ("wheelbase_m", ".33"),
        ("base_speed_mps", "10"),
        ("steering_slow_threshold_deg", "10"),
        ("slow_factor", ".8"),
        ("lane_change_length_m", "9"),
        ("scan_ahead_m", "35"),
    ):
        ET.SubElement(ctrl, tag).text = val
    ET.indent(doc)
    result = world_file.with_name(track + "_team11.sdf")
    doc.write(result, encoding="utf-8", xml_declaration=True)
    manifest = {
        "track": track, "source_repo": str(source), "world_file": str(result),
        "resource_path": meta["resource_path"], "ego": ego_name,
        "obstacle": obstacle_name, "first_lane": initial_lane,
        "lane_spacing_m": spacing, "speed_mps": 10,
        "steering_threshold_deg": 10, "slow_speed_mps": 8,
        "routes": [str(destination/"team11_routes"/f"lane_{j}.csv") for j in range(3)],
        "note": "Kinematic demonstration: pose control, not force/steering joint simulation"
    }
    (destination/"team11_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return manifest

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("track", choices=TRACKS)
    parser.add_argument("--source", default="~/urrc_2")
    parser.add_argument("--output", default=None)
    parser.add_argument("--plugin", default="liburrc_team11.so")
    parser.add_argument("--pace-plugin", default="liburrc_pace_cars.so")
    args = parser.parse_args()
    root = Path(args.source).expanduser().resolve()
    out = Path(args.output).expanduser() if args.output else root/"generated/team11"/args.track
    result = prepare(root, out, args.track, args.plugin, args.pace_plugin)
    print(json.dumps(result, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
