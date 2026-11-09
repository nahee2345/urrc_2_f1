#!/usr/bin/env python3
"""Generate three routes and the grid-01 car without a stationary obstacle."""
from __future__ import annotations
import argparse
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from core import (TRACKS, load_centerline, export_routes, closest_lane,
                  lane_spacing)

def prepare(source, destination, track, plugin_file, pace_plugin_file, start_mode='camera'):
    if start_mode not in ('camera', 'immediate'):
        raise ValueError(f'Unsupported start mode: {start_mode}')
    source = Path(source).expanduser().resolve()
    destination = Path(destination).expanduser().resolve()
    pace = source / "race/pace_cars"
    sys.path.insert(0, str(pace))
    from routes import add_car, generate
    # Source pace generator independently restores grids 02 and 09 for every run.
    meta = generate(source, track, destination, str(pace_plugin_file))
    world_file = Path(meta["world_file"])
    doc = ET.parse(world_file)
    world = doc.getroot().find("world")
    if world is None:
        raise RuntimeError("Missing Gazebo world")
    csv_file = source / "src/urrc_track_gazebo/tracks/processed" / (track + "_centerline.csv")
    points = load_centerline(csv_file)
    spacing = export_routes(points, destination / "team11_routes")
    grid = json.loads((source / "src/urrc_track_gazebo/tracks/processed" / (track + ".json")).read_text())["grid"][0]
    initial_lane = closest_lane(points, grid["x"], grid["y"])
    runner = {"position": 1, "x": grid["x"], "y": grid["y"],
              "road_z": grid["road_z"], "yaw": grid["yaw"]}
    ego_name = add_car(world, runner, "0.96 0.82 0.04 1")
    ego_model = world.find(f"model[@name='{ego_name}']")
    link = ego_model.find("link[@name='base_link']")
    camera_topic = f"/urrc/{track}/team11/front/image"
    camera = ET.SubElement(link, "sensor", name="team11_front_camera", type="camera")
    ET.SubElement(camera, "pose").text = "0.20 0 0.24 0 -0.43 0"
    ET.SubElement(camera, "always_on").text = "true"
    ET.SubElement(camera, "update_rate").text = "20"
    ET.SubElement(camera, "visualize").text = "false"
    ET.SubElement(camera, "topic").text = camera_topic
    settings = ET.SubElement(camera, "camera")
    ET.SubElement(settings, "horizontal_fov").text = "1.3"
    image = ET.SubElement(settings, "image")
    ET.SubElement(image, "width").text = "640"
    ET.SubElement(image, "height").text = "360"
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(settings, "clip")
    ET.SubElement(clip, "near").text = "0.1"
    ET.SubElement(clip, "far").text = "80"
    # TEAM11 starts as soon as the model is available; pace cars retain their light sequence.
    ctrl = ET.SubElement(world, "plugin", filename=str(Path(plugin_file).resolve()),
                         name="urrc::Team11ThreeLane")
    for tag, val in (
        ("camera_topic", camera_topic),
        ("start_mode", start_mode),
        ("ego_name", ego_name),
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
    # Reject silently missing practice vehicles or overlapping TEAM11 spawns.
    expected = {
        "urrc_pace_grid_01": 1,
        "urrc_pace_grid_02": 2,
        "urrc_pace_grid_09": 9,
    }
    models = {model.get("name"): model for model in world.findall("model")}
    for name in expected:
        if name not in models:
            raise RuntimeError(f"Required vehicle missing from generated world: {name}")
    pace_plugins = [
        plugin for plugin in world.findall("plugin")
        if plugin.get("name") == "urrc::PaceCars"
    ]
    if len(pace_plugins) != 1:
        raise RuntimeError(f"Expected exactly one pace plugin; found {len(pace_plugins)}")
    names = [car.findtext("name") for car in pace_plugins[0].findall("car")]
    if sorted(names) != ["urrc_pace_grid_02", "urrc_pace_grid_09"]:
        raise RuntimeError(f"Original practice cars not restored: {names}")
    grid_data = json.loads(
        (source / "src/urrc_track_gazebo/tracks/processed" / (track + ".json")).read_text()
    )["grid"]
    for name, number in expected.items():
        pose = [float(v) for v in models[name].findtext("pose").split()]
        original = grid_data[number-1]
        if (abs(pose[0] - original["x"]) > 1e-6 or
            abs(pose[1] - original["y"]) > 1e-6 or
            abs(pose[5] - original["yaw"]) > 1e-6):
            raise RuntimeError(f"{name} is not at painted grid {number}")
    print(f"[TEAM11] Grid 01=yellow TEAM11, Grid 02=red practice (10m/s), "
          f"Grid 09=blue practice (20m/s); all present", flush=True)
    ET.indent(doc)
    result = world_file.with_name(track + "_team11.sdf")
    doc.write(result, encoding="utf-8", xml_declaration=True)
    manifest = {
        "track": track, "source_repo": str(source), "world_file": str(result),
        "resource_path": meta["resource_path"], "ego": ego_name,
        "first_lane": initial_lane,
        "start_mode": start_mode,
        "grid_position": 1, "start_camera_topic": camera_topic,
        "practice_cars": [name for name in sorted(names)],
        "grid_pose": [grid["x"], grid["y"], grid["road_z"], grid["yaw"]],
        "lane_spacing_m": spacing, "speed_mps": 10,
        "steering_threshold_deg": 10, "corner_speed_range_mps": [3, 7],
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
    parser.add_argument("--start-mode", choices=("camera", "immediate"), default="camera")
    args = parser.parse_args()
    root = Path(args.source).expanduser().resolve()
    out = Path(args.output).expanduser() if args.output else root/"generated/team11"/args.track
    result = prepare(root, out, args.track, args.plugin, args.pace_plugin, args.start_mode)
    print(json.dumps(result, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
