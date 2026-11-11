#!/usr/bin/env python3
"""Generate three routes and the grid-01 car without a stationary obstacle."""
from __future__ import annotations
import argparse
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from core import (TRACKS, load_centerline, export_routes, closest_lane,
                  lane_spacing)

def prepare(source, destination, track, plugin_file, pace_plugin_file, start_mode='camera', lane='auto', fixed_lane=False, grid_position=1):
    if start_mode not in ('camera', 'immediate'):
        raise ValueError(f'Unsupported start mode: {start_mode}')
    if str(lane) not in ('auto','1','2','3'):
        raise ValueError(f'Invalid lane {lane}; expected auto or 1..3')
    if not 1 <= grid_position <= 12:
        raise ValueError('TEAM11 start grid must be 1..12')
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
    grid = json.loads((source / "src/urrc_track_gazebo/tracks/processed" / (track + ".json")).read_text())["grid"][grid_position-1]
    # TEAM11 occupies its own selected grid. Suppress an original pace car
    # only if the selected grid is also a pace-car starting position.
    if grid_position in (2,9):
        original_name = f"urrc_pace_grid_{grid_position:02d}"
        original = world.find(f"model[@name='{original_name}']")
        if original is None:
            raise RuntimeError(f'Original grid practice car not found: {original_name}')
        world.remove(original)
        pace_plugin = next(p for p in world.findall('plugin')
                           if p.get('name') == 'urrc::PaceCars')
        for car in list(pace_plugin.findall('car')):
            if car.findtext('name') == original_name:
                pace_plugin.remove(car)
    nearest_lane = closest_lane(points, grid["x"], grid["y"])
    # Fixed starting-grid mapping, with explicit --lane overriding for diagnostics.
    initial_lane = (2 if grid_position % 2 else 0) if str(lane) == "auto" else int(lane) - 1
    runner = {"position": grid_position, "x": grid["x"], "y": grid["y"],
              "road_z": grid["road_z"], "yaw": grid["yaw"]}
    ego_name = add_car(world, runner, "0.96 0.82 0.04 1")
    ego_model = world.find(f"model[@name='{ego_name}']")
    # Heavy ego mass is descriptive in this static kinematic demonstration.
    ego_link = ego_model.find("link[@name='base_link']")
    inertial = ET.SubElement(ego_link, "inertial")
    ET.SubElement(inertial, "mass").text = "50"
    tensor = ET.SubElement(inertial, "inertia")
    for tag, value in (("ixx","0.1294"),("iyy","1.1483"),
                       ("izz","1.2621"),("ixy","0"),("ixz","0"),("iyz","0")):
        ET.SubElement(tensor,tag).text=value
    pace_plugin = next(p for p in world.findall("plugin")
                       if p.get("name") == "urrc::PaceCars")
    ET.SubElement(pace_plugin, "team11_ego_name").text = ego_name
    ET.SubElement(pace_plugin, "team11_mass_kg").text = "50"
    ET.SubElement(pace_plugin, "pace_mass_kg").text = "1.5"
    link = ego_model.find("link[@name='base_link']")
    camera_topic = f"/urrc/{track}/team11/front/image"
    camera = ET.SubElement(link, "sensor", name="team11_front_camera", type="camera")
    # Aim TEAM11's camera toward the actual gantry from the chosen grid.
    gate = world.find("model[@name='race_start_gantry']")
    if gate is None:
        raise RuntimeError("Starting-light gantry missing")
    gx, gy, gz, _, _, _ = map(float, gate.findtext("pose").split())
    target_z = gz + 1.96
    yaw = grid["yaw"]
    cx = grid["x"] + .20*math.cos(yaw)
    cy = grid["y"] + .20*math.sin(yaw)
    cz = grid["road_z"] + .003 + .24
    ground_distance = math.hypot(gx-cx, gy-cy)
    if ground_distance < .5:
        raise RuntimeError("Starting light gantry is too close")
    camera_pitch = -math.atan2(target_z-cz, ground_distance)
    camera_yaw = math.atan2(gy-cy,gx-cx)-yaw
    camera_yaw = math.atan2(math.sin(camera_yaw),math.cos(camera_yaw))
    camera_hfov = .52
    ET.SubElement(camera, "pose").text = (
        f"0.20 0 0.24 0 {camera_pitch:.8f} {camera_yaw:.8f}")
    ET.SubElement(camera, "always_on").text = "true"
    ET.SubElement(camera, "update_rate").text = "20"
    ET.SubElement(camera, "visualize").text = "false"
    ET.SubElement(camera, "topic").text = camera_topic
    settings = ET.SubElement(camera, "camera")
    ET.SubElement(settings, "horizontal_fov").text = str(camera_hfov)
    image = ET.SubElement(settings, "image")
    ET.SubElement(image, "width").text = "1280"
    ET.SubElement(image, "height").text = "720"
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
        ("fixed_lane", str(fixed_lane).lower()),
        ("spacing_m", str(spacing)),
        ("wheelbase_m", ".33"),
        ("base_speed_mps", "10"),
        ("steering_slow_threshold_deg", "10"),
        ("slow_factor", ".8"),
        ("max_deceleration_mps2", "4.0"),
        ("max_acceleration_mps2", "2.0"),
        ("lane_change_length_m", "9"),
        ("scan_ahead_m", "35"),
    ):
        ET.SubElement(ctrl, tag).text = val
    # Reject silently missing practice vehicles or overlapping TEAM11 spawns.
    expected = {f"urrc_pace_grid_{grid_position:02d}": grid_position}
    for slot in (2,9):
        if slot != grid_position:
            expected[f"urrc_pace_grid_{slot:02d}"] = slot
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
    expected_pace = sorted(f"urrc_pace_grid_{slot:02d}"
                           for slot in (2,9) if slot != grid_position)
    if sorted(names) != expected_pace:
        raise RuntimeError(f"Practice cars inconsistent with TEAM11 grid {grid_position}: {names}")
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
    print(f"[TEAM11] Grid {grid_position:02d}=yellow TEAM11, "
          f"lane {initial_lane+1}, practice={names}", flush=True)
    ET.indent(doc)
    result = world_file.with_name(track + "_team11.sdf")
    doc.write(result, encoding="utf-8", xml_declaration=True)
    manifest = {
        "track": track, "source_repo": str(source), "world_file": str(result),
        "resource_path": meta["resource_path"], "ego": ego_name,
        "first_lane": initial_lane,
        "selected_lane_number": initial_lane+1,
        "nearest_lane_number": nearest_lane+1,
        "fixed_lane": fixed_lane,
        "start_mode": start_mode,
        "grid_position": grid_position, "start_camera_topic": camera_topic,
        "camera_target": [gx,gy,target_z],
        "camera_pitch_rad": camera_pitch, "camera_yaw_rad": camera_yaw,
        "camera_hfov_rad": camera_hfov, "camera_target_distance_m": ground_distance,
        "grid_lane_policy": "odd=3,even=1",
        "collision_mode": "scripted_asymmetric_bounce",
        "nominal_ego_mass_kg": 50, "nominal_other_mass_kg": 1.5,
        "practice_cars": [name for name in sorted(names)],
        "grid_pose": [grid["x"], grid["y"], grid["road_z"], grid["yaw"]],
        "lane_spacing_m": spacing, "speed_mps": 10,
        "steering_threshold_deg": 10, "corner_speed_range_mps": [3, 7],
        "max_deceleration_mps2": 4.0, "max_acceleration_mps2": 2.0,
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
    parser.add_argument("--lane", choices=("auto","1","2","3"), default="auto")
    parser.add_argument("--fixed-lane", action="store_true")
    parser.add_argument("--grid", type=int, choices=range(1,13), default=1)
    args = parser.parse_args()
    root = Path(args.source).expanduser().resolve()
    out = Path(args.output).expanduser() if args.output else root/"generated/team11"/args.track
    result = prepare(root, out, args.track, args.plugin, args.pace_plugin, args.start_mode, args.lane, args.fixed_lane, args.grid)
    print(json.dumps(result, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
