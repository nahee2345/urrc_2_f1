#!/usr/bin/env python3
"""Generate three routes and the grid-01 car without a stationary obstacle."""
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
                  lane_spacing, projected_obstacle)

def prepare(source, destination, track, plugin_file, pace_plugin_file, start_mode='camera', lane='auto', fixed_lane=False, grid_position=1, practice_cars=False, scenario='normal', physics_contact=False):
    if physics_contact and not practice_cars:
        raise ValueError('physics_contact requires practice cars')
    if scenario not in ('normal', 'slow-ahead', 'rear-fast', 'rear-contact'):
        raise ValueError(f'Unknown opponent scenario: {scenario}')
    if scenario == 'slow-ahead' and (not practice_cars or grid_position == 2):
        raise ValueError('slow-ahead requires --practice-cars and grid other than 2')
    if scenario == 'rear-contact' and (not physics_contact or grid_position == 9):
        raise ValueError('rear-contact requires --physics-contact and grid other than 9')
    if scenario == 'rear-fast' and (not practice_cars or grid_position == 9):
        raise ValueError('rear-fast requires --practice-cars and grid other than 9')
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
    # The baseline requested by the user has NO practice traffic.
    # With --practice-cars, restore all original cars except the one
    # occupying TEAM11's selected spawn slot.
    pace_plugin = next(p for p in world.findall("plugin")
                       if p.get("name") == "urrc::PaceCars")
    for slot in (2, 9):
        if practice_cars and slot != grid_position and (scenario not in ('rear-fast', 'rear-contact') or slot == 9):
            continue
        name = f"urrc_pace_grid_{slot:02d}"
        model = world.find(f"model[@name='{name}']")
        if model is not None:
            world.remove(model)
        for car in list(pace_plugin.findall("car")):
            if car.findtext("name") == name:
                pace_plugin.remove(car)
    nearest_lane = closest_lane(points, grid["x"], grid["y"])
    # Choose the real LEFT/RIGHT side of the road, not just grid parity.
    _, grid_lateral = projected_obstacle(points, grid["x"], grid["y"])
    if abs(grid_lateral) < .15:
        raise RuntimeError(f"Grid {grid_position} lies too close to centerline")
    physical_lane = 0 if grid_lateral > 0 else 2
    # Original maps are staggered (odd right, even left). Catch metadata
    # regressions instead of silently sending a car to the wrong side.
    parity_lane = 2 if grid_position % 2 else 0
    if physical_lane != parity_lane:
        raise RuntimeError(f"Grid {grid_position} actual side contradicts expected stagger")
    initial_lane = physical_lane if str(lane) == "auto" else int(lane) - 1
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
    # Physical contact mode is opt-in; default scripted mode stays identical.
    ET.SubElement(pace_plugin, "physics_contact_mode").text = str(physics_contact).lower()
    if physics_contact:
        ET.SubElement(pace_plugin, "physics_contact_topic").text = f"/urrc/{track}/team11/physics_contact"
    if physics_contact:
        ET.SubElement(world, "plugin", filename="gz-sim-contact-system",
                      name="gz::sim::systems::Contact")
        for model in world.findall("model"):
            name = model.get("name", "")
            if name not in (ego_name, "urrc_pace_grid_02", "urrc_pace_grid_09"):
                continue
            is_ego = name == ego_name
            mass = 50. if is_ego else 1.5
            model.find("static").text = "false"
            body = model.find("link[@name='base_link']")
            assert body is not None
            if body.find("inertial") is None:
                inertial = ET.SubElement(body, "inertial")
                ET.SubElement(inertial, "mass").text = str(mass)
                tensor = ET.SubElement(inertial, "inertia")
                # Solid rectangular body dimensions 0.50 x .23 x .16m.
                dims = (.50, .23, .16)
                for axis, value in (("ixx",mass*(dims[1]**2+dims[2]**2)/12.),
                                    ("iyy",mass*(dims[0]**2+dims[2]**2)/12.),
                                    ("izz",mass*(dims[0]**2+dims[1]**2)/12.),
                                    ("ixy",0.),("ixz",0.),("iyz",0.)):
                    ET.SubElement(tensor, axis).text = f"{value:.8f}"
            if is_ego:
                sensor = ET.SubElement(body, "sensor", name="team11_collision_contacts", type="contact")
                ET.SubElement(sensor, "always_on").text = "true"
                ET.SubElement(sensor, "update_rate").text = "60"
                contact_topic = f"/urrc/{track}/team11/physics_contact"
                ET.SubElement(sensor, "topic").text = contact_topic
                contact_conf = ET.SubElement(sensor, "contact")
                ET.SubElement(contact_conf, "collision").text = "body_collision"
            collision = body.find("collision[@name='body_collision']")
            if collision is not None:
                surface = ET.SubElement(collision, "surface")
                friction = ET.SubElement(surface, "friction")
                ode = ET.SubElement(friction, "ode")
                ET.SubElement(ode, "mu").text = "0.9"
                ET.SubElement(ode, "mu2").text = "0.9"

    link = ego_model.find("link[@name='base_link']")
    camera_topic = f"/urrc/{track}/team11/front/image"
    camera_ready_file = destination / "camera_ready.flag"
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
    camera_range = math.hypot(ground_distance, target_z - cz)
    lamp_spacing_px = 0.16 * (1280 / 2) / math.tan(camera_hfov / 2) / camera_range
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
    # Genuine rear-facing Gazebo LiDAR on the very same TEAM11 vehicle.
    rear_topic = f"/urrc/{track}/team11/rear_scan"
    rear = ET.SubElement(link, "sensor", name="team11_rear_lidar", type="gpu_lidar")
    ET.SubElement(rear, "pose").text = "-0.20 0 0.16 0 0 3.141592653589793"
    ET.SubElement(rear, "always_on").text = "true"
    ET.SubElement(rear, "update_rate").text = "30"
    ET.SubElement(rear, "visualize").text = "false"
    ET.SubElement(rear, "topic").text = rear_topic
    lidar = ET.SubElement(rear, "lidar")
    scan = ET.SubElement(lidar, "scan")
    horizontal = ET.SubElement(scan, "horizontal")
    for tag, value in (("samples", "721"), ("resolution", "1"),
                       ("min_angle", "-2.25"), ("max_angle", "2.25")):
        ET.SubElement(horizontal, tag).text = value
    lidar_range = ET.SubElement(lidar, "range")
    for tag, value in (("min", "0.12"), ("max", "70"), ("resolution", "0.02")):
        ET.SubElement(lidar_range, tag).text = value
    # TEAM11 starts as soon as the model is available; pace cars retain their light sequence.
    ctrl = ET.SubElement(world, "plugin", filename=str(Path(plugin_file).resolve()),
                         name="urrc::Team11ThreeLane")
    for tag, val in (
        ("camera_topic", camera_topic),
        ("rear_lidar_topic", rear_topic),
        ("camera_ready_file", str(camera_ready_file.resolve())),
        ("lamp_spacing_px", f"{lamp_spacing_px:.6f}"),
        ("start_mode", start_mode),
        ("ego_name", ego_name),
        ("route_prefix", str((destination / "team11_routes").resolve())),
        ("initial_lane", str(initial_lane)),
        ("fixed_lane", str(fixed_lane).lower()),
        ("spacing_m", str(spacing)),
        ("wheelbase_m", ".33"),
        ("base_speed_mps", "15"),
        ("physics_contact_mode", str(physics_contact).lower()),
        ("steering_slow_threshold_deg", "10"),
        ("slow_factor", ".8"),
        ("max_deceleration_mps2", "4.0"),
        ("max_acceleration_mps2", "2.0"),
        ("lane_change_length_m", "18"),
        ("scan_ahead_m", "35"),
    ):
        ET.SubElement(ctrl, tag).text = val
    # Optional reproducible integration test: move the RED 5m/s test car
    # onto TEAM11's selected lane 25m ahead. This changes GENERATED world only;
    # original painted grids and source tracks are never edited.
    if scenario == 'slow-ahead':
        red = world.find("model[@name='urrc_pace_grid_02']")
        red_config = next(c for c in pace_plugin.findall("car")
                          if c.findtext("name") == "urrc_pace_grid_02")
        route = destination / "team11_routes" / f"lane_{initial_lane}.csv"
        with route.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.reader(handle))[1:]
        grid_s, _ = projected_obstacle(points, grid["x"], grid["y"])
        lap_distance = points[-1].s + math.hypot(
            points[-1].x - points[0].x, points[-1].y - points[0].y)
        start_s = (grid_s + 25.) % lap_distance
        first = next((i for i, row in enumerate(rows)
                      if float(row[0]) >= start_s), 0)
        rotated = rows[first:] + rows[:first]
        origin_s = float(rotated[0][0])
        staged_file = destination / "team11_red_5mps_ahead.csv"
        with staged_file.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["distance_m", "x_m", "y_m", "z_m", "yaw_rad"])
            for row in rotated:
                relative = (float(row[0]) - origin_s) % lap_distance
                writer.writerow([f"{relative:.9f}", *row[1:5]])
        red_config.find("route_file").text = str(staged_file.resolve())
        red_config.find("speed_mps").text = "5"
        x, y, z, yaw = (float(v) for v in rotated[0][1:5])
        red.find("pose").text = (
            f"{x:.8f} {y:.8f} {z+.003:.8f} 0 0 {yaw:.8f}")
    # Only the generated test world is staged: 20m/s blue car starts
    # 110m behind TEAM11 in the neighboring central lane.
    # Red 10m/s car is absent in this scenario to isolate rear defense.
    if scenario in ('rear-fast','rear-contact'):
        blue = world.find("model[@name='urrc_pace_grid_09']")
        blue_config = next(c for c in pace_plugin.findall("car")
                           if c.findtext("name") == "urrc_pace_grid_09")
        route = destination / "team11_routes" / ("lane_1.csv" if scenario == "rear-fast" else f"lane_{initial_lane}.csv")
        with route.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.reader(handle))[1:]
        grid_s, _ = projected_obstacle(points, grid["x"], grid["y"])
        lap_distance = points[-1].s + math.hypot(
            points[-1].x - points[0].x, points[-1].y - points[0].y)
        start_s = (grid_s - 110. if scenario == "rear-fast" else grid_s + .40) % lap_distance
        first = next((i for i, row in enumerate(rows)
                      if float(row[0]) >= start_s), 0)
        rotated = rows[first:] + rows[:first]
        origin_s = float(rotated[0][0])
        staged_file = destination / ("team11_blue_20mps_behind.csv" if scenario == "rear-fast" else "team11_blue_20mps_contact.csv")
        with staged_file.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["distance_m", "x_m", "y_m", "z_m", "yaw_rad"])
            for row in rotated:
                relative = (float(row[0]) - origin_s) % lap_distance
                writer.writerow([f"{relative:.9f}", *row[1:5]])
        blue_config.find("route_file").text = str(staged_file.resolve())
        x, y, z, yaw = (float(v) for v in rotated[0][1:5])
        blue.find("pose").text = (
            f"{x:.8f} {y:.8f} {z+.003:.8f} 0 0 {yaw:.8f}")
    # Reject silently missing practice vehicles or overlapping TEAM11 spawns.
    expected = {f"urrc_pace_grid_{grid_position:02d}": grid_position}
    for slot in (2,9):
        if practice_cars and slot != grid_position and (scenario not in ("rear-fast","rear-contact") or slot == 9):
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
                           for slot in (2,9) if practice_cars and slot != grid_position
                           and (scenario not in ("rear-fast","rear-contact") or slot == 9))
    if sorted(names) != expected_pace:
        raise RuntimeError(f"Practice cars inconsistent with TEAM11 grid {grid_position}: {names}")
    grid_data = json.loads(
        (source / "src/urrc_track_gazebo/tracks/processed" / (track + ".json")).read_text()
    )["grid"]
    for name, number in expected.items():
        pose = [float(v) for v in models[name].findtext("pose").split()]
        original = grid_data[number-1]
        if (scenario == "slow-ahead" and name == "urrc_pace_grid_02") or (scenario in ("rear-fast","rear-contact") and name == "urrc_pace_grid_09"):
            continue
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
        "camera_ready_file": str(camera_ready_file.resolve()),
        "camera_target": [gx,gy,target_z],
        "camera_pitch_rad": camera_pitch, "camera_yaw_rad": camera_yaw,
        "camera_hfov_rad": camera_hfov, "camera_target_distance_m": ground_distance,
        "lamp_spacing_px": lamp_spacing_px,
        "grid_lane_policy": "physical side (original grids odd-right/even-left)",
        "grid_signed_lateral_m": grid_lateral,
        "grid_physical_side": "left" if grid_lateral > 0 else "right",
        "collision_mode": "gazebo_dynamic_contact" if physics_contact else "scripted_asymmetric_bounce",
        "physics_contact_mode": physics_contact,
        "nominal_ego_mass_kg": 50, "nominal_other_mass_kg": 1.5,
        "practice_cars": [name for name in sorted(names)],
        "practice_cars_enabled": practice_cars,
        "practice_scenario": scenario,
        "rear_lidar_topic": rear_topic,
        "rear_lidar_enabled": True,
        "scenario_target_speed_mps": 5 if scenario == "slow-ahead" else None,
        "scenario_target_initial_gap_m": 25 if scenario == "slow-ahead" else None,
        "grid_pose": [grid["x"], grid["y"], grid["road_z"], grid["yaw"]],
        "lane_spacing_m": spacing, "speed_mps": 15,
        "straight_speed_range_mps": [10, 15],
        "lane_change_length_m": 18,
        "steering_threshold_deg": 10, "corner_speed_range_mps": [5, 7],
        "gentle_corner_speed_range_mps": [10, 15],
        "corner_enter_curvature_per_m": 0.030,
        "corner_exit_curvature_per_m": 0.022,
        "sharp_corner_curvature_per_m": 0.080,
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
    parser.add_argument("--practice-cars", action="store_true", help="add red/blue opponent cars (off by default)")
    parser.add_argument("--scenario", choices=("normal","slow-ahead","rear-fast","rear-contact"), default="normal")
    parser.add_argument("--physics-contact", action="store_true")
    parser.add_argument("--grid", type=int, choices=range(1,13), default=1)
    args = parser.parse_args()
    root = Path(args.source).expanduser().resolve()
    out = Path(args.output).expanduser() if args.output else root/"generated/team11"/args.track
    result = prepare(root, out, args.track, args.plugin, args.pace_plugin, args.start_mode, args.lane, args.fixed_lane or not args.practice_cars, args.grid, args.practice_cars, args.scenario, args.physics_contact)
    print(json.dumps(result, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
