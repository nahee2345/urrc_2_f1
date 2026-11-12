#!/usr/bin/env python3
"""Generate an isolated Gazebo test road from TEAM11's bundled CSV geometry.

This is a reconstructed road surface, NOT the organizer's official mesh,
kerbs or timing system. Only src/team_11 is read. No source maps are edited.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

TEAM_ROOT = Path(__file__).resolve().parents[1]
TRACKS = ("monza", "spa", "monaco", "cota", "hungaroring", "baku")


def child(parent, tag, value=None, **attrs):
    e=ET.SubElement(parent,tag,attrs)
    if value is not None:
        e.text=str(value)
    return e


def string_pose(x,y,z,roll=0.,pitch=0.,yaw=0.):
    return " ".join(f"{v:.8f}" for v in (x,y,z,roll,pitch,yaw))


def box_geometry(parent, length, width, height):
    geom=child(parent,"geometry")
    child(child(geom,"box"),"size",f"{length:.5f} {width:.5f} {height:.5f}")


def build_world(track: str, grid: int, path: Path, road_stride: int = 8):
    if track not in TRACKS or not (1 <= grid <= 11) or road_stride < 2:
        raise ValueError("Only bundled six tracks and grids 1..11 are supported")
    bundle=TEAM_ROOT/"maps"/track
    meta=json.loads((bundle/"track.json").read_text(encoding="utf-8"))
    with (bundle/"centerline.csv").open(newline="",encoding="utf-8") as f:
        points=[{key:float(v) for key,v in row.items()} for row in csv.DictReader(f)]
    if len(points)<200:
        raise ValueError("Embedded centerline has too few samples")
    slot=meta["grid"][grid-1]
    if slot["position"]!=grid:
        raise ValueError("Grid metadata mismatch")

    sdf=ET.Element("sdf",version="1.9")
    world=child(sdf,"world",name=track)
    physics=child(world,"physics",name="team11_csv_test",type="ignored")
    child(physics,"max_step_size","0.004")
    child(physics,"real_time_factor","1")
    child(world,"gravity","0 0 -9.81")
    scene=child(world,"scene")
    child(scene,"ambient","0.75 0.75 0.75 1")
    child(scene,"background","0.20 0.29 0.37 1")

    for filename,system in (
        ("gz-sim-physics-system","Physics"),
        ("gz-sim-user-commands-system","UserCommands"),
        ("gz-sim-scene-broadcaster-system","SceneBroadcaster"),
    ):
        child(world,"plugin",filename=filename,name="gz::sim::systems::"+system)
    sensor_system=child(world,"plugin",filename="gz-sim-sensors-system",
                        name="gz::sim::systems::Sensors")
    child(sensor_system,"render_engine","ogre2")

    sun=child(world,"light",type="directional",name="sun")
    child(sun,"pose","0 0 10 0 0 0")
    child(sun,"direction","-0.2 0.2 -1")
    child(sun,"diffuse","0.9 0.9 0.9 1")
    child(sun,"specular","0.1 0.1 0.1 1")

    road=child(world,"model",name="team11_csv_test_road")
    child(road,"static","true")
    # A single static link avoids hundreds of link entities and substantially
    # reduces Gazebo world load/GUI attach latency without changing the
    # physical road surface geometry.
    surface=child(road,"link",name="track_surface")
    indices=list(range(0,len(points),road_stride))
    if indices[-1]!=len(points)-1:
        indices.append(len(points)-1)
    for k,(i,j) in enumerate(zip(indices,indices[1:]+indices[:1])):
        a,b=points[i],points[j]
        dx=b["x_m"]-a["x_m"]
        dy=b["y_m"]-a["y_m"]
        dz=b["z_m"]-a["z_m"]
        distance=math.hypot(dx,dy)
        if distance<0.05:
            continue
        angle=math.atan2(dy,dx)
        slope=-math.atan2(dz,distance)
        patch_pose=string_pose(
            (a["x_m"]+b["x_m"])/2,(a["y_m"]+b["y_m"])/2,
            (a["z_m"]+b["z_m"])/2-.075,0.,slope,angle)
        width=max(.6,min(a["road_width_m"],b["road_width_m"]))
        for kind in ("collision","visual"):
            shape=child(surface,kind,name=f"patch_{k:04d}_{kind}")
            child(shape,"pose",patch_pose)
            box_geometry(shape,distance+.12,width,.15)
            if kind=="visual":
                material=child(shape,"material")
                child(material,"ambient","0.25 0.29 0.33 1")
                child(material,"diffuse","0.33 0.36 0.40 1")

    # The 5 red lamps are all in ONE model. One remove request switches
    # all lamps off atomically, matching the organizer's five-light policy.
    light_model=child(world,"model",name="team11_test_lights")
    child(light_model,"static","true")
    start=meta["start_finish"]
    child(light_model,"pose",string_pose(
        start["x"],start["y"],start["z"]+1.96,0.,0.,start["yaw"]))
    light_link=child(light_model,"link",name="five_lamps")
    for lamp in range(5):
        visual=child(light_link,"visual",name=f"red_lamp_{lamp+1}")
        child(visual,"pose",string_pose(0.,(lamp-2)*.16,0.))
        child(child(visual,"geometry"),"sphere")
        child(visual.find("geometry/sphere"),"radius",".07")
        material=child(visual,"material")
        child(material,"ambient","0.9 0.0 0.0 1")
        child(material,"diffuse","1 0.01 0.01 1")
        child(material,"emissive","1 0.02 0.02 1")

    model_file=TEAM_ROOT/"models/team11_car/model.sdf"
    own_car=ET.parse(model_file).getroot().find("model")
    if own_car is None or own_car.get("name")!="team_11":
        raise ValueError("TEAM11 car model name not found")
    child(own_car,"pose",string_pose(
        slot["x"],slot["y"],slot["road_z"]+.04,0.,0.,slot["yaw"]))
    world.append(own_car)

    path=Path(path).resolve()
    path.parent.mkdir(parents=True,exist_ok=True)
    ET.indent(sdf,space="  ")
    ET.ElementTree(sdf).write(path,encoding="utf-8",xml_declaration=True)
    return {"world":str(path),"track":track,"grid":grid,
            "road_patches":len(indices)-1,
            "spawn_xyz":[slot["x"],slot["y"],slot["road_z"]+.04],
            "start_light_model":"team11_test_lights",
            "note":"CSV-reconstructed road, not original organizer race world"}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--track",choices=TRACKS,default="monza")
    ap.add_argument("--grid",type=int,choices=range(1,12),default=1)
    ap.add_argument("--output",type=Path,required=True)
    args=ap.parse_args()
    print(json.dumps(build_world(args.track,args.grid,args.output),ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
