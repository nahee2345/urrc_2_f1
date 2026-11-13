#!/usr/bin/env python3
"""Prepare isolated three-car Gazebo scenario using six official-copy tracks."""
import argparse
import csv
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parent.parent
PACKAGE=ROOT/"src"/"urrc_track_gazebo"
sys.path.insert(0,str(ROOT/"kinematic"))
from prepare import build as build_single, REPLAY_BASE_Z_OFFSET_M

def nearest(points,grid):
    return min(range(len(points)),key=lambda i:
        (points[i]["x"]-float(grid["x"]))**2+
        (points[i]["y"]-float(grid["y"]))**2)

def grid_lateral(point,grid):
    yaw=point["yaw"]
    return ((float(grid["x"])-point["x"])*(-math.sin(yaw))+
            (float(grid["y"])-point["y"])*math.cos(yaw))

def build(track,lib,output,grid_position=1,lane=None):
    if grid_position not in (1,2):raise ValueError("Grid must be 1 or 2")
    if lane is None:lane=3 if grid_position==1 else 1
    if lane not in (1,2,3):raise ValueError("Lane must be 1..3")
    result=build_single(track,lane,lib,output,10.0)
    meta=json.loads((PACKAGE/"tracks"/"processed"/f"{track}.json").read_text())
    grids={int(g["position"]):g for g in meta["grid"]}
    if grid_position not in grids or 10 not in grids:raise ValueError("Selected grid or grid 10 missing")
    with (ROOT/"routes"/track/"lane_2.csv").open(newline="") as f:
        rows=list(csv.DictReader(f))
    pts=[dict(s=float(r["s_m"]),x=float(r["x_m"]),y=float(r["y_m"]),
              z=float(r["z_m"]),yaw=float(r["yaw_rad"])) for r in rows]
    n=len(pts)
    if n<100:raise ValueError("Insufficient centreline samples")
    g1i=nearest(pts,grids[grid_position])
    g10i=nearest(pts,grids[10])
    g1s=pts[g1i]["s"]
    g10s=pts[g10i]["s"]
    loop=float(json.loads((ROOT/"routes"/track/"manifest.json").read_text())["length_m"])
    # Unwrapped positions: grid 10 must remain behind the selected start.
    if g10s>=g1s:g10s-=loop
    ahead=g1s+12.0
    offset=float(json.loads((ROOT/"routes"/track/"manifest.json").read_text())["offset_m"])
    lead_mod=ahead%loop
    j=min(range(n),key=lambda i:abs(pts[i]["s"]-lead_mod))
    lead=pts[j]
    lx=lead["x"]+offset*math.sin(lead["yaw"])
    ly=lead["y"]-offset*math.cos(lead["yaw"])
    center_file=output/"battle_centerline.csv"
    with center_file.open("w",newline="") as f:
        writer=csv.writer(f)
        writer.writerow(("distance_m","x_m","y_m","z_m","yaw_rad"))
        for p in pts:writer.writerow([format(p[k],".9f") for k in ("s","x","y","z","yaw")])
    world=ET.parse(result["world_file"])
    root=world.getroot().find("world")
    for p in list(root.findall("plugin")):
        if p.get("name")=="urrc::FixedRouteReplay":root.remove(p)
    plug=ET.SubElement(root,"plugin",name="urrc::BattleReplay",filename=str(Path(lib).resolve()))
    values={
      "start_topic":f"/urrc/{track}/battle/start",
      "centerline_file":str(center_file.resolve()),
      "log_file":str((output/"battle_state.csv").resolve()),
      "lap_length_m":loop,
      "lane_offset_m":offset,
      "team_initial_lane":lane,
      "team_grid":grid_position,
      "team_start_yaw":float(grids[grid_position]["yaw"]),
      "team_preferred_offset_m":{1:-offset,2:0.0,3:offset}[lane],
      "team_start_s":g1s,
      "team_start_offset_m":grid_lateral(pts[g1i],grids[grid_position]),
      "lead_start_s":ahead,
      "rear_start_s":g10s,
      "rear_start_offset_m":grid_lateral(pts[g10i],grids[10])
    }
    for k,v in values.items():ET.SubElement(plug,k).text=str(v)
    world.write(result["world_file"],encoding="utf-8",xml_declaration=True)
    # Generate two distinctive colours without changing the TEAM11 original.
    visual=ET.parse(result["vehicle_file"])
    def colour(source,target,rgba):
        tree=ET.parse(source)
        for el in tree.getroot().iter("visual"):
            if el.get("name")=="chassis_yellow":
                for attr in ("ambient","diffuse"):
                    node=el.find("material/"+attr)
                    if node is not None:node.text=rgba
        tree.write(target,encoding="utf-8",xml_declaration=True)
    lead_model=output/"practice_s_09.sdf"
    rear_model=output/"practice_rear_11.sdf"
    colour(result["vehicle_file"],lead_model,"0.90 0.12 0.14 1")
    colour(result["vehicle_file"],rear_model,"0.10 0.30 0.98 1")
    result.update({
      "mode":"battle",
      "team_initial_lane":lane,
      "team_grid":grid_position,
      "lead_model":str(lead_model),
      "rear_model":str(rear_model),
      "start_topic":values["start_topic"],
      "start_positions":{
        "team_11":[float(grids[grid_position]["x"]),float(grids[grid_position]["y"]),
                   float(grids[grid_position]["road_z"])+REPLAY_BASE_Z_OFFSET_M,float(grids[grid_position]["yaw"])],
        "practice_s_09":[lx,ly,lead["z"]+REPLAY_BASE_Z_OFFSET_M,lead["yaw"]],
        "practice_rear_11":[float(grids[10]["x"]),float(grids[10]["y"]),
                           float(grids[10]["road_z"])+REPLAY_BASE_Z_OFFSET_M,float(grids[10]["yaw"])]
      },
      "scenario":{"team_mps":10,"s_vehicle_mps":9,
                  "rear_grid10_mps":11,"s_lane_sequence":[1,2,3,2,1],
                  "s_segment_seconds":2,
                  "start_signal":"5 lamps off via Gazebo service, not camera",
                  "collision_detector":"oriented 0.53m x 0.25m rectangle SAT",
                  "rear_defense_ttc_s":5,"front_overtake_ttc_s":3}
    })
    (output/"battle_manifest.json").write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n")
    print(json.dumps(result,indent=2,ensure_ascii=False))
    return result

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--track",choices=("monza","spa","monaco","cota","hungaroring","baku"),required=True)
    p.add_argument("--grid",type=int,choices=(1,2),required=True)
    p.add_argument("--lane",type=int,choices=(1,2,3),default=None)
    p.add_argument("--plugin",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    build(a.track,a.plugin,a.output,a.grid,a.lane)
