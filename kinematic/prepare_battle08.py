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
from prepare08 import build as build_single, REPLAY_BASE_Z_OFFSET_M

def nearest(points,grid):
    return min(range(len(points)),key=lambda i:
        (points[i]["x"]-float(grid["x"]))**2+
        (points[i]["y"]-float(grid["y"]))**2)

def grid_lateral(point,grid):
    yaw=point["yaw"]
    return ((float(grid["x"])-point["x"])*(-math.sin(yaw))+
            (float(grid["y"])-point["y"])*math.cos(yaw))

def build(track,lib,output,grid_position=1,lane=None,defense=False,laps=1):
    if grid_position not in ((2,3) if laps==3 else (1,2)):raise ValueError("Grid must be 2 or 3 in race3; 1 or 2 otherwise")
    if laps not in (1,3):raise ValueError("Supported lap counts: 1 or 3")
    if lane is None:lane=3 if grid_position%2 else 1
    if lane not in (1,2,3):raise ValueError("Lane must be 1..3")
    result=build_single(track,lane,lib,output,10.0)
    meta=json.loads((PACKAGE/"tracks"/"processed"/f"{track}.json").read_text())
    grids={int(g["position"]):g for g in meta["grid"]}
    if grid_position not in grids or 1 not in grids or 10 not in grids:raise ValueError("Official grid 1, selected team grid or grid 10 missing")
    with (ROOT/"routes"/track/"lane_2.csv").open(newline="") as f:
        rows=list(csv.DictReader(f))
    pts=[dict(s=float(r["s_m"]),x=float(r["x_m"]),y=float(r["y_m"]),
              z=float(r["z_m"]),yaw=float(r["yaw_rad"])) for r in rows]
    n=len(pts)
    if n<100:raise ValueError("Insufficient centreline samples")
    team_i=nearest(pts,grids[grid_position])
    red_i=nearest(pts,grids[1])
    g10i=nearest(pts,grids[10])
    team_s=pts[team_i]["s"]
    red_s=pts[red_i]["s"]
    g10s=pts[g10i]["s"]
    loop=float(json.loads((ROOT/"routes"/track/"manifest.json").read_text())["length_m"])
    # Race3: red car on the official grid 1; TEAM08 on grid 2 or 3.
    # Use the shortest signed centreline displacement, including start-line wrap.
    if laps==3:
        red_ahead=(red_s-team_s+loop/2)%loop-loop/2
        if not 0.5<red_ahead<30.0:
            raise ValueError(f"Official grid 1 must be 0.5..30m ahead of team grid {grid_position}: {red_ahead:.3f}m")
        ahead=team_s+red_ahead
    else:
        ahead=team_s+12.0
    # Grid 10 must remain behind TEAM08 in unwrapped coordinates.
    rear_behind=(g10s-team_s+loop/2)%loop-loop/2
    if not defense and rear_behind>=-0.5:
        raise ValueError(f"Official grid 10 must be behind team grid {grid_position}: {rear_behind:.3f}m")
    g10s=team_s+rear_behind
    if defense:
        # Dedicated rear-pressure check; preserve grid10 practice model in ordinary battles.
        g10s=team_s-7.0
    offset=float(json.loads((ROOT/"routes"/track/"manifest.json").read_text())["offset_m"])
    lead_mod=ahead%loop
    j=min(range(n),key=lambda i:abs(pts[i]["s"]-lead_mod))
    lead=pts[j]
    red_lat=grid_lateral(pts[red_i],grids[1]) if laps==3 else -offset
    if laps==3:
        lx=float(grids[1]["x"])
        ly=float(grids[1]["y"])
        lead_z=float(grids[1]["road_z"])
        lead_yaw=float(grids[1]["yaw"])
    else:
        lx=lead["x"]+offset*math.sin(lead["yaw"])
        ly=lead["y"]-offset*math.cos(lead["yaw"])
        lead_z=lead["z"]
        lead_yaw=lead["yaw"]
    center_file=output/"battle_centerline.csv"
    with (PACKAGE/"tracks"/"processed"/f"{track}_centerline.csv").open(newline="") as f:
        road_points=list(csv.DictReader(f))
    if len(road_points)!=len(pts):
        raise ValueError("Road-width samples and centerline sample counts differ")
    with center_file.open("w",newline="") as f:
        writer=csv.writer(f)
        writer.writerow(("distance_m","x_m","y_m","z_m","yaw_rad","road_width_m"))
        for p,road in zip(pts,road_points):
            if abs(p["x"]-float(road["x_m"]))>.1 or abs(p["y"]-float(road["y_m"]))>.1:
                raise ValueError("Road width profile is not aligned with the centerline")
            width=float(road["road_width_m"])
            if not math.isfinite(width) or width<=0:
                raise ValueError("Invalid road width profile")
            writer.writerow([*[format(p[k],".9f") for k in ("s","x","y","z","yaw")],format(width,".9f")])
    world=ET.parse(result["world_file"])
    root=world.getroot().find("world")
    for p in list(root.findall("plugin")):
        if p.get("name")=="urrc::FixedRouteReplay":root.remove(p)
    plug=ET.SubElement(root,"plugin",name="urrc::BattleReplay_08",filename=str(Path(lib).resolve()))
    values={
      "start_topic":"/team08/battle_start_08",
      "centerline_file":str(center_file.resolve()),
      "log_file":str((output/"battle_state.csv").resolve()),
      "lap_length_m":loop,
      "race_laps":laps,
      "track_name":track,
      "lane_offset_m":offset,
      "team_initial_lane":lane,
      "team_grid":grid_position,
      "team_start_yaw":float(grids[grid_position]["yaw"]),
      "team_preferred_offset_m":{1:-offset,2:0.0,3:offset}[lane],
      "team_start_s":team_s,
      "team_start_offset_m":grid_lateral(pts[team_i],grids[grid_position]),
      "lead_start_s":ahead,
      "lead_start_offset_m":red_lat,
      "rear_start_s":g10s,
      "rear_cruise_mps":14.0 if defense else 11.0,
      "rear_start_offset_m":(grid_lateral(pts[team_i],grids[grid_position]) if defense else grid_lateral(pts[g10i],grids[10]))
    }
    for k,v in values.items():ET.SubElement(plug,k).text=str(v)
    world.write(result["world_file"],encoding="utf-8",xml_declaration=True)
    # Generate two distinctive colours without changing the TEAM08 original.
    visual=ET.parse(result["vehicle_file"])
    def colour(source,target,rgba):
        tree=ET.parse(source)
        for el in tree.getroot().iter("visual"):
            if el.get("name") in ("chassis_yellow","chassis_visual"):
                for attr in ("ambient","diffuse"):
                    node=el.find("material/"+attr)
                    if node is not None:node.text=rgba
        tree.write(target,encoding="utf-8",xml_declaration=True)
    lead_model=output/"team_01.sdf"
    rear_model=output/"team_02.sdf"
    colour(result["vehicle_file"],lead_model,"0.90 0.12 0.14 1")
    colour(result["vehicle_file"],rear_model,"0.10 0.30 0.98 1")
    if defense:
        from math import sin,cos
        rear_mod=g10s%loop
        ri=min(range(n),key=lambda i:abs(pts[i]["s"]-rear_mod))
        rp=pts[ri]
        rear_lat=values["rear_start_offset_m"]
        rear_spawn=[rp["x"]-rear_lat*sin(rp["yaw"]),
                    rp["y"]+rear_lat*cos(rp["yaw"]),
                    rp["z"]+REPLAY_BASE_Z_OFFSET_M,rp["yaw"]]
    result.update({
      "mode":"battle_3laps" if laps==3 else ("battle_defense" if defense else "battle"),
      "team_initial_lane":lane,
      "team_grid":grid_position,
      "red_grid":1 if laps==3 else None,
      "lead_model":str(lead_model),
      "rear_model":str(rear_model),
      "start_topic":values["start_topic"],
      "start_positions":{
        "team_08":[float(grids[grid_position]["x"]),float(grids[grid_position]["y"]),
                   float(grids[grid_position]["road_z"])+REPLAY_BASE_Z_OFFSET_M,float(grids[grid_position]["yaw"])],
        "team_01":[lx,ly,lead_z+REPLAY_BASE_Z_OFFSET_M,lead_yaw],
        "team_02":rear_spawn if defense else [float(grids[10]["x"]),float(grids[10]["y"]),
                           float(grids[10]["road_z"])+REPLAY_BASE_Z_OFFSET_M,float(grids[10]["yaw"])]
      },
      "scenario":{"team_max_mps":15,"team_cruise_target_mps":10,"team_sharp_corner_target_mps":9.0 if laps==3 else 7,
                  "laps":laps,"wall_check":"geometric road-width envelope, not Gazebo physics contact",
                  "s_vehicle_mps":9,
                  "rear_grid10_mps":14 if defense else 11,"s_lane_sequence":[1,2,3,2,1],
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
    p.add_argument("--grid",type=int,choices=(1,2,3),required=True)
    p.add_argument("--defense",action="store_true")
    p.add_argument("--laps",type=int,choices=(1,3),default=1)
    p.add_argument("--lane",type=int,choices=(1,2,3),default=None)
    p.add_argument("--plugin",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    build(a.track,a.plugin,a.output,a.grid,a.lane,a.defense,a.laps)
