#!/usr/bin/env python3
"""Prepare exactly one official-grid-start, fixed-lane kinematic replay.

Reuses the urrc_2 pace cars' cumulative-distance interpolated pose algorithm.
The original vehicle SDF and official world/track source files are unchanged.
"""
import argparse
import csv
import json
import math
import shutil
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

BASE=Path(__file__).resolve().parent.parent
PACKAGE=BASE/"src"/"urrc_track_gazebo"
CHASSIS=BASE/"src"/"team_11"/"models"/"team11_yellow_car"/"model.sdf"
SPEED=10.0
JOIN_METERS=8.0
# The TEAM11 wheel bottoms are 0.090m below base_link; urrc_2 pace-car
# wheel bottoms sit 0.003m over the road. Match that ground clearance,
# not the different vehicle model's link-origin height.
WHEEL_BOTTOM_BELOW_BASE_M=0.090
TARGET_GROUND_CLEARANCE_M=0.003
REPLAY_BASE_Z_OFFSET_M=WHEEL_BOTTOM_BELOW_BASE_M+TARGET_GROUND_CLEARANCE_M
WHEELBASE_M=.36
MAX_STEERING_DEG=30.0
MAX_CURVATURE=math.tan(math.radians(MAX_STEERING_DEG))/WHEELBASE_M

def heading(a,b):
    return math.atan2(b[1]-a[1],b[0]-a[0])

def turn_angle(a,b):
    return math.atan2(math.sin(a-b),math.cos(a-b))

def limit_check(path):
    """Validate geometric steering equivalent; pose replay has no wheel steering actuator."""
    maximum=0.0
    for a,b,c in zip(path,path[1:],path[2:]):
        d1=math.hypot(b[0]-a[0],b[1]-a[1])
        d2=math.hypot(c[0]-b[0],c[1]-b[1])
        if min(d1,d2)<1e-7:
            continue
        delta=abs(turn_angle(heading(b,c),heading(a,b)))
        k=2*math.sin(delta/2)/max(1e-7,(d1+d2)/2)
        maximum=max(maximum,k)
    angle=math.degrees(math.atan(WHEELBASE_M*maximum))
    if angle>MAX_STEERING_DEG+1e-3:
        raise ValueError(
            f"Selected replay route would need {angle:.2f}deg equivalent steering, "
            f"exceeding {MAX_STEERING_DEG:.0f}deg: smooth route or extend join")
    return angle

def smooth(t):
    t=max(0.0,min(1.0,t))
    return t*t*t*(10+t*(-15+6*t))

def make_route(track,lane,out):
    meta=json.loads((PACKAGE/"tracks"/"processed"/f"{track}.json").read_text())
    grid=next((g for g in meta["grid"] if int(g["position"])==1),None)
    if not grid:
        raise ValueError("official grid 1 missing")
    with (PACKAGE/"tracks"/"processed"/f"{track}_centerline.csv").open(newline="") as f:
        center=list(csv.DictReader(f))
    with (BASE/"routes"/track/f"lane_{lane}.csv").open(newline="") as f:
        route=list(csv.DictReader(f))
    if len(center)!=len(route) or len(route)<100:
        raise ValueError("route/centreline mismatch")
    gridx,gridy=float(grid["x"]),float(grid["y"])
    start=min(range(len(center)),key=lambda i:
              (float(center[i]["x_m"])-gridx)**2+
              (float(center[i]["y_m"])-gridy)**2)
    angle=float(center[start]["yaw_rad"])
    nx,ny=-math.sin(angle),math.cos(angle)
    grid_offset=(gridx-float(center[start]["x_m"]))*nx+(gridy-float(center[start]["y_m"]))*ny
    wanted_shift=(-1 if lane==1 else (1 if lane==3 else 0))*float(json.loads(
        (BASE/"routes"/track/"manifest.json").read_text())["offset_m"])
    # Initial point is the official grid centre; the next samples advance
    # along the selected lane. Only the initial 8m interpolates the start
    # offset into the lane offset. No lane changes during the subsequent lap.
    path=[(gridx,gridy,float(grid["road_z"]),float(grid["yaw"]))]
    traveled=0.0
    previous_center=(float(center[start]["x_m"]),float(center[start]["y_m"]))
    n=len(center)
    for step in range(n):
        i=(start+1+step)%n
        c=center[i]
        cx,cy=float(c["x_m"]),float(c["y_m"])
        traveled+=math.hypot(cx-previous_center[0],cy-previous_center[1])
        previous_center=(cx,cy)
        yaw=float(c["yaw_rad"])
        shift=grid_offset+(wanted_shift-grid_offset)*smooth(traveled/JOIN_METERS)
        x=cx-shift*math.sin(yaw)
        y=cy+shift*math.cos(yaw)
        path.append((x,y,float(c["z_m"]),yaw))
    if len(path)<100:raise ValueError("missing points")
    equivalent_steering=limit_check(path)
    out.mkdir(parents=True,exist_ok=True)
    distance=0.0
    filename=out/"route.csv"
    with filename.open("w",newline="") as f:
        writer=csv.writer(f)
        writer.writerow(("distance_m","x_m","y_m","z_m","yaw_rad"))
        writer.writerow((0.0,*path[0]))
        for a,b in zip(path,path[1:]):
            ds=math.dist(a[:3],b[:3])
            if ds<=1e-6:continue
            distance+=ds
            writer.writerow((round(distance,8),*[round(v,8) for v in b]))
    if distance<100:raise ValueError("invalid track lap length")
    return grid,filename.resolve(),distance,grid_offset,wanted_shift,equivalent_steering

def build(track,lane,library,output,speed=SPEED):
    if track not in ("monza","spa","monaco","cota","hungaroring","baku"):
        raise ValueError(track)
    if lane not in (1,2,3):raise ValueError(lane)
    grid,route,distance,offset,shift,equivalent_steering=make_route(track,lane,output)
    # A runtime model for kinematic replay. Keep the complete original
    # vehicle visual geometry, but disable the Ackermann dynamics plugin.
    # The original model.sdf is never modified.
    model=ET.parse(CHASSIS)
    root=model.getroot()
    car=root.find("model")
    car.find("static").text="true"
    for plugin in list(car.findall("plugin")):
        if "AckermannSteering" in plugin.attrib.get("name",""):
            car.remove(plugin)
    model_path=output/"replay_vehicle.sdf"
    model.write(model_path,encoding="utf-8",xml_declaration=True)
    runtime=output/"resources"/"urrc_track_gazebo"
    (runtime/"worlds").mkdir(parents=True,exist_ok=True)
    (runtime/"scripts").mkdir(parents=True,exist_ok=True)
    shutil.copy2(PACKAGE/"scripts"/"traffic_lights_v10.py",
                 runtime/"scripts"/"traffic_lights_v10.py")
    race=ET.parse(PACKAGE/"worlds"/f"{track}_race.sdf")
    world=race.getroot().find("world")
    if world is None:raise ValueError("race world missing")
    # Restore the original spectator cameras in the isolated race world.
    # The unmodified official 5-lamp script waits for these camera topics.
    sys.path.insert(0,str(PACKAGE))
    from urrc_tracks.v10_assets import _add_cameras
    with (PACKAGE/"tracks"/"processed"/f"{track}_centerline.csv").open(newline="") as f:
        samples=[(float(r["x_m"]),float(r["y_m"]),float(r["z_m"]))
                 for r in csv.DictReader(f)]
    official=json.loads((PACKAGE/"tracks"/"processed"/f"{track}.json").read_text())
    _add_cameras(world,track,official,samples)
    for p in list(world.findall("plugin")):
        if p.attrib.get("name")=="urrc::FixedRouteReplay":
            world.remove(p)
    plugin=ET.SubElement(world,"plugin",name="urrc::FixedRouteReplay",
                         filename=str(Path(library).resolve()))
    topic=f"/urrc/{track}/fixed/start"
    ET.SubElement(plugin,"start_topic").text=topic
    entry=ET.SubElement(plugin,"car")
    ET.SubElement(entry,"name").text="team_11"
    ET.SubElement(entry,"speed_mps").text=str(speed)
    ET.SubElement(entry,"z_offset_m").text=f"{REPLAY_BASE_Z_OFFSET_M:.6f}"
    ET.SubElement(entry,"route_file").text=str(route)
    world_path=runtime/"worlds"/f"{track}_fixed.sdf"
    race.write(world_path,encoding="utf-8",xml_declaration=True)
    sys.path.insert(0,str(BASE))
    from import_6_tracks import active_signal_from_race
    active_signal_from_race(world_path,runtime/"worlds"/f"{track}_signal_active.sdf")
    manifest={"track":track,"lane":lane,"lane_side":{1:"right",2:"centre",3:"left"}[lane],
              "grid":1,"grid_xyz_yaw":[float(grid["x"]),float(grid["y"]),
                    float(grid["road_z"])+REPLAY_BASE_Z_OFFSET_M,float(grid["yaw"])],
              "speed_mps":speed,"length_m":distance,"source_offset_m":offset,
              "wheel_ground_clearance_m":TARGET_GROUND_CLEARANCE_M,
              "vehicle_base_link_z_above_road_m":REPLAY_BASE_Z_OFFSET_M,
              "max_equivalent_steering_deg":round(equivalent_steering,3),
              "steering_limit_deg":MAX_STEERING_DEG,
              "target_offset_m":shift,"route_file":str(route),"world_file":str(world_path.resolve()),
              "vehicle_file":str(model_path.resolve()),"start_topic":topic,
              "lights_script":str((runtime/"scripts"/"traffic_lights_v10.py").resolve()),
              "method":"Gazebo SetWorldPoseCmd directly; no wheel dynamics"}
    (output/"manifest.json").write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+"\n")
    return manifest

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--track",required=True)
    p.add_argument("--lane",type=int,choices=(1,2,3),required=True)
    p.add_argument("--plugin",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--speed",type=float,default=SPEED)
    a=p.parse_args()
    if not .1<=a.speed<=20:raise ValueError("speed out of range .1..20m/s")
    print(json.dumps(build(a.track,a.lane,a.plugin,a.output,a.speed),indent=2,ensure_ascii=False))

if __name__=="__main__":main()
