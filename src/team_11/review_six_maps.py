#!/usr/bin/env python3
"""Camera-free route verification across six official tracks and all grid slots.

This checks actual OFFICIAL centreline/grid files and the real strategy.Planner.
It is NOT a Gazebo physical drive, a completed lap, or proof of collision safety.
"""
import argparse
import csv
import json
import math
import sys
import tempfile
from pathlib import Path

from routes import TRACKS, WALL_CLEARANCE_M, generate, read_centerline, track_package
from strategy import Planner, Track, Vehicle
from team_11.race_context import configure

FIELDS = ("track", "grid", "path", "expected_grid_path", "result",
          "nearest_center_index", "spawn_to_center_m", "requested_path",
          "steer_deg", "straight_cap_ms", "corner_curvature",
          "corner_limit_ms", "braked_ms", "detail")


def save_paths(package, name, root):
    points=read_centerline(package/"tracks"/"processed"/(name+"_centerline.csv"))
    lanes, narrow=generate(points)
    dest=root/name
    dest.mkdir(parents=True,exist_ok=True)
    for lane, rows in lanes.items():
        with (dest/f"lane_{lane}.csv").open("w",newline="") as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    return Track(dest), narrow


def find_corner(track):
    # A real corner in the actual centreline rather than injected curvature.
    peak=max(range(track.n),key=lambda i:track.curvature[i])
    return peak, track.curvature[peak]


def verify_case(name, grid, lane, track, grid_pose):
    session=configure(name,grid)
    index=track.nearest(float(grid_pose["x"]),float(grid_pose["y"]))
    center=track.lanes[2][index]
    gap=math.hypot(center["x_m"]-float(grid_pose["x"]),
                   center["y_m"]-float(grid_pose["y"]))
    row=dict(track=name,grid=grid,path=lane,expected_grid_path=session.initial_lane,
             result="PASS",nearest_center_index=index,
             spawn_to_center_m=f"{gap:.3f}",requested_path="",
             steer_deg="",straight_cap_ms="",corner_curvature="",
             corner_limit_ms="",braked_ms="",detail="")
    failures=[]
    # The session's preferred initial path must match the actual grid-side
    # position. Grids near the same lateral centre cannot be distinguished.
    at_right=track.lanes[1][index]
    at_left=track.lanes[3][index]
    distance_to = lambda p: math.hypot(float(grid_pose["x"])-p["x_m"],
                                       float(grid_pose["y"])-p["y_m"])
    preferred=distance_to(at_right) if session.initial_lane==1 else distance_to(at_left)
    opposite=distance_to(at_left) if session.initial_lane==1 else distance_to(at_right)
    if preferred>opposite+0.15:
        failures.append("initial-grid-side-mismatch")
    if gap>max(4.0,center["road_width_m"]):
        failures.append("spawn-not-near-centerline")
    point=track.lanes[lane][index]
    ego=Vehicle(point["x_m"],point["y_m"],point["yaw_rad"],5.0,1.0,"ego")
    planner=Planner(track,spawn_lane=lane)
    # Test the commanded path at the grid location, before moving the model.
    cmd=planner.step(ego,[],1.0)
    row["requested_path"]=cmd.lane
    row["steer_deg"]=f"{math.degrees(cmd.steer):.2f}"
    if cmd.lane not in (1,2,3):
        failures.append("invalid-target")
    if cmd.mode=="STOP":
        failures.append("planner-stopped-"+cmd.reason.replace(" ","_"))
    if not 0<=cmd.speed<=15 or abs(cmd.steer)>math.radians(30)+1e-6:
        failures.append("control-limit")
    # A three-path test must respect actual permissible corridor geometry.
    if abs(point["lane_offset_m"])+.125+WALL_CLEARANCE_M>point["road_width_m"]/2+1e-5:
        failures.append("lane-outside-estimated-road")
    # The centreline's tightest bend provides an actual map-specific brake test.
    corner_idx,peak=find_corner(track)
    row["corner_curvature"]=f"{peak:.5f}"
    if peak<=1e-5:
        failures.append("no-corner-in-centerline")
    corner_point=track.lanes[2][corner_idx]
    corner_vehicle=Vehicle(corner_point["x_m"],corner_point["y_m"],
                           corner_point["yaw_rad"],15.0,2.0,"ego")
    corner_planner=Planner(track,spawn_lane=2)
    _,limit=corner_planner._corner_speed(corner_idx,15.0)
    row["corner_limit_ms"]=f"{limit:.3f}"
    if not math.isclose(limit,8.0,abs_tol=1e-6):
        failures.append("corner-target-not-8")
    # Start from cruising speed; output must slow rather than retain 15 m/s.
    corner_planner.last_speed=15.0
    corner_cmd=corner_planner.step(corner_vehicle,[],2.0)
    row["braked_ms"]=f"{corner_cmd.speed:.3f}"
    if limit < 15.0-1e-5 and not corner_cmd.speed<15.0-1e-5:
        failures.append("corner-does-not-brake")
    if abs(corner_cmd.steer)>math.radians(30)+1e-6:
        failures.append("corner-steering-limit")
    # Use _corner_speed on a zero-curvature track independent of path position.
    original=track.curvature
    try:
        track.curvature=[0.0]*track.n
        _,cap=planner._corner_speed(index,15.0)
        row["straight_cap_ms"]=f"{cap:.3f}"
        if not math.isclose(cap,15.0,abs_tol=1e-6):
            failures.append("straight-speed-cap-not-15")
    finally:
        track.curvature=original
    row["result"]="FAIL" if failures else "PASS"
    row["detail"]=";".join(failures) if failures else "offline_geometry_and_planner_only"
    return row


def run(package,output):
    output.parent.mkdir(parents=True,exist_ok=True)
    failures=0
    count=0
    with tempfile.TemporaryDirectory(prefix="team11_route_review_") as tmp:
        with output.open("w",newline="",encoding="utf-8") as stream:
            writer=csv.DictWriter(stream,fieldnames=FIELDS)
            writer.writeheader()
            for name in TRACKS:
                meta_path=package/"tracks"/"processed"/(name+".json")
                grid_data=json.loads(meta_path.read_text(encoding="utf-8"))
                slots={int(g["position"]):g for g in grid_data["grid"]}
                if set(range(1,12))-set(slots):
                    raise ValueError(f"{name}: official grid 1..11 incomplete")
                track,narrow=save_paths(package,name,Path(tmp))
                for grid in range(1,12):
                    for lane in (1,2,3):
                        result=verify_case(name,grid,lane,track,slots[grid])
                        writer.writerow(result)
                        count+=1
                        failures+=result["result"]=="FAIL"
                        if result["result"]=="FAIL":
                            print(f"FAIL {name} GRID {grid} PATH {lane}: {result['detail']}")
                print(f"{name}: 11 grids × 3 paths; samples={track.n}, narrow={narrow}",
                      flush=True)
    print(f"TOTAL: {count} cases, PASS {count-failures}, FAIL {failures}")
    print(f"CSV: {output}")
    print("OFFLINE ONLY: no physical vehicle motion, no actual lap or overtake proof")
    return 1 if failures else 0


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--official",default=str(Path.home()/"urrc_f1_ws"))
    p.add_argument("--output",default=str(Path.home()/"urrc_2_f1"/"log"/"team11_six_maps_routes.csv"))
    args=p.parse_args()
    try:
        return run(track_package(args.official),Path(args.output))
    except (OSError,ValueError,KeyError,TypeError) as exc:
        print("ERROR: cannot validate official data: "+str(exc),file=sys.stderr)
        return 2

if __name__=="__main__":
    sys.exit(main())
