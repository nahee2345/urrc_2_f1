#!/usr/bin/env python3
"""Spawns on an existing official Gazebo world. Never changes world assets."""
import argparse,json,os,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
TRACKS=("monza","spa","monaco","cota","hungaroring","baku")
def official(package=None):
    options=[Path(package).expanduser()] if package else []
    env=os.environ.get("URRC_OFFICIAL_WS")
    if env:options.append(Path(env).expanduser())
    options.extend([Path.home()/"urrc_f1_ws",Path.home()/"urrc_2"])
    for folder in options:
        for p in (folder,folder/"src"/"urrc_track_gazebo"):
            if (p/"tracks"/"processed").is_dir() and (p/"worlds").is_dir():
                return p
    raise FileNotFoundError("Official source not found. Use --official ~/urrc_f1_ws")
def make_command(pkg,track,grid):
    if track not in TRACKS or not 1<=grid<=11:raise ValueError("track/grid")
    meta=json.loads((pkg/"tracks"/"processed"/(track+".json")).read_text())
    pos=next(p for p in meta["grid"] if p["position"]==grid)
    model=Path(__file__).resolve().parents[1]/"models"/"team11_yellow_car"/"model.sdf"
    if os.environ.get("TEAM11_PRACTICE_LIGHTWEIGHT")=="1":
        # Stage7-only lightweight copy: no zoom camera or GPU LiDAR.\n        # Official submitted model is untouched.
        folder=Path(os.environ.get("TEAM11_PRACTICE_DIR",str(Path.home()/".cache"/"team11_practice")))
        folder.mkdir(parents=True,exist_ok=True)
        tree=ET.parse(model)
        link=tree.getroot().find("model/link[@name='base_link']")
        if link is None:raise ValueError("base_link missing")
        for sensor in list(link.findall("sensor")):
            if sensor.get("name") in ("team11_start_zoom_camera","team11_lidar"):
                link.remove(sensor)
        model=folder/"team11_lightweight.sdf"
        tree.write(model,encoding="unicode",xml_declaration=True)
    return ["ros2","run","ros_gz_sim","create","--world",track,
        "--file",str(model),"--name","team_11",
        "--x",str(pos["x"]),"--y",str(pos["y"]),
        "--z",str(pos["road_z"]+.09),"-Y",str(pos["yaw"])]
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--track",choices=TRACKS,default="monza")
    p.add_argument("--grid",type=int,default=1)
    p.add_argument("--official")
    p.add_argument("--dry-run",action="store_true")
    a=p.parse_args()
    cmd=make_command(official(a.official),a.track,a.grid)
    if a.dry_run:print(" ".join(cmd))
    else:subprocess.run(cmd,check=True,timeout=60)
if __name__=="__main__":main()
