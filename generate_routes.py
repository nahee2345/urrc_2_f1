#!/usr/bin/env python3
"""Generate 18 deterministic fixed routes from the six imported official centerlines.

1 = right, 2 = centre, 3 = left, looking along track direction.
Uses no autonomous road detection, overtaking, or lane changing.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

TRACKS=("monza","spa","monaco","cota","hungaroring","baku")
FIELDS=("index","s_m","x_m","y_m","z_m","yaw_rad","curvature_1pm","lane")
REPO=Path(__file__).resolve().parent
SOURCE=REPO/"src"/"urrc_track_gazebo"/"tracks"/"processed"
OUTPUT=REPO/"routes"
CAR_HALF_WIDTH=.125
SAFETY_MARGIN=.15
DESIRED_OFFSET=.55

def points_for(track):
    source=SOURCE/f"{track}_centerline.csv"
    if not source.is_file():
        raise FileNotFoundError(f"{source} missing. Run python3 import_6_tracks.py")
    with source.open(newline="",encoding="utf-8") as stream:
        rows=[{k:float(v) for k,v in p.items()} for p in csv.DictReader(stream)]
    if len(rows)<100:
        raise ValueError(f"{track}: insufficient centerline points")
    if any(not math.isfinite(x) for row in rows for x in row.values()):
        raise ValueError(f"{track}: nonfinite centreline")
    for row in rows:
        if row["road_width_m"]/2-CAR_HALF_WIDTH-SAFETY_MARGIN<=0:
            raise ValueError(f"{track}: road too narrow for car")
    return rows,source

def build(track):
    points,source=points_for(track)
    n=len(points)
    min_width=min(p["road_width_m"] for p in points)
    offset=min(DESIRED_OFFSET,min_width/2-CAR_HALF_WIDTH-SAFETY_MARGIN)
    if offset<.15:
        raise ValueError(f"{track}: lane offset below safe 0.15m")
    step=[]
    for i,p in enumerate(points):
        q=points[(i+1)%n]
        ds=math.hypot(q["x_m"]-p["x_m"],q["y_m"]-p["y_m"])
        if ds<1e-5:raise ValueError(f"{track}: duplicate neighbours {i}")
        step.append(ds)
    s=[0.]
    for ds in step[:-1]:s.append(s[-1]+ds)
    length=s[-1]+step[-1]
    output=OUTPUT/track
    output.mkdir(parents=True,exist_ok=True)
    counts={}
    for lane,shift in ((1,-offset),(2,0.0),(3,offset)):
        xyz=[(p["x_m"]-shift*math.sin(p["yaw_rad"]),
              p["y_m"]+shift*math.cos(p["yaw_rad"]),
              p["z_m"]) for p in points]
        data=[]
        for i,((x,y,z),p) in enumerate(zip(xyz,points)):
            prev=xyz[(i-1)%n]; nxt=xyz[(i+1)%n]
            h0=math.atan2(y-prev[1],x-prev[0])
            h1=math.atan2(nxt[1]-y,nxt[0]-x)
            d=math.hypot(x-prev[0],y-prev[1])+math.hypot(nxt[0]-x,nxt[1]-y)
            curvature=math.atan2(math.sin(h1-h0),math.cos(h1-h0))/max(.001,.5*d)
            data.append((i,round(s[i],6),round(x,8),round(y,8),round(z,8),
                         round(p["yaw_rad"],8),round(curvature,7),lane))
        dest=output/f"lane_{lane}.csv"
        with dest.open("w",newline="",encoding="utf-8") as stream:
            writer=csv.writer(stream)
            writer.writerow(FIELDS)
            writer.writerows(data)
        counts[lane]=len(data)
    metadata={"track":track,"route_order":{"1":"right","2":"center","3":"left"},
              "fixed_lane_only":True,"points_per_lane":counts,
              "offset_m":round(offset,4),"length_m":round(length,4),
              "official_centerline_sha256":hashlib.sha256(source.read_bytes()).hexdigest()}
    (output/"manifest.json").write_text(
        json.dumps(metadata,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    return metadata

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--track",choices=(*TRACKS,"all"),default="all")
    args=p.parse_args()
    names=TRACKS if args.track=="all" else (args.track,)
    for name in names:
        data=build(name)
        print(f"ROUTES PASS {name}: 3 x {data['points_per_lane'][1]} samples,"
              f" spacing={data['offset_m']}m",flush=True)
    print("ROUTES TOTAL",len(names)*3,"CSV files",flush=True)

if __name__=="__main__":main()
