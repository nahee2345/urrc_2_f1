"""Validate all 3 lanes against the source road polygon, not just width estimates."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from shapely.geometry import Point, Polygon
from core import load_centerline, generate_lanes, lane_spacing, CAR_WIDTH_M

def validate(source, track, margin=.12):
    base=Path(source)/"src/urrc_track_gazebo/tracks/processed"
    points=load_centerline(base/f"{track}_centerline.csv")
    meta=json.loads((base/f"{track}.json").read_text(encoding="utf-8"))
    poly=meta["road_polygon"]
    road=Polygon(poly["exterior"],[poly["interior"]])
    if not road.is_valid:
        raise ValueError("Invalid road polygon")
    # Vehicle half-width and minimum extra wall tolerance.
    inset=CAR_WIDTH_M/2+margin
    interior=road.buffer(-inset)
    if interior.is_empty:
        raise ValueError("No drivable polygon after wall margin")
    for index, lane in enumerate(generate_lanes(points)):
        missed=[]
        for i, p in enumerate(lane):
            if not interior.covers(Point(p[1],p[2])):
                missed.append(i)
        if missed:
            raise ValueError(f"{track} lane {index}: {len(missed)} points outside road safe corridor; first index {missed[0]}")
    print(f"PASS {track}: 3 routes, {len(points)} points/lane, spacing={lane_spacing(points):.3f}m")

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--source",required=True)
    args=p.parse_args()
    for track in ("monza","hungaroring"):
        validate(args.source,track)

if __name__=="__main__":
    main()
