#!/usr/bin/env python3
"""Read-only structural TEAM08 audit, not a Gazebo driving certification."""
import ast
import csv
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
TEAM=ROOT/"src/team/team_08"
TRACKS=("monza","spa","monaco","cota","hungaroring","baku")

def main():
    errors=[]
    def check(condition,message):
        if not condition:errors.append(message)
    entry_path=TEAM/"race_entry.json"
    if not entry_path.is_file():
        print("FAIL race_entry.json missing");return 2
    entry=json.loads(entry_path.read_text(encoding="utf-8"))
    check(entry.get("entry_id")=="team_08","wrong entry_id")
    check(entry.get("ros_package")=="team08_race","wrong ROS package")
    check(entry.get("algorithm_launch_file")=="race.launch.py","missing launch entry")
    check((TEAM/"launch/race.launch.py").is_file(),"missing launch")
    check((TEAM/"package.xml").is_file(),"missing package.xml")
    check((TEAM/"models/team08_car/model.sdf").is_file(),"missing model.sdf")
    check((TEAM/"team08_race/control.py").is_file(),"missing controller")
    check((TEAM/"team08_race/strategy.py").is_file(),"missing race strategy")
    check((TEAM/"legacy/BattleReplay.cc").is_file(),"missing replay reference")
    for t in TRACKS:
        folder=TEAM/"maps"/t
        for name in ("centerline.csv","grid.json","track.json","lane_0.csv","lane_1.csv","lane_2.csv"):
            check((folder/name).is_file(),f"missing {t}/{name}")
        grid=folder/"grid.json"
        if grid.is_file():
            try:json.loads(grid.read_text(encoding="utf-8"))
            except ValueError:errors.append(f"invalid {t} grid JSON")
    for p in TEAM.rglob("*.py"):
        try:ast.parse(p.read_text(encoding="utf-8"),filename=str(p))
        except (SyntaxError,UnicodeError) as e:errors.append(f"Python parse: {p.relative_to(TEAM)}: {e}")
    for p in (TEAM/"package.xml",TEAM/"models/team08_car/model.sdf"):
        if p.is_file():
            try:ET.parse(p)
            except ET.ParseError as e:errors.append(f"XML parse: {p}: {e}")
    for p in TEAM.rglob("*"):
        if p.is_file() and p.suffix in (".py",".xml",".json",".sdf",".cfg"):
            try:text=p.read_text(encoding="utf-8")
            except UnicodeError:continue
            if any(term in text for term in ("team_11","team11","TEAM11","/team11/")):
                errors.append(f"legacy team identifier in {p.relative_to(TEAM)}")
    for problem in errors:print("FAIL",problem)
    print("TEAM08 STATIC", "PASS" if not errors else "FAIL",f"({len(errors)} issues)")
    print("Gazebo real-camera start, steering and lap completion: NOT TESTED")
    return 0 if not errors else 2

if __name__=="__main__":sys.exit(main())
