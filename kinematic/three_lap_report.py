#!/usr/bin/env python3
"""Collect three-lap race results from case logs and the original Gazebo logs."""
import argparse
import csv
import re
from pathlib import Path
from diagnose_battle import classify, text_file

FIELDS = ("track", "grid", "lane", "result", "rank", "team_peak_mps",
          "team_average_mps", "min_wall_margin_m", "wall_boundary_status",
          "min_body_clearance_m", "failure_category", "exit_code",
          "wall_measurement", "case_log")

def extract(case_log, track, grid, lane, exit_code):
    case_log = Path(case_log)
    result = classify(case_log, exit_code)
    text = text_file(case_log)
    path_match = re.search(r"^Log:\s+(.+/prepared/battle_state\.csv)\s*$", text, re.M)
    if path_match:
        gazebo = Path(path_match.group(1)).parent.parent / "gazebo.log"
        text = text_file(gazebo) + "\n" + text
    records = re.findall(r"\[BATTLE\] RACE3_RESULT ([^\n]*)", text)
    record = records[-1] if records else ""
    def get(key):
        found = re.search(r"(?<!\w)" + re.escape(key) + r"=([^\s]+)", record)
        return found.group(1) if found else ""
    wall = bool(re.search(r"\[BATTLE\] WALL_BOUNDARY ", text))
    margin = get("wall_margin_min_m")
    return {
        "track": track,
        "grid": grid,
        "lane": lane,
        "result": result["replay_result"],
        "rank": get("rank") if result["replay_result"] == "PASS_3_LAPS" else "UNRANKED",
        "team_peak_mps": get("team_peak_mps"),
        "team_average_mps": get("team_average_mps"),
        "min_wall_margin_m": margin,
        "wall_boundary_status": "CROSSED" if wall else ("CLEAR_GEOMETRIC" if margin else "UNKNOWN"),
        "min_body_clearance_m": get("min_body_clearance_m"),
        "failure_category": result["failure_category"],
        "exit_code": exit_code,
        "wall_measurement": "GEOMETRIC_ONLY_NOT_PHYSICS_CONTACT",
        "case_log": str(case_log),
    }

def append(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        if path.stat().st_size == 0:
            writer.writeheader()
        writer.writerow(data)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--case", required=True, type=Path)
    p.add_argument("--track", required=True)
    p.add_argument("--grid", required=True)
    p.add_argument("--lane", required=True)
    p.add_argument("--exit-code", type=int, default=0)
    p.add_argument("--append", type=Path, required=True)
    args = p.parse_args()
    row = extract(args.case, args.track, args.grid, args.lane, args.exit_code)
    append(args.append, row)
    print("RACE3 "
          + " ".join(f"{key}={row[key]}" for key in (
              "track", "grid", "result", "rank", "team_peak_mps",
              "team_average_mps", "wall_boundary_status", "failure_category")))
    print("WALL_SENSOR=GEOMETRIC_ONLY_NOT_PHYSICS_CONTACT")

if __name__ == "__main__":
    main()
