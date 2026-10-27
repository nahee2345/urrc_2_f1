#!/usr/bin/env python3
"""Stage 0: enforce immutable upstream commit and non-mutating compliance checks."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def contract_check(root):
    release=json.loads((root/"release.json").read_text(encoding="utf-8"))
    event=json.loads((root/"race/event_config.json").read_text(encoding="utf-8"))
    assert release["role"]=="competition"
    assert release["vehicle_max_m"]==[0.55,0.25,0.18]
    assert release["contract_version"]==2
    assert event["light_count"]==5 and event["light_interval_seconds"]==1.0
    assert event["qualifying_laps"]==2 and 5<=event["final_laps"]<=10
    assert event["checkpoint_spacing_m"]==10.0
    assert event["track_pool"]==["monza","spa","monaco","cota","hungaroring","baku"]
    for t in event["track_pool"]:
        assert (root/"src/urrc_track_gazebo/worlds"/f"{t}_race.sdf").is_file()
        assert (root/"src/urrc_track_gazebo/tracks/processed"/f"{t}.json").is_file()
    assert (root/"run_v10.sh").is_file()
    assert (root/"test_v10.sh").is_file()
    return release,event


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--root",type=Path,default=ROOT/"upstream/urrc_f1_ws")
    args=ap.parse_args()
    lock=json.loads((ROOT/"docs/source_lock.json").read_text(encoding="utf-8"))
    if not (args.root/".git").exists():
        print("MISSING: upstream submodule not cloned. Run git submodule update --init --recursive.",file=sys.stderr)
        return 2
    p=subprocess.run(["git","-C",str(args.root),"rev-parse","HEAD"],check=True,text=True,capture_output=True)
    sha=p.stdout.strip()
    if sha!=lock["urrc_f1_ws"]["commit"]:
        print(f"FAIL: upstream SHA {sha} differs from locked SHA {lock['urrc_f1_ws']['commit']}",file=sys.stderr)
        return 1
    try:
        release,event=contract_check(args.root)
    except (AssertionError,KeyError,FileNotFoundError,ValueError) as exc:
        print(f"FAIL: upstream contract check: {exc}",file=sys.stderr)
        return 1
    print(json.dumps({"stage":0,"result":"PASS_STATIC_ONLY","source_commit":sha,
          "track_count":len(event["track_pool"]),"vehicle_max_m":release["vehicle_max_m"],
          "camera_driving_policy":"experimental overlay, NOT official race contract"},ensure_ascii=False,indent=2))
    return 0

if __name__=="__main__":raise SystemExit(main())
