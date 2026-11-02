#!/usr/bin/env python3
"""Generate and statically validate dynamic Gazebo Harmonic SDF vehicle."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from vehicle_generator.model import Vehicle,generate,manifest


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--config",type=Path,default=ROOT/"config/stage1_rwd.json")
    ap.add_argument("--output",type=Path,default=ROOT/"generated/urrc_stage1_rwd/model.sdf")
    args=ap.parse_args()
    v=Vehicle.from_file(args.config)
    model=generate(v)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(model,encoding="utf-8")
    info=manifest(v)
    args.output.with_name("manifest.json").write_text(json.dumps(info,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(info,ensure_ascii=False,indent=2))
    print(f"WROTE {args.output}")

if __name__=="__main__":main()
