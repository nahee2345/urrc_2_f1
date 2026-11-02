#!/usr/bin/env python3
"""Build per-AI SDF and metadata, including physical sensor housings."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from vehicle_generator.model import Vehicle
from vehicle_generator.sensors import build,load_presets


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--ai",default="all",help="AI-01 .. AI-10, or all")
    p.add_argument("--preset",type=Path,default=ROOT/"config/stage2_sensor_presets.json")
    p.add_argument("--vehicle-config",type=Path,default=ROOT/"config/stage1_rwd.json")
    p.add_argument("--outdir",type=Path,default=ROOT/"generated/stage2")
    args=p.parse_args()
    presets=load_presets(args.preset)
    if args.ai!="all" and args.ai not in presets:
        p.error("choose AI-01 ... AI-10 or all")
    for ai in (list(presets) if args.ai=="all" else [args.ai]):
        c=presets[ai]
        vehicle=replace(Vehicle.from_file(args.vehicle_config),
                        name=f"urrc_{ai.lower().replace('-', '_')}",drive=c["drive"])
        model,report=build(vehicle,c["driving"])
        d=args.outdir/ai
        d.mkdir(parents=True,exist_ok=True)
        (d/"model.sdf").write_text(model,encoding="utf-8")
        (d/"manifest.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        print(f"BUILT {ai} {vehicle.drive} {report['sensor_count']} sensors, "
              f"mass={report['total_mass_kg']:.3f}kg, envelope={report['envelope_m']}")
    return 0


if __name__=="__main__":raise SystemExit(main())
