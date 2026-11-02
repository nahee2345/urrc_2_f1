#!/usr/bin/env python3
"""기준 Monza를 읽기 전용으로 사용하여 운영자 계측용 복사 월드를 생성한다."""
from __future__ import annotations
from dataclasses import replace
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from vehicle_generator.model import Vehicle
from vehicle_generator.sensors import build,load_presets
from stage3.instrument import instrument_model,make_monza_copy
from stage3.assets import prepare_monza_assets
from stage3.terrain import check_fidelity
from stage3.lap import gates_from_upstream


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--ai",default="AI-01")
    p.add_argument("--upstream",type=Path,default=ROOT/"upstream/urrc_f1_ws")
    p.add_argument("--outdir",type=Path,default=ROOT/"generated/stage3")
    args=p.parse_args()
    presets=load_presets()
    if args.ai not in presets:p.error("AI-01~AI-10 중 선택")
    v=replace(Vehicle.from_file(ROOT/"config/stage1_rwd.json"),
              name="urrc_"+args.ai.lower().replace("-","_"),
              drive=presets[args.ai]["drive"])
    model,meta=build(v,presets[args.ai]["driving"])
    prepared,topics=instrument_model(model,v.name)
    official,asset_lock=prepare_monza_assets(args.upstream,args.outdir)
    world=make_monza_copy(official,prepared,v.name)
    finish,cps=gates_from_upstream(args.upstream,"monza")
    d=args.outdir/args.ai;d.mkdir(parents=True,exist_ok=True)
    (d/"model_instrumented.sdf").write_text(prepared,encoding="utf-8")
    (d/"monza_experiment.sdf").write_text(world,encoding="utf-8")
    terrain=check_fidelity(args.upstream,official,d/"monza_experiment.sdf")
    (d/"terrain_fidelity.json").write_text(json.dumps(terrain,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    contact={"vehicle":v.name,"topics":topics}
    (d/"contacts.json").write_text(json.dumps(contact,ensure_ascii=False,indent=2)+"\n")
    (d/"gates_summary.json").write_text(json.dumps(
        {"track":"monza","finish":finish,"checkpoints":len(cps),
         "source":"upstream/urrc_f1_ws (읽기 전용)",
         "not_vehicle_driver_input":True,"enhanced_resources":str(official),
         "source_sha256":asset_lock["source_sha256"],
         "road_collision_source_identical":asset_lock["road_collision_source_identical"],
         "centerline_source_identical":asset_lock["centerline_source_identical"]},ensure_ascii=False,indent=2)+"\n")
    print(json.dumps({"ai":args.ai,"vehicle":v.name,"world":str(d/"monza_experiment.sdf"),
      "sensor_count":meta["sensor_count"],"contact_topics":len(topics),
      "enhanced_assets":str(official),"original_collision_preserved":True,
      "terrain_status":terrain["status"],"source_elevation_m":terrain["elevation_range_report_m"],
      "checkpoints":len(cps),"note":"원본 복사 생성 완료; 실제 몬자 주행 PASS 아님"},
      ensure_ascii=False,indent=2))

if __name__=="__main__":main()
