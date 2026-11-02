#!/usr/bin/env python3
"""원본 v10.8의 5개 순차점등/동시소등 로직을 격리된 월드에서 사용.

원본 스크립트나 맵은 수정하지 않는다. 원본을 복사해 v10.8 준비를 마친
generated/stage3/resources/.../traffic_lights_v10.py를 로드한다.
"""
from __future__ import annotations
import argparse,importlib.util,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def load():
    p=ROOT/"generated/stage3/resources/urrc_track_gazebo/scripts/traffic_lights_v10.py"
    if not p.is_file():
        raise FileNotFoundError("3단계 준비 필요: python3 scripts/stage3_prepare_monza.py --ai AI-01")
    spec=importlib.util.spec_from_file_location("_official_v10_lights_isolated",p)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def main():
    ap=argparse.ArgumentParser(description="공식 v10.8: 5등 순차점등 후 한 번에 소등")
    ap.add_argument("state",choices=["on","off","sequence"])
    ap.add_argument("--seed",type=int)
    ap.add_argument("--skip-camera-check",action="store_true",help="렌더러 준비 진단용")
    a=ap.parse_args()
    os.environ.setdefault("GZ_PARTITION",f"urrc_stage3_{os.getuid()}")
    lights=load()
    try:
        if a.state=="on":
            lights.wait_ready("monza",check_cameras=not a.skip_camera_check)
            lights._create("monza")
            for i in range(1,6):lights._red("monza",i)
            print("빨간불 5개 모두 켜짐. 실제 렌즈 프레임 확인용")
        elif a.state=="off":
            lights._call("monza","remove","gz.msgs.Entity",
                         'name: "urrc_start_signal_active" type: 2',optional=True)
            print("5등 모델 단일 제거")
        else:
            lights.run("monza",seed=a.seed,check_cameras=not a.skip_camera_check)
        return 0
    except Exception as exc:
        print("원본 v10.8 신호등 실행 실패:",exc,file=sys.stderr)
        return 2

if __name__=="__main__":sys.exit(main())
