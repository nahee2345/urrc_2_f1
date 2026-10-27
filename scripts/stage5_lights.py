#!/usr/bin/env python3
"""Monaco 공식 v10.8 신호등을 격리된 테스트 월드에서 실행."""
import argparse,importlib.util,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser()
    p.add_argument("state",choices=("on","off","sequence"))
    p.add_argument("--seed",type=int)
    p.add_argument("--skip-camera-check",action="store_true")
    a=p.parse_args()
    os.environ.setdefault("GZ_PARTITION",f"urrc_stage5_{os.getuid()}")
    source=ROOT/"generated/stage5/resources/urrc_track_gazebo/scripts/traffic_lights_v10.py"
    if not source.is_file():
        raise FileNotFoundError("Monaco 월드 준비가 필요합니다: scripts/stage5_prepare_monaco.py")
    spec=importlib.util.spec_from_file_location("_urrc_read_only_monaco_lights",source)
    lights=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lights)
    try:
        if a.state=="on":
            lights.wait_ready("monaco",check_cameras=not a.skip_camera_check)
            lights._create("monaco")
            for i in range(1,6):lights._red("monaco",i)
            print("Monaco 5개 빨간불 전체 점등")
        elif a.state=="off":
            lights._off("monaco")
            print("Monaco 5개 빨간불 동시 소등")
        else:
            lights.run("monaco",seed=a.seed,check_cameras=not a.skip_camera_check)
        return 0
    except Exception as e:
        print("Monaco 신호등 서비스 오류:",e,file=sys.stderr)
        return 1
if __name__=="__main__":raise SystemExit(main())
