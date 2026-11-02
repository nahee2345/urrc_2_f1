#!/usr/bin/env python3
"""원본 v10.8과 동일한 GUI 설정으로 격리된 몬자 실험 월드 실행."""
from __future__ import annotations
import argparse,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def make_command(ai="AI-01",server=False):
    generated=ROOT/"generated/stage3"
    package=generated/"resources/urrc_track_gazebo"
    world=generated/ai/"monza_experiment.sdf"
    if not world.is_file() or not (package/"worlds/monza_signal_active.sdf").is_file():
        raise FileNotFoundError("3단계 에셋 미생성: python3 scripts/stage3_prepare_monza.py --ai "+ai)
    command=["gz","sim","-r","-v","3"]
    if server:command+=["-s","--headless-rendering"]
    else:command+=["--gui-config",str(package/"worlds/monza_grid.gui.config")]
    command.append(str(world))
    env=os.environ.copy()
    paths=[str(package/"models"),str(package/"worlds")]
    for key in ("GZ_SIM_RESOURCE_PATH","SDF_PATH"):
        env[key]=os.pathsep.join(paths+([env[key]] if env.get(key) else []))
    env.setdefault("GZ_PARTITION",f"urrc_stage3_{os.getuid()}")
    return command,env

def main():
    p=argparse.ArgumentParser(description="공식 몬자 시각 구성과 3단계 물리 차량 실행")
    p.add_argument("--ai",default="AI-01")
    p.add_argument("--server",action="store_true",help="GitHub Actions용 GUI 없는 실행")
    p.add_argument("--dry-run",action="store_true")
    a=p.parse_args()
    command,env=make_command(a.ai,a.server)
    if a.dry_run:
        print("명령:", " ".join(command))
        print("Gazebo 리소스:",env["GZ_SIM_RESOURCE_PATH"])
        print("GZ_PARTITION:",env["GZ_PARTITION"])
        return 0
    return subprocess.call(command,env=env)

if __name__=="__main__":sys.exit(main())
