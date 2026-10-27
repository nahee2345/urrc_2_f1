#!/usr/bin/env python3
"""공식 Monaco 그리드 시점/고저차로 5단계 AI 차량 실행."""
import argparse,os,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def build(ai,server=False):
    base=ROOT/"generated/stage5"
    pkg=base/"resources/urrc_track_gazebo"
    world=base/ai/"monaco_experiment.sdf"
    if not world.is_file() or not (pkg/"worlds/monaco_signal_active.sdf").is_file():
        raise FileNotFoundError("먼저 python3 scripts/stage5_prepare_monaco.py --ai "+ai)
    cmd=["gz","sim","-r","-v","3"]
    if server:cmd+=["-s","--headless-rendering"]
    else:cmd+=["--gui-config",str(pkg/"worlds/monaco_grid.gui.config")]
    cmd.append(str(world))
    env=os.environ.copy()
    for name in ("GZ_SIM_RESOURCE_PATH","SDF_PATH"):
        env[name]=os.pathsep.join([str(pkg/"models"),str(pkg/"worlds"),env.get(name,"")])
    env.setdefault("GZ_PARTITION",f"urrc_stage5_{os.getuid()}")
    return cmd,env
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--ai",default="AI-01")
    p.add_argument("--server",action="store_true")
    p.add_argument("--dry-run",action="store_true")
    a=p.parse_args()
    cmd,env=build(a.ai,a.server)
    if a.dry_run:
        print("실행 명령:", " ".join(cmd))
        print("원본 Monaco 격리 리소스:",env["GZ_SIM_RESOURCE_PATH"])
    else:raise SystemExit(subprocess.call(cmd,env=env))
if __name__=="__main__":main()
