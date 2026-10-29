#!/usr/bin/env python3
"""공식 Monaco 그리드 시점/고저차로 5단계 AI 차량 실행."""
import argparse,os,subprocess,sys,signal,time
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
    p.add_argument("--lights",action="store_true",help="Monaco GUI 실행 후 실제 5등 순차 점등·동시 소등")
    p.add_argument("--dry-run",action="store_true")
    a=p.parse_args()
    cmd,env=build(a.ai,a.server)
    if a.dry_run:
        print("실행 명령:", " ".join(cmd))
        print("원본 Monaco 격리 리소스:",env["GZ_SIM_RESOURCE_PATH"])
    elif a.lights:
        if a.server:p.error("--lights는 GUI 모드에서만 사용합니다")
        sim=subprocess.Popen(cmd,env=env,start_new_session=True)
        try:
            # 단순 spawn은 신호등 서비스를 실행하지 않는다. 별도 프로세스로 점등.
            print("Monaco GUI 실행 중. 신호등 서비스와 카메라 준비를 확인합니다.",flush=True)
            command=[sys.executable,str(ROOT/"scripts/stage5_lights.py"),"sequence"]
            status=subprocess.run(command,env=env,cwd=ROOT,timeout=130)
            if status.returncode:
                print("신호등 실행 실패. Gazebo visual_config 및 카메라 서비스를 확인하세요.",file=sys.stderr)
                return status.returncode
            print("5개 순차 점등·일괄 소등 완료. Gazebo 창은 계속 표시됩니다.",flush=True)
            return sim.wait()
        except (KeyboardInterrupt,subprocess.TimeoutExpired):
            return 130
        finally:
            if sim.poll() is None:
                os.killpg(sim.pid,signal.SIGTERM)
                try:sim.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(sim.pid,signal.SIGKILL);sim.wait()
    else:raise SystemExit(subprocess.call(cmd,env=env))
if __name__=="__main__":raise SystemExit(main())
