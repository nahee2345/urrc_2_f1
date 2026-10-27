#!/usr/bin/env python3
"""실제 원본 몬자 복사 월드에서 카메라·계측 토픽이 발행되는지만 검사.

신호등 점등/소등 인식 및 실주행·무충돌 3랩 시험은 별도 단계이다.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--ai",default="AI-01")
    p.add_argument("--verify-lights",action="store_true",help="원본 5등 순차점등 후 단일 모델 소등 검증")
    p.add_argument("--startup-seconds",type=float,default=20.)
    p.add_argument("--capture-seconds",type=float,default=8.)
    p.add_argument("--report",type=Path,default=ROOT/"generated/stage3/monza_smoke_result.json")
    a=p.parse_args()
    if not shutil.which("gz"):raise SystemExit("gz 미설치: 실측 보류")
    gen=ROOT/"generated/stage3"/a.ai
    world=gen/"monza_experiment.sdf"
    if not world.is_file():
        raise SystemExit("stage3_prepare_monza.py를 먼저 실행하세요")
    car="urrc_"+a.ai.lower().replace("-","_")
    result={"test":"stage3_original_monza_infra","ai":a.ai,
            "status":"NOT_RUN","source":"read-only upstream Monza world"}
    env=dict(os.environ)
    models=ROOT/"generated/stage3/resources/urrc_track_gazebo/models"
    if not (models/"urrc_monza/meshes/road_textured.obj").is_file():
        raise SystemExit("v10.8 향상된 에셋이 없음: stage3_prepare_monza.py 실행 필요")
    env["GZ_SIM_RESOURCE_PATH"]=f"{models}:{models.parent/'worlds'}:"+env.get("GZ_SIM_RESOURCE_PATH","")
    env["SDF_PATH"]=str(models)+":"+env.get("SDF_PATH","")
    env.setdefault("LIBGL_ALWAYS_SOFTWARE","1")
    env.setdefault("QT_QPA_PLATFORM","offscreen")
    server=None;observers=[]
    with tempfile.TemporaryDirectory(prefix="urrc_stage3_") as temp:
        log=Path(temp)/"sim.log"
        try:
            sd=subprocess.run(["gz","sdf","-p",str(world)],capture_output=True,text=True,timeout=45,env=env)
            if sd.returncode:
                result.update(status="FAIL_SDF",reason=sd.stderr[-2000:])
                return 1
            with log.open("wb") as simlog:
                server=subprocess.Popen(["gz","sim","-s","-r","--headless-rendering","-v","3",str(world)],
                    stdout=simlog,stderr=subprocess.STDOUT,env=env)
                time.sleep(a.startup_seconds)
                if server.poll() is not None:
                    result.update(status="FAIL_SERVER",reason="몬자 헤드리스 서버 종료")
                    return 1
                topics=subprocess.run(["gz","topic","-l"],capture_output=True,text=True,timeout=25,env=env).stdout
                result["camera_topic"]=f"/model/{car}/sensors/start_camera"
                result["pose_topic"]=f"/model/{car}/urrc_timing"
                result["camera_advertised"]=result["camera_topic"] in topics.splitlines()
                result["pose_advertised"]=result["pose_topic"] in topics.splitlines()
                for label,topic in (("camera",result["camera_topic"]),("pose",result["pose_topic"])):
                    file=Path(temp)/f"{label}.log"
                    out=file.open("wb")
                    proc=subprocess.Popen(["gz","topic","-e","-t",topic],
                        stdout=out,stderr=subprocess.STDOUT,env=env)
                    observers.append((proc,out,file))
                time.sleep(a.capture_seconds)
                for proc,out,_ in observers:
                    if proc.poll() is None:proc.terminate();proc.wait(timeout=5)
                    out.close()
            camera=observers[0][2].read_text(errors="replace")
            poses=observers[1][2].read_text(errors="replace")
            result["camera_bytes"]=observers[0][2].stat().st_size
            result["pose_bytes"]=observers[1][2].stat().st_size
            result["image_received"]=bool(re.search(r"\bwidth:\s*160\b",camera)
                       and re.search(r"\bheight:\s*120\b",camera) and "data:" in camera)
            result["pose_received"]=bool("pose {" in poses or 'name: "' in poses)
            if all(result[k] for k in ("camera_advertised","pose_advertised","image_received","pose_received")):
                if a.verify_lights:
                    light=subprocess.run(["python3",str(ROOT/"scripts/stage3_lights.py"),
                                          "sequence","--skip-camera-check"],
                                         capture_output=True,text=True,timeout=75,env=env)
                    result["five_light_command_exit"]=light.returncode
                    result["five_light_output"]=light.stdout[-2500:]
                    result["five_light_errors"]=light.stderr[-1000:]
                    result["five_light_sequence_applied"]=(light.returncode==0
                        and all(f"RED {n}/5" in light.stdout for n in range(1,6))
                        and "LIGHTS OUT" in light.stdout
                        and "model_remove=single_request" in light.stdout)
                    if not result["five_light_sequence_applied"]:
                        result.update(status="FAIL_LIGHT_SEQUENCE",reason="원본 5등 점등 및 동시 소등 서비스 검사 실패")
                        return 1
                result.update(status="PASS_MONZA_INFRA_AND_SIGNAL_COMMANDS" if a.verify_lights
                    else "PASS_MONZA_INFRA_ONLY",
                    reason="실제 카메라/포즈 및 원본 5등 점등/동시 소등 서비스 동작 확인; 카메라 RGB 분류·실주행 3랩 검증 아님")
                return 0
            result.update(status="FAIL_TOPIC",reason="실제 카메라/운영자 포즈 중 누락")
            return 1
        except (subprocess.TimeoutExpired,OSError) as exc:
            result.update(status="ERROR",reason=str(exc))
            return 1
        finally:
            for proc,out,_ in observers:
                if proc.poll() is None:
                    proc.terminate()
                    try:proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait()
                if not out.closed:out.close()
            if server is not None and server.poll() is None:
                server.terminate()
                try:server.wait(timeout=5)
                except subprocess.TimeoutExpired:server.kill();server.wait()
            if log.exists():result["gazebo_log_tail"]=log.read_text(errors="replace")[-2500:]
            a.report.parent.mkdir(parents=True,exist_ok=True)
            a.report.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
            print(json.dumps({k:v for k,v in result.items() if k!="gazebo_log_tail"},ensure_ascii=False,indent=2))

if __name__=="__main__":
    raise SystemExit(main())
