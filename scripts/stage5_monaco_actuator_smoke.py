#!/usr/bin/env python3
"""공식 모나코 복사 월드의 AI-01 실제 전륜구동·검정 SDF 검사.

Gazebo JointController의 실제 FWD 물리 전진만 검사한다.
ROS2 카메라 추종이나 5등 출발 인터록·3바퀴 완주 검증은 아니다.
"""
from __future__ import annotations
import argparse,json,math,os,re,shutil,subprocess,sys,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.stage1_smoke import model_pose_stream


def pose_samples(raw,model):
    """Gazebo PosePublisher의 모델 및 link 이름 모두 수용."""
    poses=model_pose_stream(raw,model)
    if len(poses)>1:return poses
    # Link-only pose publisher를 쓰는 경우
    return model_pose_stream(raw,model+"::base_link")


def command(env,topic,number):
    return subprocess.run(["gz","topic","-t",topic,"-m","gz.msgs.Double",
                           "-p",f"data: {number:.7g}"],
                          env=env,text=True,capture_output=True,timeout=12,check=True)


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--startup-seconds",type=float,default=20.)
    p.add_argument("--motion-seconds",type=float,default=8.)
    p.add_argument("--report",type=Path,default=ROOT/"generated/stage5/ai01_physics_result.json")
    args=p.parse_args()
    vehicle="urrc_ai_01"
    world=ROOT/"generated/stage5/AI-01/monaco_experiment.sdf"
    models=ROOT/"generated/stage5/resources/urrc_track_gazebo/models"
    if not world.is_file() or not models.is_dir():raise SystemExit("5단계 Monaco 월드 생성 필요")
    if shutil.which("gz") is None:raise SystemExit("Gazebo Harmonic 미설치")
    env=dict(os.environ)
    env["GZ_PARTITION"]=f"urrc_stage5_actuator_{os.getpid()}"
    env["GZ_SIM_RESOURCE_PATH"]=str(models)+":"+env.get("GZ_SIM_RESOURCE_PATH","")
    env["SDF_PATH"]=str(models)+":"+env.get("SDF_PATH","")
    env.setdefault("LIBGL_ALWAYS_SOFTWARE","1")
    env.setdefault("QT_QPA_PLATFORM","offscreen")
    result={"test":"stage5_real_monaco_fwd_actuator",
            "vehicle":vehicle,"status":"NOT_RUN",
            "limitations":"카메라 주행 AI/ROS 브리지/5등 출발/3랩은 이 시험에서 검증하지 않음"}
    server=observer=None
    with tempfile.TemporaryDirectory(prefix="urrc_stage4_") as d:
        temp=Path(d);log=temp/"gazebo.log";poses=temp/"poses.log"
        try:
            with log.open("wb") as output:
                server=subprocess.Popen(["gz","sim","-s","-r","--headless-rendering",
                                         "-v","3",str(world)],
                                         stdout=output,stderr=subprocess.STDOUT,env=env)
                time.sleep(args.startup_seconds)
                if server.poll() is not None:
                    result.update(status="FAIL_SERVER",reason="Gazebo 실행 종료")
                    return 1
                topic=f"/model/{vehicle}/urrc_timing"
                advertised=subprocess.run(["gz","topic","-l"],env=env,text=True,
                                           capture_output=True,timeout=15).stdout
                result["pose_topic_available"]=topic in advertised.splitlines()
                if not result["pose_topic_available"]:
                    result.update(status="FAIL_POSE_TOPIC",reason=topic+" 미발행")
                    return 1
                with poses.open("wb") as out:
                    observer=subprocess.Popen(["gz","topic","-e","-t",topic],
                                              stdout=out,stderr=subprocess.STDOUT,env=env)
                    time.sleep(2)
                    before=pose_samples(poses.read_text(errors="replace"),vehicle)
                    for side in ("left","right"):
                        command(env,f"/model/{vehicle}/steer/{side}",0.)
                    for wheel in ("front_left","front_right"):
                        command(env,f"/model/{vehicle}/drive/{wheel}",6.)
                    time.sleep(args.motion_seconds)
                    for wheel in ("front_left","front_right"):
                        command(env,f"/model/{vehicle}/drive/{wheel}",0.)
                    time.sleep(1.5)
                    after=pose_samples(poses.read_text(errors="replace"),vehicle)
                    result["pose_count"]=len(after)
                    result["before_count"]=len(before)
                    if len(before)<2 or len(after)<len(before)+2:
                        result.update(status="FAIL_TELEMETRY",reason="주행 전후 모델 포즈 샘플 부족")
                        return 1
                    delta=math.dist(before[-1][:2],after[-1][:2])
                    result["displacement_xy_m"]=round(delta,6)
                    result["initial_z_m"]=before[-1][2]
                    result["final_z_m"]=after[-1][2]
                    if not .05<delta<15:
                        result.update(status="FAIL_PHYSICS",reason=f"전륜 실제 변위 비정상: {delta:.3f}m")
                        return 1
                    if not math.isfinite(result["final_z_m"]) or abs(result["final_z_m"]-result["initial_z_m"])>.8:
                        result.update(status="FAIL_TERRAIN_CONTACT",reason="차량 고저차/접촉에서 비정상 높이")
                        return 1
                    result.update(status="PASS_MONACO_FWD_ACTUATORS",
                                  reason="실제 모나코 3D 노면에서 AI-01 전륜 조인트 명령으로 이동 확인")
                    return 0
        except (OSError,subprocess.SubprocessError) as exc:
            result.update(status="ERROR",reason=str(exc))
            return 1
        finally:
            for proc in (observer,server):
                if proc is not None and proc.poll() is None:
                    proc.terminate()
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait()
            if log.exists():result["gazebo_log_tail"]=log.read_text(errors="replace")[-2000:]
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
            print(json.dumps({k:v for k,v in result.items() if k!="gazebo_log_tail"},
                             ensure_ascii=False,indent=2))


if __name__=="__main__":raise SystemExit(main())
