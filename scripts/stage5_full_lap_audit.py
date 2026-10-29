#!/usr/bin/env python3
"""0~5단계 Monaco '실제 Gazebo·ROS2·카메라 3랩' 강제 합격 게이트.

정적 unit PASS, 모의 camera, 단순 motor 3m, pose teleport, centerline 정답
등은 3바퀴 완료를 입증하지 못한다. 이 프로그램은 ROS+Gazebo 통합 실행을
직접 호출하고 그 실행의 실제 센서·체크포인트·접촉·시간 증거만 검사한다.
"""
from __future__ import annotations
import argparse,json,math,os
from pathlib import Path
import subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
DATA=ROOT/"generated/stage5/AI-01"
REPORT=ROOT/"generated/stage5/monaco_real_three_lap_audit.json"


def load_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError,ValueError,TypeError):
        return {}


def evaluate(exit_code):
    items=[]
    def require(condition,name,value=None):
        items.append({"check":name,"passed":bool(condition),"actual":value})
    original=load_json(DATA/"terrain_fidelity.json")
    require(original.get("status")=="PASS_MONACO_SOURCE_FIDELITY",
            "공식 Monaco 도로 지형·충돌 메시 무결성",original.get("status"))
    require(original.get("elevation_reference_m",0)>8,
            "공식 Monaco 고저차 8m 이상",original.get("elevation_reference_m"))
    calibration=load_json(DATA/"calibration.json")
    require(calibration.get("calibrated") is True and
            calibration.get("calibration_method")=="live_rgb_off_on_difference" and
            len(calibration.get("rois",[]))==5,
            "실제 출발 카메라 OFF/ON 5등 교정",calibration.get("calibration_method"))
    verdict=load_json(DATA/"stage5_verdict.json").get("result",{})
    require(exit_code==0,"통합 3랩 프로세스 정상 종료",exit_code)
    require(verdict.get("status")=="PASS_THREE_LAPS","독립 운영자 3랩 PASS",verdict.get("status"))
    require(verdict.get("laps")==3,"실제 정방향 랩 3회",verdict.get("laps"))
    times=verdict.get("lap_times_seconds",[])
    require(len(times)==3 and all(isinstance(x,(float,int)) and
            math.isfinite(x) and x>0 for x in times),
            "각 3랩의 실제 시뮬레이션 경과시간",times)
    require(verdict.get("collisions")==0,"충돌 0건",verdict.get("collisions"))
    require(verdict.get("rejected_crossings")==0 and
            verdict.get("invalid_reasons")==[],"단축·역주행·계측 불연속 없음",
            verdict.get("invalid_reasons"))
    events=[]
    try:
        for line in (DATA/"lap_events.jsonl").read_text().splitlines():
            events.append(json.loads(line))
    except (ValueError,OSError):
        pass
    checkpoints=[e for e in events if e.get("type")=="checkpoint"]
    laps=[e for e in events if e.get("type")=="lap"]
    require(len(laps)==3,"실제 운영자 출발선 정방향 이벤트 3회",len(laps))
    require(len(checkpoints)>=3*60,"Monaco 체크포인트 3바퀴 통과 증거",len(checkpoints))
    pose=load_json(DATA/"pose_status.json")
    actual_distance=pose.get("distance_measured_xy_m")
    require(isinstance(actual_distance,(float,int)) and
            math.isfinite(actual_distance) and actual_distance>=1900,
            "실제 Gazebo 물리 차량 누적 주행거리 1900m 이상",actual_distance)
    require(pose.get("pose_samples",0)>1000,
            "실제 차량 Gazebo PosePublisher 1000개 이상",pose.get("pose_samples"))
    rgb=load_json(DATA/"drive_status.json")
    require(rgb.get("start_allowed") is True,
            "실제 start_camera 신호등 소등 후 출발 허가",rgb.get("start_allowed"))
    require(rgb.get("camera_frames",0)>1000,
            "실제 RGB 주행 카메라 1000프레임 이상",rgb.get("camera_frames"))
    for filename in ("rgb_camera.ppm","start_camera_lights_off.ppm"):
        f=DATA/filename
        require(f.is_file() and f.stat().st_size>5000,
                f"실제 카메라 프레임 파일 {filename}",
                f.stat().st_size if f.is_file() else None)
    return {"status":"PASS_REAL_MONACO_THREE_LAPS" if all(i["passed"] for i in items)
            else "NOT_PASSED","track":"monaco","vehicle":"AI-01",
            "checks":items,"source":"actual Gazebo 3D world + ROS2 camera and independent lap judge"}


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--minutes",type=int,default=180)
    p.add_argument("--speed-limit",type=float,default=.55)
    a=p.parse_args()
    if not 20<=a.minutes<=180:
        p.error("Monaco 3랩은 20~180분 범위에서 실측해야 합니다")
    if not .15<=a.speed_limit<=.85:
        p.error("안전 속도 범위 위반")
    DATA.mkdir(parents=True,exist_ok=True)
    REPORT.parent.mkdir(parents=True,exist_ok=True)
    for stale in ("stage5_verdict.json","lap_events.jsonl","pose_status.json",
                  "drive_status.json","rgb_camera.ppm","calibration.json",
                  "start_camera_lights_off.ppm"):
        # 워크플로 격리 머신 전용 새 실행; 이전 PASS 증거를 절대 재사용하지 않음.
        target=DATA/stale
        if target.exists():target.unlink()
    log=REPORT.parent/"monaco_full_three_lap_console.log"
    command=[sys.executable,str(ROOT/"scripts/stage5_start.py"),
             "--minutes",str(a.minutes),"--speed-limit",str(a.speed_limit),
             "--recalibrate"]
    with log.open("wb") as output:
        run=subprocess.run(command,cwd=ROOT,env=os.environ.copy(),
                           stdout=output,stderr=subprocess.STDOUT,
                           timeout=a.minutes*60+300)
    report=evaluate(run.returncode)
    REPORT.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    if report["status"]!="PASS_REAL_MONACO_THREE_LAPS":
        print("\n실제 Gazebo 레이스 로그 마지막:",file=sys.stderr)
        print(log.read_text(errors="replace")[-7500:],file=sys.stderr)
        return 1
    return 0


if __name__=="__main__":raise SystemExit(main())
