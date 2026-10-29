#!/usr/bin/env python3
"""5단계 AI-01 Monaco RGB 레이스 실행기.

시뮬레이션 감독용 월드 좌표를 AI에 전달하지 않는다.
신호등 카메라 실제 교정 없이는 실행을 거부하며, 충돌 데이터 누락을
3랩 PASS로 처리하지 않는다. 원본 GitHub에는 쓰기를 수행하지 않는다.
"""
from __future__ import annotations
import argparse,json,os,signal,shutil,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
# scripts/에서 직접 실행해도 stage3/.. 패키지를 정상 참조하도록 고정.
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))

def build_commands(calibration,speed=0.55,output=None):
    calibration=Path(calibration)
    from stage3.ros_gate import load_calibration
    load_calibration(calibration)
    if not .15<=speed<=.85:raise ValueError("Monaco 초기 속도 상한: 0.15~0.85 m/s")
    p=ROOT/"generated/stage5/AI-01"
    if not (p/"monaco_experiment.sdf").is_file() or not (p/"bridge.yaml").is_file():
        raise FileNotFoundError("먼저 scripts/stage5_prepare_monaco.py 및 scripts/stage5_bridge.py 실행")
    out=Path(output or p/"lap_events.jsonl")
    return [
        ("monaco_gazebo",[sys.executable,str(ROOT/"scripts/stage5_run_monaco.py"),"--ai","AI-01"]),
        ("ros_bridge",["ros2","run","ros_gz_bridge","parameter_bridge",
            "--ros-args","-p",f"config_file:={p/'bridge.yaml'}"]),
        ("camera_light_gate",[sys.executable,"-m","stage3.ros_gate",
            "--vehicle","urrc_ai_01","--entry-id","urrc_ai_01",
            "--calibration",str(calibration),
            "--permission-topic","/urrc_2_f1/operator_test_start",
            "--ros-args","-p","use_sim_time:=true"]),
        ("rgb_controller",[sys.executable,"-m","stage4.ros_drive",
            "--vehicle","urrc_ai_01","--speed-limit",str(speed),
            "--ros-args","-p","use_sim_time:=true"]),
        ("race_supervisor",[sys.executable,"-m","stage3.pose_monitor",
            "--vehicle","urrc_ai_01","--track","monaco",
            "--contacts-manifest",str(p/"contacts.json"),"--target-laps","3",
            "--output",str(out),"--ros-args","-p","use_sim_time:=true"]),
        ("isolated_operator_permission",["ros2","topic","pub","-r","10",
            "/urrc_2_f1/operator_test_start","std_msgs/msg/Bool","{data: true}"]),
    ]

def parse_verdict(path):
    path=Path(path)
    if not path.is_file():return {"status":"NOT_PASSED","reason":"운영자 랩 기록 미생성"}
    rows=[]
    for line in path.read_text().splitlines():
        try:rows.append(json.loads(line))
        except json.JSONDecodeError:continue
    final=next((row for row in reversed(rows)
                if row.get("type") in ("final","result")),None)
    if final is None:return {"status":"NOT_PASSED","reason":"결과 JSON 누락"}
    # 결과는 확인된 3랩과 실제 모든 충돌 센서 연결만 PASS. 임의 override 없음.
    valid=(final.get("status")=="PASS_THREE_LAPS"
           and final.get("laps")==3
           and final.get("collisions")==0
           and final.get("rejected_crossings")==0
           and len(final.get("lap_times_seconds",[]))==3
           and not final.get("invalid_reasons"))
    return {"status":"PASS_THREE_LAPS" if valid else "NOT_PASSED",
            "laps":final.get("laps",0),"lap_times_seconds":final.get("lap_times_seconds",[]),
            "collisions":final.get("collisions"),"rejected_crossings":final.get("rejected_crossings"),
            "invalid_reasons":final.get("invalid_reasons",[])}

def main():
    p=argparse.ArgumentParser(description="Monaco 3랩 실측: 카메라 출발 인터록 필수")
    p.add_argument("--calibration",type=Path,default=ROOT/"generated/stage5/AI-01/calibration.json")
    p.add_argument("--speed-limit",type=float,default=.55)
    p.add_argument("--minutes",type=float,default=85)
    p.add_argument("--dry-run",action="store_true")
    a=p.parse_args()
    if not 1<=a.minutes<=180:p.error("1~180분 범위 지정")
    base=ROOT/"generated/stage5/AI-01"
    race_log=base/"lap_events.jsonl"
    try:commands=build_commands(a.calibration,a.speed_limit,race_log)
    except (ValueError,FileNotFoundError) as exc:
        p.error(str(exc))
    if a.dry_run:
        for name,cmd in commands:print(name+": "+" ".join(cmd))
        return 0
    if not shutil.which("ros2") or not shutil.which("gz"):
        raise SystemExit("ROS 2 Jazzy·Gazebo Harmonic 설치 및 ROS 환경 로드 필요")
    env=os.environ.copy()
    env.setdefault("GZ_PARTITION",f"urrc_stage5_{os.getuid()}")
    env["PYTHONPATH"]=str(ROOT)+os.pathsep+env.get("PYTHONPATH","")
    if race_log.exists():race_log.unlink()
    processes=[]
    began=time.monotonic()
    base.mkdir(parents=True,exist_ok=True)
    try:
        for name,cmd in commands:
            f=(base/f"{name}.log").open("wb")
            proc=subprocess.Popen(cmd,cwd=ROOT,env=env,
                        stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append((name,proc,f))
            time.sleep(3 if name=="monaco_gazebo" else 1)
            if proc.poll() is not None:
                raise RuntimeError(f"{name} 시작 실패: {base/(name+'.log')}")
        # 원본 Monaco 신호등 실행은 카메라/visual_config 모두 준비될 때까지 자체 대기.
        light=subprocess.run([sys.executable,str(ROOT/"scripts/stage5_lights.py"),
              "sequence"],env=env,cwd=ROOT,capture_output=True,text=True,timeout=125)
        (base/"lights.log").write_text(light.stdout+"\n"+light.stderr)
        if light.returncode:
            raise RuntimeError("공식 신호등 점등/소등 실패: "+light.stderr[-300:])
        deadline=began+a.minutes*60
        while time.monotonic()<deadline:
            time.sleep(2)
            for name,proc,_ in processes:
                if proc.poll() is not None:raise RuntimeError(name+" 비정상 종료")
            verdict=parse_verdict(race_log)
            if verdict["status"]=="PASS_THREE_LAPS":
                print(json.dumps(verdict,ensure_ascii=False,indent=2))
                return 0
        print("운영자 기록 기준 3랩 미확인: 시간 만료")
        return 1
    except (RuntimeError,subprocess.TimeoutExpired) as exc:
        print("안전 종료:",exc,file=sys.stderr)
        return 2
    finally:
        for _,proc,_ in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGTERM)
        for _,proc,f in reversed(processes):
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            f.close()
        result=parse_verdict(race_log)
        (base/"stage5_verdict.json").write_text(json.dumps(
            {"track":"monaco","ai":"AI-01","result":result,
             "source":"real ROS2/Gazebo judge logs; not synthetic"},
             ensure_ascii=False,indent=2)+"\n")
        print("실험 판정:",json.dumps(result,ensure_ascii=False))

if __name__=="__main__":raise SystemExit(main())
