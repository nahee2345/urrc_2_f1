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
    data=json.loads(calibration.read_text(encoding="utf-8"))
    if data.get("track")!="monaco" or data.get("calibration_method")!="live_rgb_off_on_difference":
        raise ValueError("Monaco 레이스는 실제 소등/점등 차분으로 교정된 5등 설정만 허용합니다")
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
            "--status-file",str(p/"drive_status.json"),
            "--image-file",str(p/"rgb_camera.ppm"),
            "--ros-args","-p","use_sim_time:=true"]),
        ("race_supervisor",[sys.executable,"-m","stage3.pose_monitor",
            "--vehicle","urrc_ai_01","--track","monaco",
            "--contacts-manifest",str(p/"contacts.json"),"--target-laps","3",
            "--output",str(out),
            "--status-file",str(p/"pose_status.json"),
            "--ros-args","-p","use_sim_time:=true"]),
        ("isolated_operator_permission",["ros2","topic","pub","-r","10",
            "/urrc_2_f1/operator_test_start","std_msgs/msg/Bool","{data: true}"]),
    ]

def select_commands(commands,reuse_gazebo=False):
    """외부 GUI/브리지가 있으면 이 둘을 다시 띄우지 않는다."""
    if not reuse_gazebo:return list(commands)
    return [(name,cmd) for name,cmd in commands
            if name not in ("monaco_gazebo","ros_bridge")]


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

def read_diagnostics(path):
    """실제 ROS 카메라 제어/운영자 포즈의 마지막 기록만 읽는다."""
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError,ValueError,TypeError):
        return None


def diagnostic_line(drive,pose):
    if drive is None:
        d="주행 컨트롤러 진단 미수신 (rgb_controller.log 확인)"
    else:
        d=(f"카메라={drive.get('camera_frames',0)}프레임, "
           f"출발허가={drive.get('start_allowed',False)}, "
           f"도로검출={drive.get('lane_valid',False)}, "
           f"전륜목표={drive.get('front_left_radps',0)}rad/s, "
           f"차단/상태={drive.get('stop_reason','unknown')}")
    if pose is None:
        p="물리 위치 계측 미수신 (race_supervisor.log 확인)"
    else:
        p=(f"실제 변위={pose.get('displacement_from_start_xy_m',0)}m, "
           f"누적 주행={pose.get('distance_measured_xy_m',0)}m, "
           f"유효 랩={pose.get('laps',0)}")
    return "[AI-01 실제 주행 진단] "+d+" | "+p


def stop_motors(env):
    """ROS 종료 뒤에도 남을 수 있는 Gazebo 모터 명령을 직접 0으로 전환."""
    errors=[]
    for topic in (
        "/model/urrc_ai_01/drive/front_left",
        "/model/urrc_ai_01/drive/front_right",
        "/model/urrc_ai_01/steer/left",
        "/model/urrc_ai_01/steer/right",
    ):
        try:
            p=subprocess.run(["gz","topic","-t",topic,"-m","gz.msgs.Double","-p","data: 0"],
                             cwd=ROOT,env=env,timeout=4,capture_output=True,text=True)
            if p.returncode:
                errors.append(topic)
        except (OSError,subprocess.TimeoutExpired):
            errors.append(topic)
    return errors


def main():
    p=argparse.ArgumentParser(description="Monaco 3랩 실측: 카메라 출발 인터록 필수")
    p.add_argument("--calibration",type=Path,default=ROOT/"generated/stage5/AI-01/calibration.json")
    p.add_argument("--speed-limit",type=float,default=.55)
    p.add_argument("--minutes",type=float,default=85)
    p.add_argument("--dry-run",action="store_true")
    p.add_argument("--reuse-gazebo",action="store_true",help="기존 Monaco GUI/ROS 브리지 유지하고 카메라·주행 노드만 기동")
    a=p.parse_args()
    if not 1<=a.minutes<=180:p.error("1~180분 범위 지정")
    base=ROOT/"generated/stage5/AI-01"
    race_log=base/"lap_events.jsonl"
    try:commands=build_commands(a.calibration,a.speed_limit,race_log)
    except (ValueError,FileNotFoundError) as exc:
        p.error(str(exc))
    commands=select_commands(commands,a.reuse_gazebo)
    if a.dry_run:
        for name,cmd in commands:print(name+": "+" ".join(cmd))
        return 0
    if not shutil.which("ros2") or not shutil.which("gz"):
        raise SystemExit("ROS 2 Jazzy·Gazebo Harmonic 설치 및 ROS 환경 로드 필요")
    env=os.environ.copy()
    env.setdefault("GZ_PARTITION",f"urrc_stage5_{os.getuid()}")
    env["PYTHONPATH"]=str(ROOT)+os.pathsep+env.get("PYTHONPATH","")
    if race_log.exists():race_log.unlink()
    # 이전 세션 상태를 현재 증거로 오인하지 않는다.
    for old in ("drive_status.json","pose_status.json","rgb_camera.ppm"):
        candidate=base/old
        if candidate.exists():candidate.unlink()
    processes=[]
    began=time.monotonic()
    base.mkdir(parents=True,exist_ok=True)
    if a.reuse_gazebo:
        try:
            service=subprocess.run(["gz","service","-l"],cwd=ROOT,env=env,
                                   capture_output=True,text=True,timeout=8)
        except (OSError,subprocess.TimeoutExpired) as exc:
            print("[레이스 시작 불가] Monaco GUI 확인 실패:",exc,file=sys.stderr,flush=True)
            return 2
        if service.returncode!=0 or "/world/monaco/visual_config" not in service.stdout.splitlines():
            print("[레이스 시작 불가] 기존 Monaco GUI 서비스가 없습니다. "
                  "calibration_gazebo.log 확인",file=sys.stderr,flush=True)
            return 2
        print("[레이스] 교정이 끝난 Monaco GUI 및 ROS 브리지를 그대로 사용합니다.",flush=True)
    try:
        for name,cmd in commands:
            f=(base/f"{name}.log").open("wb")
            proc=subprocess.Popen(cmd,cwd=ROOT,env=env,
                        stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append((name,proc,f))
            print(f"[레이스] {name} 시작 / 로그: {base/(name+'.log')}",flush=True)
            time.sleep(3 if name=="monaco_gazebo" else 1)
            if proc.poll() is not None:
                raise RuntimeError(f"{name} 시작 실패: {base/(name+'.log')}")
        # 원본 Monaco 신호등 실행은 카메라/visual_config 모두 준비될 때까지 자체 대기.
        light=subprocess.run([sys.executable,str(ROOT/"scripts/stage5_lights.py"),
              "sequence"],env=env,cwd=ROOT,capture_output=True,text=True,timeout=125)
        (base/"lights.log").write_text(light.stdout+"\n"+light.stderr)
        if light.stdout:
            print("[레이스 신호등] "+light.stdout.strip()[-850:],flush=True)
        if light.returncode:
            raise RuntimeError("공식 신호등 점등/소등 실패: "+light.stderr[-300:])
        deadline=began+a.minutes*60
        next_status=0.
        print("[AI-01] 신호등 소등 후 실제 카메라/바퀴/물리 변위를 모니터링합니다.",flush=True)
        while time.monotonic()<deadline:
            time.sleep(1)
            for name,proc,_ in processes:
                if proc.poll() is not None:raise RuntimeError(name+" 비정상 종료")
            now=time.monotonic()
            if now>=next_status:
                next_status=now+3
                print(diagnostic_line(
                    read_diagnostics(base/"drive_status.json"),
                    read_diagnostics(base/"pose_status.json")),flush=True)
            verdict=parse_verdict(race_log)
            if verdict["status"]=="PASS_THREE_LAPS":
                print(json.dumps(verdict,ensure_ascii=False,indent=2))
                return 0
        print("운영자 기록 기준 3랩 미확인: 시간 만료")
        return 1
    except KeyboardInterrupt:
        print("\n[레이스 안전 종료] Ctrl+C 감지. 모든 주행 프로세스를 종료합니다.",flush=True)
        return 130
    except (RuntimeError,subprocess.TimeoutExpired) as exc:
        print("[레이스 실패]",exc,file=sys.stderr,flush=True)
        for name,_,_ in processes:
            log=base/(name+".log")
            if log.exists():
                tail=log.read_text(errors="replace")[-850:]
                if tail.strip():
                    print(f"[{name} 로그] {tail}",file=sys.stderr,flush=True)
        return 2
    finally:
        # 카메라/허가 스트림이 사라지는 종료 순간에도 Gazebo 모터를 반드시 0으로 보낸다.
        errors=stop_motors(env)
        if errors:
            print("[종료 경고] Gazebo 0속도 송신 실패 토픽: "+", ".join(errors),
                  file=sys.stderr,flush=True)
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
