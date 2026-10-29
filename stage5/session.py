"""5단계 Monaco: 한 Gazebo GUI/ROS 브리지에서 교정 후 레이스까지 유지.

교정 직후 gz sim / ros_gz_bridge를 강제 종료하고 재기동하던 문제 해결.
주행/신호등 안전 검증은 예전과 동일하게 유지. 모터는 교정 중 작동 안 함.
"""
from __future__ import annotations
from pathlib import Path
import os
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
GENERATED=ROOT/"generated/stage5/AI-01"
LIGHT_SCRIPT=ROOT/"scripts/stage5_lights.py"
CALIBRATOR=ROOT/"scripts/stage5_calibrate_lights.py"
WORLD=ROOT/"scripts/stage5_run_monaco.py"
RACE=ROOT/"scripts/stage5_race.py"


def close_process(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid,signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid,signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()


def log_tail(path,limit=1100):
    path=Path(path)
    try:
        return path.read_text(errors="replace")[-limit:]
    except OSError:
        return "로그 없음"


def launch_one(name,cmd,env,session):
    path=GENERATED/f"calibration_{name}.log"
    f=path.open("wb")
    try:
        p=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=f,
                           stderr=subprocess.STDOUT,start_new_session=True)
        session.append((name,p,f))
    except BaseException:
        f.close()
        raise
    print(f"[Gazebo 통합] {name} 시작 | 로그: {path}",flush=True)
    return p


def await_gazebo(gui,env,timeout=90):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if gui.poll() is not None:
            raise RuntimeError("Gazebo GUI 프로세스가 종료됐습니다:\n"+
                               log_tail(GENERATED/"calibration_gazebo.log"))
        try:
            found=subprocess.run(["gz","service","-l"],cwd=ROOT,env=env,
                                 text=True,capture_output=True,timeout=6)
            if found.returncode==0 and "/world/monaco/visual_config" in found.stdout.splitlines():
                return
        except (OSError,subprocess.TimeoutExpired):
            pass
        time.sleep(2)
    raise RuntimeError("Monaco GUI 서비스가 실행되지 않았습니다. 로그:\n"+
                       log_tail(GENERATED/"calibration_gazebo.log"))


def ensure_live(session,env):
    for name,proc,_ in session:
        if proc.poll() is not None:
            raise RuntimeError(name+" 실행이 중단됐습니다. 로그:\n"+
                               log_tail(GENERATED/f"calibration_{name}.log"))
    gui=next(p for name,p,_ in session if name=="gazebo")
    await_gazebo(gui,env,timeout=8)


def calibrate_on_existing(env,calibration,session,seconds=40):
    """실제 카메라 OFF/ON 렌즈 5개를 확인. 동일 GUI와 브리지 유지."""
    from scripts.stage5_start import calibrated
    lights_on=False
    baseline=GENERATED/"start_camera_lights_off.ppm"
    try:
        ensure_live(session,env)
        print("[교정] 1/3 빨간불 OFF 상태의 카메라 화면 수집",flush=True)
        subprocess.run([sys.executable,str(CALIBRATOR),"--capture-baseline",
                        "--output",str(baseline),"--seconds","20"],
                       cwd=ROOT,env=env,timeout=35,check=True)
        print("[교정] 2/3 교정용 빨간불 5개 전체 점등",flush=True)
        subprocess.run([sys.executable,str(LIGHT_SCRIPT),"on"],
                       cwd=ROOT,env=env,timeout=95,check=True)
        lights_on=True
        print("[교정] 3/3 빨간 렌즈 5개 실제 OFF/ON 영상 차분",flush=True)
        subprocess.run([sys.executable,str(CALIBRATOR),
                        "--baseline",str(baseline),"--output",str(calibration),
                        "--seconds",str(seconds)],cwd=ROOT,env=env,
                       timeout=seconds+20,check=True)
        if not calibrated(calibration):
            raise RuntimeError("Monaco OFF/ON 실제 교정 조건이 충족되지 않았습니다. 모터 구동 금지")
        print("[교정] 출발 카메라 5등 교정 성공",flush=True)
    finally:
        if lights_on:
            print("[교정] 빨간불 OFF 전환 (Gazebo GUI는 계속 유지)",flush=True)
            off=subprocess.run([sys.executable,str(LIGHT_SCRIPT),"off"],
                               cwd=ROOT,env=env,text=True,
                               capture_output=True,timeout=30)
            if off.returncode!=0:
                # 소등 실패면 차량 자동출발은 절대 실행되지 않도록 함.
                raise RuntimeError("교정용 신호등 소등 실패. 자동주행 차단: "+
                                   off.stderr[-300:])


def run_stage5(calibration,speed,minutes,recalibrate,calibrate_only,env):
    """단일 터미널: Gazebo를 한 번 열고 교정과 레이스가 같은 창을 공유한다."""
    from scripts.stage5_start import calibrated,run_race
    calibration=Path(calibration).resolve()
    GENERATED.mkdir(parents=True,exist_ok=True)
    session=[]
    try:
        gui=launch_one("gazebo",[sys.executable,str(WORLD),"--ai","AI-01"],env,session)
        await_gazebo(gui,env)
        bridge=launch_one("bridge",["ros2","run","ros_gz_bridge",
                 "parameter_bridge","--ros-args","-p",
                 f"config_file:={GENERATED/'bridge.yaml'}"],env,session)
        time.sleep(3)
        if bridge.poll() is not None:
            raise RuntimeError("ROS 브리지 조기 종료:\n"+
                               log_tail(GENERATED/"calibration_bridge.log"))
        if recalibrate or not calibrated(calibration):
            print("[5단계] 실제 카메라 교정 실행",flush=True)
            calibrate_on_existing(env,calibration,session)
        else:
            print("[5단계] 기존 Monaco 실제 OFF/ON 카메라 교정값 재사용",flush=True)
        if calibrate_only:
            print("[5단계] 교정 완료. 레이스는 시작하지 않았습니다.",flush=True)
            return 0
        if not calibrated(calibration):
            raise RuntimeError("교정 파일 무효: 레이스 시작 차단")
        ensure_live(session,env)
        print("[Gazebo] 교정한 Monaco 창 그대로 유지. 절대 재시작하지 않습니다.",flush=True)
        command=[sys.executable,str(RACE),"--calibration",str(calibration),
                 "--speed-limit",str(speed),"--minutes",str(minutes),
                 "--reuse-gazebo"]
        return run_race(command,env)
    finally:
        print("[5단계] 실험 종료: 이 실행기가 시작한 ROS 브리지·Gazebo 정리",flush=True)
        for _,proc,_ in reversed(session):
            close_process(proc)
        for _,_,handle in session:
            handle.close()
