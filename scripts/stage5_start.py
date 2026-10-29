#!/usr/bin/env python3
"""Monaco 5단계 단일 터미널 실행: 생성→교정→안전 출발→레이스.

실제 5개 빨간등 영상 교정 전에는 모터를 절대로 구동하지 않는다.
원본 URRC GitHub는 읽기 전용이며, 임시 프로세스는 종료 시 모두 정리한다.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from stage3.ros_gate import load_calibration

AI = "AI-01"
GENERATED = ROOT / "generated/stage5/AI-01"
CALIBRATION = GENERATED / "calibration.json"


def calibrated(path=CALIBRATION):
    """모의/미교정 설정은 허용하지 않는다."""
    path = Path(path)
    if not path.is_file():
        return False
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("track") != "monaco":
            return False
        load_calibration(path)
        return True
    except (OSError, ValueError, KeyError, TypeError):
        return False


def stages(calibration=CALIBRATION, speed=0.55, minutes=120, calibrate_only=False):
    """검사하기 쉬운 순수 명령 생성 함수. 실제 프로세스는 시작하지 않는다."""
    if not 0.15 <= speed <= 0.85:
        raise ValueError("Monaco 속도 범위 0.15~0.85 m/s")
    if not 1 <= minutes <= 180:
        raise ValueError("실험 시간은 1~180분")
    prep = [
        ("world_prepare", [sys.executable, str(ROOT / "scripts/stage5_prepare_monaco.py"), "--ai", AI]),
        ("bridge_prepare", [sys.executable, str(ROOT / "scripts/stage5_bridge.py")]),
    ]
    if calibrate_only:
        return prep
    return prep + [
        ("race", [sys.executable, str(ROOT / "scripts/stage5_race.py"),
                  "--calibration", str(Path(calibration)),
                  "--speed-limit", str(speed), "--minutes", str(minutes)])
    ]


def base_environment():
    env = os.environ.copy()
    # 단계별 Gazebo 파티션은 모든 자식 프로세스에서 정확히 동일해야 한다.
    env.setdefault("GZ_PARTITION", f"urrc_stage5_{os.getuid()}")
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    return env


def run_checked(name, cmd, env, timeout=None):
    print(f"[5단계] {name}", flush=True)
    return subprocess.run(cmd, cwd=ROOT, env=env, check=True, timeout=timeout)


def terminate_group(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()


def service_ready(env, simulator, timeout=90):
    end = time.monotonic() + timeout
    service = "/world/monaco/visual_config"
    while time.monotonic() < end:
        if simulator.poll() is not None:
            raise RuntimeError("Gazebo 조기 종료: " + str(GENERATED / "calibration_gazebo.log"))
        try:
            p = subprocess.run(["gz", "service", "-l"], env=env, cwd=ROOT,
                               capture_output=True, text=True, timeout=6)
            if p.returncode == 0 and service in p.stdout.splitlines():
                return
        except (OSError, subprocess.TimeoutExpired):
            pass
        time.sleep(2)
    raise RuntimeError(f"Gazebo 신호등 시각 서비스 연결 실패: {service}")


def capture_diagnostic(env):
    """교정 실패시 진단용 실제 카메라 사진을 남긴다. ROI 값은 만들지 않는다."""
    output = GENERATED / "start_camera_diagnostic.ppm"
    try:
        subprocess.run([sys.executable, str(ROOT / "scripts/stage3_capture_roi.py"),
                        "--topic", "/model/urrc_ai_01/sensors/start_camera",
                        "--out", str(output), "--timeout", "8"],
                       cwd=ROOT, env=env, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=13, check=True)
        print(f"[진단] 실제 출발 카메라 영상: {output}", flush=True)
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        print("[진단] 출발 카메라 영상도 수신하지 못했습니다.", flush=True)


def calibrate_in_one_terminal(env, path, seconds=40):
    """신호등 5등 ON 상태의 실제 ROS 카메라 영상만으로 안전하게 ROI를 생성."""
    gui = bridge = None
    log_handles = []
    try:
        # 이 준비 구간에는 운전자/모터 명령 퍼블리셔를 시작하지 않는다.
        simulator_cmd = [sys.executable, str(ROOT / "scripts/stage5_run_monaco.py"),
                         "--ai", AI]
        for name, cmd in (("gazebo", simulator_cmd),
                          ("bridge", ["ros2", "run", "ros_gz_bridge",
                                      "parameter_bridge", "--ros-args", "-p",
                                      f"config_file:={GENERATED / 'bridge.yaml'}"])):
            dest = GENERATED / f"calibration_{name}.log"
            handle = dest.open("wb")
            log_handles.append(handle)
            proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=handle,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            if name == "gazebo":
                gui = proc
                service_ready(env, gui)
            else:
                bridge = proc
                time.sleep(3)
                if bridge.poll() is not None:
                    raise RuntimeError("ROS2 브리지 시작 실패: " + str(dest))

        print("[5단계] 빨간불 5개 전체 점등 및 실제 카메라 자동 교정", flush=True)
        run_checked("원본 5등 점등", [sys.executable,
                    str(ROOT / "scripts/stage5_lights.py"), "on"], env, timeout=90)

        result = subprocess.run([
            sys.executable, str(ROOT / "scripts/stage5_calibrate_lights.py"),
            "--output", str(path), "--seconds", str(seconds)],
            cwd=ROOT, env=env, timeout=seconds + 20)
        if result.returncode != 0 or not calibrated(path):
            capture_diagnostic(env)
            raise RuntimeError(
                "실제 빨간불 5개 자동 교정 실패. 모터 구동 금지. "
                "calibration_gazebo.log / calibration_bridge.log 및 카메라 진단 이미지 확인")

        print("[5단계] 출발 카메라 5등 교정 PASS", flush=True)
        run_checked("신호등 교정용 점등 해제",
                    [sys.executable, str(ROOT / "scripts/stage5_lights.py"), "off"],
                    env, timeout=45)
    finally:
        terminate_group(bridge)
        terminate_group(gui)
        for handle in log_handles:
            handle.close()


def run_race(cmd, env):
    """같은 터미널 출력으로 레이스 실행. Ctrl+C 때 자식 프로세스 전체 종료."""
    print("[5단계] 카메라 출발 인터록·RGB 제어·운영자 3랩 감시 시작", flush=True)
    proc = subprocess.Popen(cmd, cwd=ROOT, env=env, start_new_session=True)
    try:
        return proc.wait()
    except KeyboardInterrupt:
        print("\n[안전 종료] Ctrl+C 감지, 레이스 프로세스 정리", flush=True)
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            return proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            terminate_group(proc)
            return 130
    finally:
        terminate_group(proc)


def main():
    p = argparse.ArgumentParser(
        description="Monaco 단일 터미널: 맵 생성→카메라 교정→신호등 출발→AI-01 주행")
    p.add_argument("--speed-limit", type=float, default=0.55)
    p.add_argument("--minutes", type=float, default=120)
    p.add_argument("--calibration", type=Path, default=CALIBRATION)
    p.add_argument("--recalibrate", action="store_true", help="기존 교정 파일이 있어도 실제 영상으로 재교정")
    p.add_argument("--calibrate-only", action="store_true", help="맵 준비와 신호등 교정만 한 뒤 종료")
    p.add_argument("--dry-run", action="store_true", help="프로세스 실행 없이 순서만 표시")
    args = p.parse_args()
    try:
        commands = stages(args.calibration, args.speed_limit, args.minutes,
                          args.calibrate_only)
    except ValueError as exc:
        p.error(str(exc))
    env = base_environment()
    if args.dry_run:
        for label, command in commands:
            print(label, " ".join(command))
        print("교정: 실제 Gazebo+브리지→5등 전체 점등→ROS 카메라 ROI→신호등 초기화")
        return 0
    if not shutil.which("gz") or not shutil.which("ros2"):
        p.error("ROS 2 Jazzy 및 Gazebo Harmonic 설치와 source /opt/ros/jazzy/setup.bash 확인")
    GENERATED.mkdir(parents=True, exist_ok=True)
    try:
        for name, cmd in commands[:2]:
            run_checked(name, cmd, env)
        if args.recalibrate or not calibrated(args.calibration):
            print("[5단계] 유효한 Monaco 출발 카메라 교정 파일 없음: 자동 교정 시작", flush=True)
            calibrate_in_one_terminal(env, args.calibration)
        else:
            print("[5단계] 검증 형식에 맞는 기존 Monaco 교정 설정 재사용", flush=True)
        if args.calibrate_only:
            print("[5단계] 교정 완료. 차량 자동주행은 실행하지 않았습니다.", flush=True)
            return 0
        if not calibrated(args.calibration):
            raise RuntimeError("출발 카메라 ROI 검증 실패: 주행 시작 차단")
        return run_race(commands[-1][1], env)
    except KeyboardInterrupt:
        print("\n[안전 종료] 사용자 중단. 출발 허가 없이 종료합니다.", flush=True)
        return 130
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"[5단계 실패] {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
