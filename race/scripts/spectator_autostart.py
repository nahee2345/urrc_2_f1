#!/usr/bin/env python3
"""Optional read-only 3D spectator sidecar for official individual qualifying.

Never controls Gazebo, modifies entrant files, or decides race results.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

stop = False
def request_stop(_signal, _frame):
    global stop
    stop = True

def alive(pid):
    try:
        os.kill(int(pid), 0)
        return True
    except (ValueError, TypeError, OSError):
        return False

def race_partition():
    """Find the one official Gazebo transport partition, never touch other runs."""
    found = set()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = (proc / 'cmdline').read_bytes().replace(bytes([0]), b' ')
            if b'gz sim' not in args:
                continue
            for item in (proc / 'environ').read_bytes().split(bytes([0])):
                if item.startswith(b'GZ_PARTITION=urrc_race_'):
                    found.add(item.split(b'=', 1)[1].decode('utf-8'))
        except (OSError, UnicodeError):
            continue
    return next(iter(found)) if len(found) == 1 else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    parser.add_argument('--team', required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    launch = root / 'race/spectator_3d/run_official.sh'
    if not launch.is_file():
        print('[관전] 패키지가 없어 자동 관전을 생략합니다', flush=True)
        return 0
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    status_path = root / 'race/results/operator_v26_active.json'
    deadline = time.monotonic() + 150
    started_at = time.time()
    viewer = None
    try:
        while not stop and alive(args.parent_pid) and time.monotonic() < deadline:
            try:
                status = json.loads(status_path.read_text(encoding='utf-8'))
                fresh = status_path.stat().st_mtime >= started_at - 2
            except (OSError, ValueError):
                status, fresh = {}, False
            if (fresh and status.get('stage') == 'qualifying'
                    and args.team in status.get('teams', [])
                    and alive(status.get('pid'))):
                partition = race_partition()
                if not partition:
                    time.sleep(0.5)
                    continue
                print('[관전] 공식 예선 및 Gazebo 확인: 3D 웹 관전 자동 실행', flush=True)
                env = os.environ.copy()
                env['GZ_PARTITION'] = partition
                viewer = subprocess.Popen(
                    ['bash', str(launch)], cwd=root,
                    env=env, stdin=subprocess.DEVNULL,
                )
                break
            time.sleep(0.5)
        if viewer is None:
            if not stop:
                print('[관전] 실행 중인 예선 세션 미발견, 관전만 생략', flush=True)
            return 0
        while not stop and alive(args.parent_pid):
            if viewer.poll() is not None:
                print(f'[관전] 관전 서버 종료 코드 {viewer.returncode}; 공식 예선은 계속 진행', flush=True)
                return 0
            time.sleep(0.5)
        return 0
    finally:
        if viewer is not None and viewer.poll() is None:
            viewer.terminate()
            try:
                viewer.wait(timeout=5)
            except subprocess.TimeoutExpired:
                viewer.kill()
                viewer.wait()

if __name__ == '__main__':
    sys.exit(main())
