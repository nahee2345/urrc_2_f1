#!/usr/bin/env python3
"""One-command read-only spectator sidecar for official-profile rehearsals.

Official rehearsal remains the authoritative process. The viewer is optional.
"""
from __future__ import annotations
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

TRACKS = {'monza', 'spa', 'monaco', 'cota', 'hungaroring', 'baku'}

def get_option(args, key):
    for i, item in enumerate(args):
        if item == key and i + 1 < len(args):
            return args[i + 1]
        if item.startswith(key + '='):
            return item.split('=', 1)[1]
    return None

def partitions():
    found = set()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            cmd = (proc/'cmdline').read_bytes().replace(bytes([0]), b' ')
            if b'gz sim' not in cmd:
                continue
            for entry in (proc/'environ').read_bytes().split(bytes([0])):
                if entry.startswith(b'GZ_PARTITION=urrc_race_'):
                    found.add(entry.split(b'=', 1)[1].decode())
        except (OSError, UnicodeError):
            pass
    return found

def main():
    root = Path(sys.argv[1]).resolve()
    args = sys.argv[2:]
    if not args or args[0] != 'rehearse':
        raise ValueError('rehearse 인자 필요')
    stage = args[1] if len(args) > 1 else ''
    track = get_option(args, '--track')
    if track is None:
        assignment = root / 'race/results/v3_maps.json'
        mapping = json.loads(assignment.read_text(encoding='utf-8')).get('maps', {}) if assignment.is_file() else {}
        track = mapping.get(stage, 'monza' if stage == 'vehicles' else None)
    if track not in TRACKS:
        print('[관전] 맵을 확인하지 못해 관전만 생략',flush=True)
    baseline = partitions()
    # Only delegate to the original rehearsal runner: no official result edits.
    run = subprocess.Popen(['bash', str(root/'race_v23.sh'), *args], cwd=root)
    sidecar = None
    try:
        while run.poll() is None:
            if sidecar is None and track in TRACKS:
                fresh = partitions() - baseline
                if len(fresh) == 1:
                    partition = fresh.pop()
                    env = os.environ.copy()
                    env['GZ_PARTITION'] = partition
                    script = root/'race/scripts/spectator_rehearsal_sidecar.py'
                    if script.is_file() and (root/'race/spectator_3d/server.py').is_file():
                        try:
                            sidecar = subprocess.Popen(
                                [sys.executable,str(script),'--root',str(root),'--stage',stage,'--track',track],
                                env=env,cwd=root,stdin=subprocess.DEVNULL,
                                start_new_session=True)
                            print(f'[관전] 모의주행 자동 관전 시작: {track} / 포트 8765',flush=True)
                        except OSError as exc:
                            print(f'[관전] 실행 실패: {exc} (모의주행은 계속 진행)',flush=True)
            time.sleep(.5)
        return run.returncode
    except KeyboardInterrupt:
        # Forward an operator interrupt to the race runner, never orphan it.
        if run.poll() is None:
            run.send_signal(signal.SIGINT)
        try:
            return run.wait(timeout=15)
        except subprocess.TimeoutExpired:
            run.terminate()
            return run.wait()
    finally:
        if sidecar is not None and sidecar.poll() is None:
            try:
                os.killpg(sidecar.pid,signal.SIGTERM)
                sidecar.wait(timeout=5)
            except (OSError,subprocess.TimeoutExpired):
                try: os.killpg(sidecar.pid,signal.SIGKILL)
                except OSError: pass

if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError,OSError,KeyError) as exc:
        print('[관전] 연결 오류:',exc,file=sys.stderr)
        raise SystemExit(2)
