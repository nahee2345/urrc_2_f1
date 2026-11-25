#!/usr/bin/env python3
"""Optional read-only 3D spectator for a non-official Gazebo rehearsal.

Uses the caller's GZ_PARTITION; does not start a simulator or alter race results.
"""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--stage', choices=('qualifying', 'semi_a', 'semi_b', 'final'), required=True)
    parser.add_argument('--track', choices=('monza', 'spa', 'monaco', 'cota', 'hungaroring', 'baku'), required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    directory = root / 'race/spectator_3d'
    if not (directory / 'server.py').is_file():
        print('[관전] 3D 패키지 미설치: 관전만 생략', flush=True)
        return 0
    env = os.environ.copy()
    if not env.get('GZ_PARTITION'):
        print('[관전] GZ_PARTITION이 없어 관전만 생략', flush=True)
        return 0
    env.update(URRC_SPECTATOR_STAGE=args.stage, URRC_SPECTATOR_TRACK=args.track)
    env['GZ_SIM_RESOURCE_PATH'] = str(root/'src/urrc_track_gazebo/models') + (
        ':'+env['GZ_SIM_RESOURCE_PATH'] if env.get('GZ_SIM_RESOURCE_PATH') else '')
    web = directory / 'web'
    if not (web/'node_modules/three/build/three.module.js').is_file():
        print('[관전] Three.js 최초 설치', flush=True)
        npm = subprocess.run(['npm', 'ci', '--no-audit', '--no-fund'], cwd=web, env=env, check=False)
        if npm.returncode:
            print('[관전] Three.js 설치 실패, 공식 모의주행은 계속합니다', flush=True)
            return 0
    assets = subprocess.run(['/usr/bin/python3', str(directory/'assets.py')], cwd=root, env=env, check=False)
    if assets.returncode:
        print('[관전] 맵 리소스 변환 실패, 공식 모의주행은 계속합니다', flush=True)
        return 0
    print(f'[관전] 자동 시작: {args.stage}/{args.track} (웹 8765, 읽기 전용)', flush=True)
    return subprocess.call(['/usr/bin/python3', '-u', str(directory/'server.py')], cwd=root, env=env)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print('[관전] 오류 (주행에는 영향 없음):', exc, flush=True)
        raise SystemExit(0)
