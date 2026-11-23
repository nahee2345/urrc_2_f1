#!/usr/bin/env python3
"""Read-only per-stage entrant gate. Not a substitute for Gazebo live tests."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import shutil
import sys

STAGES = ('qualifying', 'semi_a', 'semi_b', 'final')
LAPS = {'qualifying': 2, 'semi_a': 3, 'semi_b': 3, 'final': 5}


def inspect(root: Path, stage: str, team: str | None):
    sys.path.insert(0, str(root / 'race/scripts'))
    from team_sync_v7 import files_digest
    from vehicle_preflight_v16 import check_entries
    from bracket_v27 import eligible_ids
    from rules_v27 import stage_track

    failures = []
    warnings = []
    def fail(s): failures.append(str(s))
    def warn(s): warnings.append(str(s))

    profile = json.loads((root / 'race/v24_profile.json').read_text(encoding='utf-8'))
    if profile.get('mode') != 'official':
        fail('공식 운영 프로필이 아닙니다')

    source = root / 'src/team'
    registered = root / 'race/entrants'
    folders = sorted(p.name for p in source.iterdir() if p.is_dir() and any(p.iterdir()))
    if not 1 <= len(folders) <= 12:
        fail(f'제출팀 수 범위 위반: {len(folders)}')
    if any(name not in {f'team_{n:02d}' for n in range(1,13)} for name in folders):
        fail('지원하지 않는 team 폴더명이 있습니다')

    if team is not None and stage != 'qualifying':
        fail('--team은 개별 예선에만 지정할 수 있습니다')
    if team and team not in folders:
        fail(f'요청 팀 제출본 없음: {team}')
    try:
        selected = ([team] if team else folders) if stage == 'qualifying' else eligible_ids(stage, root / 'race/results')
        if stage != 'qualifying' and not selected and stage == 'semi_b':
            warn('본선 B 출전팀 없음: 규칙에 따라 생략 가능')
        elif not selected:
            fail('선발된 출전팀이 없습니다')
    except Exception as exc:
        selected = []
        fail(f'공식 진출자 선정 실패: {exc}')

    try:
        track = stage_track(root, stage, None)
    except Exception as exc:
        track = None
        fail(f'공식 맵 확정 검사 실패: {exc}')

    if track:
        track_root = root / 'src/urrc_track_gazebo'
        required = [
            track_root / 'worlds' / f'{track}_race.sdf',
            track_root / 'worlds' / f'{track}_grid.gui.config',
            track_root / 'models' / f'urrc_{track}' / 'model.sdf',
        ]
        mesh = track_root / 'models' / f'urrc_{track}' / 'meshes'
        required.extend(mesh / name for name in (
            'road.obj', 'road_textured.obj', 'asphalt_ground.obj',
            'road_asphalt.mtl', 'asphalt.png', 'offroad.png'))
        for path in required:
            if not path.is_file() or path.stat().st_size == 0:
                fail(f'맵 리소스 누락: {path.relative_to(root)}')

    if not shutil.which('gz'):
        fail('Gazebo gz 명령이 없어 모델 변환 검증 불가')

    try:
        reports = {r['team']: r for r in check_entries(root, conversion=True)}
    except Exception as exc:
        reports = {}
        fail(f'차량 정적검수 실행 실패: {exc}')

    for name in selected:
        info = reports.get(name)
        if not info or info.get('status') in ('MISSING', 'EMPTY', 'FAIL'):
            fail(f'{name}: 차량 정적검수 실패: {info}')
        elif info['status'] == 'REVIEW':
            warn(f'{name}: 수동 검토 필요: {info.get("warnings", [])}')
        original = source / name
        marker_file = registered / name / '.urrc_v7_managed'
        if not marker_file.is_file():
            fail(f'{name}: 공식 접수본 관리 마커 누락 (먼저 sync)')
            continue
        try:
            marker = json.loads(marker_file.read_text(encoding='utf-8'))
            if marker.get('source') != f'src/team/{name}':
                fail(f'{name}: 공식 접수 원본 경로 불일치')
            if marker.get('sha256') != files_digest(original):
                fail(f'{name}: 제출본이 공식 접수 후 변경됨 (재접수·재검수 필요)')
        except Exception as exc:
            fail(f'{name}: 제출물 무결성 확인 실패: {exc}')

    result = {
        'stage': stage, 'required_laps': LAPS[stage], 'track': track,
        'teams': selected, 'static_gate': 'FAIL' if failures else ('REVIEW' if warnings else 'PASS'),
        'errors': failures, 'warnings': warnings,
        'note': '정적검수 결과입니다. 실제 동시 스폰, 카메라 프레임, 조향, 랩 완주는 별도 Gazebo 실주행 검증 필요',
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 2 if failures else 3 if warnings else 0


def main():
    p = argparse.ArgumentParser(description='URRC stage team static gate (read only)')
    p.add_argument('stage', choices=STAGES)
    p.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    p.add_argument('--team', help='qualifying에서만 허용: team_XX')
    args = p.parse_args()
    try:
        return inspect(args.root.resolve(), args.stage, args.team)
    except Exception as exc:
        print(json.dumps({'static_gate': 'FAIL', 'error': str(exc)}, ensure_ascii=False))
        return 2


if __name__ == '__main__':
    sys.exit(main())
