#!/usr/bin/env python3
"""Run an official two-lap qualifier for exactly one team and accumulate all results.

Does not edit entrant files. Keeps full stage result INCOMPLETE until every
registered entrant has a valid one-team qualifying run. Official race manager
performs world launch, vehicle checks, timing, signal and DNF adjudication.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import uuid

TEAM_RE = re.compile(r'^team_(?:0[1-9]|1[0-2])$')


def atomic_json(path: Path, obj: dict) -> None:
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    os.replace(tmp, path)


def dir_digest(path: Path) -> str:
    hasher = hashlib.sha256()
    for item in sorted(path.rglob('*')):
        if not item.is_file() or any(part in ('build', 'install', 'log', '__pycache__') for part in item.relative_to(path).parts):
            continue
        if item.is_symlink():
            raise ValueError(f'외부 파일을 참조하는 심볼릭 링크는 허용하지 않습니다: {item}')
        rel = item.relative_to(path).as_posix()
        if item.name.startswith('.urrc_'):
            continue
        hasher.update(rel.encode('utf-8') + b'\0' + item.read_bytes() + b'\0')
    return hasher.hexdigest()


def snapshot(root: Path) -> dict:
    profiles = json.loads((root / 'race/v24_profile.json').read_text(encoding='utf-8'))
    if profiles.get('mode') != 'official':
        raise ValueError('공식 환경에서만 팀별 예선을 실행할 수 있습니다.')
    mapsfile = root / 'race/results/v3_maps.json'
    if not mapsfile.is_file():
        raise ValueError('맵 배정이 없습니다. bash race_v24.sh refresh-maps 를 먼저 실행하십시오.')
    maps = json.loads(mapsfile.read_text(encoding='utf-8'))
    track = maps.get('maps', {}).get('qualifying')
    if track not in ('monza', 'hungaroring', 'spa', 'cota', 'baku'):
        raise ValueError('예선 맵 배정이 잘못되었습니다.')
    entrants = root / 'race/entrants'
    folders = sorted(p for p in entrants.glob('team_*') if p.is_dir() and (p / 'race_entry.json').is_file())
    if not (1 <= len(folders) <= 12) or any(not TEAM_RE.fullmatch(p.name) for p in folders):
        raise ValueError('등록된 참가자는 team_01~team_12 형식으로 1~12팀이어야 합니다.')
    return dict(track=track, teams=[p.name for p in folders],
                fingerprints={p.name: dir_digest(p) for p in folders},
                maps_sha256=hashlib.sha256(mapsfile.read_bytes()).hexdigest(),
                config_sha256=hashlib.sha256((root / 'race/event_config.json').read_bytes()).hexdigest())


def accumulate(root: Path, session: dict) -> tuple[dict, bool]:
    resultdir = root / 'race/results'
    base = resultdir / 'qualifying_individual'
    completed = []
    last = None
    for team in session['teams']:
        file = base / f'{team}.json'
        if not file.is_file():
            continue
        payload = json.loads(file.read_text(encoding='utf-8'))
        if payload.get('valid') is not True or payload.get('state') != 'complete':
            raise ValueError(f'{team}: 확정된 개별 예선 결과가 아닙니다')
        rows = payload.get('results')
        if not isinstance(rows, list) or len(rows) != 1 or rows[0].get('entry_id') != team:
            raise ValueError(f'{team}: 개별 예선 기록의 팀 ID가 일치하지 않습니다.')
        completed.append(dict(rows[0]))
        last = payload

    # Same ranking algorithm as the official race manager. Missing teams do not
    # receive invented results; a partial field cannot authorize semifinals.
    sys.path.insert(0, str(root / 'race/scripts'))
    from timing import rank_rows
    from race_manager import save_results

    completed = rank_rows(completed)
    all_done = len(completed) == len(session['teams'])
    provisional = any(bool(row.get('rank_provisional')) for row in completed)
    overall = dict(last or {})
    overall.update(event={'event_id': session['event_id'], 'track': session['track']},
                   track=session['track'], laps=2, v2_stage='qualifying',
                   valid=all_done, state='complete' if all_done else 'collecting',
                   provisional=provisional, results=completed,
                   entry_fingerprints=session['fingerprints'])
    aggregate_dir = base / 'aggregate'
    aggregate_dir.mkdir(parents=True, exist_ok=True)
    save_results(resultdir, aggregate_dir, 'qualifying', overall)

    if all_done:
        import csv
        target = root / 'src/fast_lap.txt'
        with target.open('w', encoding='utf-8', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['rank', 'team', 'total_seconds', 'status'])
            for row in completed:
                elapsed = row.get('time_seconds')
                writer.writerow([row['rank'], row['entry_id'],
                                 f'{elapsed:.6f}' if isinstance(elapsed, (int, float)) else '',
                                 row['status']])
    from rules_v27 import ranked_qualifying
    ranked_qualifying(root,overall)
    return overall, all_done


def execute(root: Path, team: str) -> int:
    if not TEAM_RE.fullmatch(team):
        raise ValueError('팀 이름은 team_01~team_12 형식이어야 합니다.')
    results = root / 'race/results'
    results.mkdir(parents=True, exist_ok=True)
    base = results / 'qualifying_individual'
    base.mkdir(parents=True, exist_ok=True)
    # Avoid concurrent official qualifying commands corrupting a shared result.
    with (base / '.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        now = snapshot(root)
        if team not in now['teams']:
            raise ValueError(f'{team}은 공식 접수된 팀이 아닙니다. check/sync/prepare를 확인하십시오.')
        manifest_file = base / 'manifest.json'
        if manifest_file.is_file():
            session = json.loads(manifest_file.read_text(encoding='utf-8'))
            for key in ('track', 'teams', 'fingerprints', 'maps_sha256', 'config_sha256'):
                if session.get(key) != now[key]:
                    raise ValueError(f'예선 시작 후 등록팀·설정·맵이 변경되었습니다: {key}. 기존 기록에 합칠 수 없습니다.')
        else:
            if (results / 'qualifying.json').exists() or any(base.glob('team_*.json')):
                raise ValueError('기존 예선 기록이 있습니다. 새 예선 세션에 자동 병합하지 않습니다.')
            session = {**now, 'event_id': uuid.uuid4().hex[:12]}
            atomic_json(manifest_file, session)
        if (base / f'{team}.json').exists():
            raise ValueError(f'{team}은 이미 공식 예선 기록이 있습니다. 중복 출전할 수 없습니다.')
        print(f'[예선 단독] {team} | {session["track"]} | 2랩 | 전체 {len(session["teams"])}팀', flush=True)
        env = os.environ.copy()
        env['URRC_QUALIFYING_ONLY'] = team
        # Preserve the official stage launcher and its validation/signal/timing.
        cmd = ['bash', str(root / 'race_v23.sh'), 'qualifying', '--track', session['track']]
        # Optional spectator: isolated from the official qualifier and timing.
        spectator = None
        watcher = root / 'race/scripts/spectator_autostart.py'
        if watcher.is_file() and (root / 'race/spectator_3d/run_official.sh').is_file():
            try:
                spectator = subprocess.Popen(
                    [sys.executable, str(watcher), '--root', str(root),
                     '--parent-pid', str(os.getpid()), '--team', team],
                    cwd=root, env=env, stdin=subprocess.DEVNULL)
            except OSError as exc:
                print('[관전] 실행 실패 (예선은 계속 진행):', exc, flush=True)
        try:
            run = subprocess.run(cmd, cwd=root, env=env, check=False)
        finally:
            if spectator is not None and spectator.poll() is None:
                spectator.terminate()
                try:
                    spectator.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    spectator.kill()
                    spectator.wait()
        if run.returncode != 0:
            print(f'[예선 단독] {team} 실행 실패 (종료 코드 {run.returncode}). 공식 집계에 포함하지 않습니다.', file=sys.stderr)
            # If previous teams have qualified, restore their cumulative summary.
            if any(base.glob('team_*.json')):
                accumulate(root, session)
                subprocess.run([sys.executable, str(root / 'race/scripts/summary_txt_v18.py'),
                                '--root', str(root)], check=False)
            return run.returncode
        file = results / 'qualifying.json'
        if not file.is_file():
            raise ValueError('개별 예선 결과 파일이 생성되지 않았습니다.')
        payload = json.loads(file.read_text(encoding='utf-8'))
        rows = payload.get('results')
        if (payload.get('valid') is not True or payload.get('state') != 'complete' or
            payload.get('track') != session['track'] or not isinstance(rows, list) or
            len(rows) != 1 or rows[0].get('entry_id') != team):
            raise ValueError('개별 예선 검증 실패. 임의로 공식 집계하지 않습니다.')
        if payload.get('laps')!=2 or rows[0].get('laps',0)>2:
            raise ValueError('2랩 예선 공식 기록이 아닙니다.')
        if rows[0].get('status')=='finished' and rows[0].get('laps')!=2:
            raise ValueError('예선 2랩 완주 검증 실패')
        atomic_json(base / f'{team}.json', payload)
        overall, all_done = accumulate(root, session)
        subprocess.run([sys.executable, str(root / 'race/scripts/summary_txt_v18.py'),
                        '--root', str(root)], check=False)
        print(f'[예선 누적] {len(overall["results"])}/{len(session["teams"])}팀 완료', flush=True)
        if all_done:
            print('예선 종료: 전체 팀 공식 순위가 확정 가능한 상태입니다. standings qualifying 로 확인하십시오.', flush=True)
        else:
            waiting = [x for x in session['teams'] if not (base / f'{x}.json').exists()]
            print('남은 팀:', ', '.join(waiting), ' / 본선 진출은 아직 확정되지 않았습니다.', flush=True)
        return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='공식 예선 1팀씩 선택 실행, 결과 안전 누적')
    parser.add_argument('--team', required=True)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args(argv)
    try:
        return execute(args.root.expanduser().resolve(), args.team)
    except (ValueError, OSError, KeyError, json.JSONDecodeError, BlockingIOError) as exc:
        print('[예선 단독] 오류:', exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('[예선 단독] 운영자 중단. 현재 팀 기록은 누적하지 않습니다.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
