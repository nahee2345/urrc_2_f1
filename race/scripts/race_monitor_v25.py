#!/usr/bin/env python3
"""Read-only v25 live race order and checkpoint time gaps from base_link gate telemetry.

Time gaps are measured at the SAME ordered checkpoint, not instantaneous physical gaps.
Never changes official scoring or entrant packages.
"""
from __future__ import annotations
import argparse
import json
import math
import time
from pathlib import Path

from race_monitor_v6 import follow_jsonl, load_json, format_seconds

STAGES = ('qualifying', 'semi_a', 'semi_b', 'final')


def summarize(events, grid, results=None):
    state = {team: dict(team=team, grid=i + 1, laps=0, gate=0,
                        status='WAITING', crossing_times=[], started=None, now=None)
             for i, team in enumerate(grid)}
    for ev in events:
        team = ev.get('entry_id')
        if team not in state:
            continue
        s = state[team]
        kind = ev.get('type')
        if kind == 'arm':
            s.update(status='READY', laps=0, gate=0, crossing_times=[],
                     started=None, now=None)
        elif kind == 'pose':
            sample = ev.get('sample')
            if isinstance(sample, list) and sample:
                s['now'] = sample[0]
        elif kind == 'crossing':
            stamp = ev.get('time')
            if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                continue
            s['crossing_times'].append(float(stamp))
            s['gate'] = ev.get('gate', 0)
            if ev.get('gate') == 0:
                if s['started'] is None:
                    s['started'] = float(stamp)
                else:
                    s['laps'] += 1
            s['status'] = 'RUNNING'
    for r in results or []:
        team = r.get('entry_id') if isinstance(r, dict) else None
        if team in state:
            s = state[team]
            s['laps'] = r.get('laps', s['laps'])
            s['status'] = 'FINISHED' if r.get('status') == 'finished' else 'DNF'

    def rank_key(s):
        status_class = 0 if s['status'] in ('RUNNING', 'FINISHED') else (1 if s['status'] == 'DNF' else 2)
        count = len(s['crossing_times'])
        # Same checkpoint count: the earlier crossing is the vehicle in front.
        last_stamp = s['crossing_times'][-1] if count else math.inf
        return status_class, -count, last_stamp, s['grid']

    rows = sorted(state.values(), key=rank_key)
    for i, s in enumerate(rows):
        s['rank'] = i + 1
        count = len(s['crossing_times'])
        s['crossings'] = count
        now, started = s['now'], s['started']
        s['elapsed_s'] = (max(0.0, now - started)
                          if isinstance(now, (int, float)) and started is not None and now >= started else None)
        s['ahead'] = rows[i-1]['team'] if i else None
        s['gap_ahead_s'] = None
        s['gap_leader_s'] = None
        s['gap_checkpoints'] = None
        if count and rows and s['status'] in ('RUNNING', 'FINISHED'):
            lead = rows[0]
            s['gap_checkpoints'] = max(0, len(lead['crossing_times']) - count)
            # Compare when this car and the car ahead crossed the SAME checkpoint.
            # A positive value means that the car ahead passed earlier.
            if i:
                ahead = rows[i-1]
                if len(ahead['crossing_times']) >= count:
                    s['gap_ahead_s'] = max(0.0, s['crossing_times'][count-1] - ahead['crossing_times'][count-1])
                if len(lead['crossing_times']) >= count:
                    s['gap_leader_s'] = max(0.0, s['crossing_times'][count-1] - lead['crossing_times'][count-1])
            else:
                s['gap_ahead_s'] = s['gap_leader_s'] = 0.0
    for s in rows:
        s.pop('crossing_times')
    return rows


def gap_label(seconds):
    return f'+{seconds:.3f}s' if seconds is not None else '-'


def render(rows, stage, track):
    print('\033[2J\033[H', end='')
    print(f'URRC F1 실시간 잠정 순위 | {stage} | {track} | base_link 체크포인트 계측')
    print('순위 | 팀       | 상태     | 랩  | 경과시간  | 앞차       | 앞차 시간차 | 선두 시간차')
    print('-' * 96)
    for r in rows:
        print(f"{r['rank']:>4} | {r['team']:<8} | {r['status']:<8} | {r['laps']:>3} | "
              f"{format_seconds(r['elapsed_s']):>10} | {str(r['ahead'] or '-'):>10} | "
              f"{gap_label(r['gap_ahead_s']):>11} | {gap_label(r['gap_leader_s']):>11}")
    print('\n시간차 = 동일 체크포인트를 통과한 시각 차이(최신 통과 지점 기준 추정).')
    print('실제 두 차량 사이의 순간 시간·거리 차이나 최종 확정 순위는 아닙니다. Ctrl+C는 모니터만 종료합니다.')


def monitor(root, stage, interval=1.0, once=False):
    results_dir = root / 'race/results'
    gridfile = results_dir / 'active_grid_v6.json'
    if not gridfile.exists():
        raise ValueError('현재 경기 그리드 정보가 없습니다. 먼저 실제 경기를 시작하십시오.')
    grid_data = load_json(gridfile) or {}
    if grid_data.get('stage') != stage:
        raise ValueError(f"현재 경기 단계는 {grid_data.get('stage')}입니다. 요청한 {stage}가 아닙니다.")
    grid = grid_data.get('teams') or []
    if not isinstance(grid, list) or not grid:
        raise ValueError('계측 대상 팀 목록이 없습니다.')
    result_file = results_dir / f'{stage}.json'
    offset = 0
    pending = b''
    events = []
    poses = {}
    current_audit = None
    while True:
        current = load_json(result_file, {}) or {}
        audit = current.get('audit_directory')
        if audit and audit != current_audit:
            current_audit, offset, pending, events, poses = audit, 0, b'', [], {}
        if current_audit:
            new, offset, pending = follow_jsonl(root / current_audit / 'timing.jsonl', offset, pending)
            for ev in new:
                if ev.get('type') == 'pose':
                    poses[ev.get('entry_id')] = ev
                elif ev.get('type') in ('arm', 'crossing'):
                    events.append(ev)
        rows = summarize(events + list(poses.values()), grid, current.get('results', []))
        render(rows, stage, grid_data.get('track', '?'))
        if once or current.get('state') in ('complete', 'invalid'):
            break
        time.sleep(interval)


def main(argv=None):
    parser = argparse.ArgumentParser(description='URRC v25 base_link 실시간 잠정 순위·체크포인트 시간차')
    parser.add_argument('stage', choices=STAGES)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args(argv)
    try:
        monitor(args.root.resolve(), args.stage, max(0.25, args.interval), args.once)
    except (OSError, ValueError, KeyError) as exc:
        print('모니터 오류:', exc)
        return 2
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
