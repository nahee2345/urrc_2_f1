#!/usr/bin/env python3
"""Single-terminal standings from the race manager's official base_link gate timers."""
from __future__ import annotations
import math
from race_monitor_v25 import summarize, gap_label, format_seconds


def lap_label(gate0_count:int, target:int)->str:
    if gate0_count==0:return '출발선 전'
    if gate0_count>target:return f'{target}랩 완주'
    if gate0_count==1:return f'1/{target}랩 주행'
    return f'{gate0_count-1}랩 완주/{gate0_count}랩째'


def view(monitor, entries, stage, track, laps, completed=()):
    """Read-only. Missing base_link data stays WAITING; no fabricated timestamps."""
    events=[]
    teams=[e['entry_id'] for e in entries]
    with monitor.cv:
        for eid in teams:
            timer=monitor.timers[eid]
            if timer.active:events.append(dict(type='arm',entry_id=eid))
            for hit in timer.crossings:
                events.append(dict(type='crossing',entry_id=eid,**hit))
            if timer.previous is not None:
                events.append(dict(type='pose',entry_id=eid,sample=[timer.previous[0]]))
    events.sort(key=lambda e: (0 if e['type']=='arm' else 1 if e['type']=='crossing' else 2,
                               e.get('time',0)))
    rows=summarize(events,teams,completed)
    disqualified={r.get('entry_id') for r in completed if r.get('status')=='DQ'}
    for row in rows:
        if row['team'] in disqualified:row['status']='DQ'
    print(f'\n[실시간 잠정 순위] {stage} / {track} / base_link / 목표 {laps}랩',flush=True)
    print('순위 | 팀      | 랩 진행상태       | 경과시간 | 앞차     | 앞차 시간차 | 선두 시간차',flush=True)
    print('-'*93,flush=True)
    for row in rows:
        gate0=sum(1 for ev in events if ev.get('entry_id')==row['team'] and ev.get('type')=='crossing' and ev.get('gate')==0)
        label=lap_label(gate0,laps)
        print(f"{row['rank']:>4} | {row['team']:<8}| {label:<16}| {format_seconds(row.get('elapsed_s')):>9} | "
              f"{(row.get('ahead') or '-'):>8} | {gap_label(row.get('gap_ahead_s')):>11} | {gap_label(row.get('gap_leader_s')):>11}",flush=True)
    print('시간차: 두 차량이 같은 체크포인트를 통과한 시뮬레이션 시각 차이(순간 간격 아님)',flush=True)
    return rows
