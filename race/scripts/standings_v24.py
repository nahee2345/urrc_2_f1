#!/usr/bin/env python3
"""Read-only official stage standings; no guessed ranks, no changes to scoring."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import sys

STAGES=('qualifying','semi_a','semi_b','final')
LABELS={'qualifying':'예선','semi_a':'본선 A','semi_b':'본선 B','final':'결승'}


def clock(t):
    if not isinstance(t,(float,int)) or isinstance(t,bool) or not math.isfinite(t) or t<0:
        return '-'
    m,s=divmod(t,60)
    return f'{int(m):02d}:{s:06.3f}'


def show(root,stage):
    print(f'\n[{LABELS[stage]}]')
    path=root/'race/results'/f'{stage}.json'
    if not path.is_file():
        print('경기 기록이 없습니다.')
        return
    try:
        data=json.loads(path.read_text(encoding='utf-8'))
    except (OSError,ValueError) as exc:
        print(f'기록 파일 오류: {exc}',file=sys.stderr)
        return
    rows=data.get('results')
    if not isinstance(rows,list):
        print('기록 형식 오류',file=sys.stderr)
        return
    provisional=data.get('provisional') or any(r.get('rank_provisional') for r in rows if isinstance(r,dict))
    if data.get('valid') is True and data.get('state')=='complete' and not provisional:
        verdict='확정'
    elif data.get('state')=='collecting':
        verdict='개별 예선 진행 중(잠정)'
    elif data.get('state')=='running':
        verdict='경기 진행 중(잠정)'
    elif data.get('valid') is False or data.get('state')=='invalid':
        verdict='무효 또는 미확정'
    else:
        verdict='잠정'
    print(f'맵 {data.get("track","-")} | {data.get("laps","-")}랩 | {verdict}')
    print('순위 | 팀      | 완주 | 랩수 | 시간      | 비고')
    def rank(row):
        n=row.get('rank')
        return (n if type(n)==int else 999,str(row.get('entry_id','')))
    for r in sorted((r for r in rows if isinstance(r,dict)),key=rank):
        num=r.get('rank')
        position=str(num) if type(num)==int else '-'
        if r.get('rank_provisional'):position+='*'
        done=r.get('status')=='finished'
        elapsed=clock(r.get('time_seconds')) if done else '-'
        note='판정 필요' if r.get('rank_provisional') else r.get('reason','-') or '-'
        print(f'{position:>4} | {str(r.get("entry_id","-")):<7} | {"완주" if done else "실격" if r.get("status")=="DQ" else "DNF":<4} | {str(r.get("laps",0)):>4} | {elapsed:>9} | {note}')
    if data.get('error'):print('오류:',data['error'])


def main():
    p=argparse.ArgumentParser()
    p.add_argument('stage',nargs='?',choices=(*STAGES,'all'),default='all')
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    args=p.parse_args()
    root=args.root.resolve()
    profile=root/'race/v24_profile.json'
    try:
        config=json.loads(profile.read_text(encoding='utf-8'))
        if config.get('mode')!='official':
            raise ValueError('대회용 urrc_22 폴더가 아닙니다.')
    except (OSError,ValueError) as e:
        print('대회 프로필 오류:',e,file=sys.stderr)
        return 2
    for stage in STAGES if args.stage=='all' else (args.stage,):show(root,stage)
    return 0


if __name__=='__main__':raise SystemExit(main())
