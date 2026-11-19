#!/usr/bin/env python3
"""v27 checked extension after v26 operator runtime adaptation; no driver file changes."""
from __future__ import annotations
import ast


def single(source, old, new, label):
    if source.count(old)!=1:
        raise RuntimeError(f'v27 실행기 변경 불가: {label} 소스 구조 불일치')
    return source.replace(old,new,1)


def adapt_run_v27(source:str)->str:
    # Qualifying: the white start/finish line is first crossed BEFORE lap 1,
    # then crossing #2 completes lap 1; crossing #4 completes lap 3.
    source=single(source,
       "    laps=2 if mode=='qualifying' else (final_laps if final_laps is not None else int(cfg['final_laps']))",
       "    laps=3 if mode=='qualifying' else (final_laps if final_laps is not None else int(cfg['final_laps']))",
       '예선 3랩')
    source=single(source,
       "        row=result_row(e,monitor.snapshot(e['entry_id']),reason,position)\n        out.append(row)",
       "        row=result_row(e,monitor.snapshot(e['entry_id']),reason,position)\n"
       "        if mode=='qualifying' and reason.startswith('운영자 수동 DNF:'):\n"
       "            row['status']='DQ'\n"
       "        out.append(row)",
       '예선 수동 실격 DQ 표기')
    source=single(source,
       "            shown={e['entry_id']:0 for e in active}\n",
       "            shown={e['entry_id']:0 for e in active}\n            live_next=0.0\n",
       '실시간 집계 주기')
    source=single(source,
       "            while active:\n                check_world()\n                if time.monotonic()>=_v13_next_diag:",
       "            while active:\n                check_world()\n"
       "                if time.monotonic()>=live_next:\n"
       "                    from live_console_v27 import view as live_view\n"
       "                    live_view(monitor,entries,globals().get('stage',mode),track,laps,out)\n"
       "                    live_next=time.monotonic()+2.0\n"
       "                if time.monotonic()>=_v13_next_diag:",
       '동일 터미널 실시간 순위 출력')
    ast.parse(source)
    return source
