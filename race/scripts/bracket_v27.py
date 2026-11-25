#!/usr/bin/env python3
"""v27 bracket. Only finished qualifiers advance; final grid requires operator approval."""
from __future__ import annotations
from pathlib import Path
from rules_v27 import complete_rows, approved_grid


def eligible_ids(stage:str,results:Path):
    if stage=='qualifying':return None
    root=Path(results).resolve().parents[1]
    _,all_rows=complete_rows(root,'qualifying')
    valid=[r for r in all_rows if r.get('status')=='finished']
    n=len(valid)
    if n<2 or n>12:raise ValueError(f'본선 출전 가능한 예선 완주자 2~12팀 필요: {n}팀')
    if stage in ('semi_a','semi_b'):
        if n<=3:
            return [r['entry_id'] for r in valid] if stage=='semi_a' else []
        return [r['entry_id'] for r in valid
                if r['rank']%2==(1 if stage=='semi_a' else 0)]
    if stage=='final':return approved_grid(root)
    raise ValueError('지원하지 않는 경기 단계: '+stage)
