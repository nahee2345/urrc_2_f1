#!/usr/bin/env python3
"""Explicit operator approval of top-three A/B and distance-normalized final grid."""
import argparse
import json
from pathlib import Path
from rules_v27 import approve,finalist_plan


def main(argv=None):
    p=argparse.ArgumentParser(description='결승 진출 A/B 각 1~3위 승인')
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    p.add_argument('--preview',action='store_true',help='승인 없이 선정 후보와 그리드만 표시')
    x=p.parse_args(argv)
    try:
        plan=finalist_plan(x.root.resolve()) if x.preview else approve(x.root.resolve())
        print('결승 '+('예상' if x.preview else '승인')+' 그리드 (앞 순서가 1번 그리드)')
        for i,row in enumerate(plan['details'],1):
            print(f'{i:>2}번 | {row["team"]} | {row["group"]} {row["group_rank"]}위'
                  f' | 실제 {row["seconds"]:.3f}초 / 보정 비교 {row["normalized_seconds"]:.3f}초')
        print('거리 보정은 서로 다른 코스의 난도 차이를 보정하지 못하는 참고 값입니다.')
        if not x.preview:print('승인 파일: race/results/finalists_v27.json')
        return 0
    except (OSError,ValueError,KeyError,TypeError) as exc:
        print('[결승 승인 FAIL]',exc)
        return 2

if __name__=='__main__':raise SystemExit(main())
