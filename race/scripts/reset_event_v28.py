#!/usr/bin/env python3
"""Reset event results and draw new maps; never modify entrant or track sources."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import shutil
import sys

OUTPUTS = ('race/results', 'race/previews', 'race/test_reports')

def generated_paths(root: Path) -> list[Path]:
    root = root.resolve()
    paths = []
    for relative in OUTPUTS:
        directory = root / relative
        if directory.is_symlink():
            raise ValueError(f'출력 폴더 심볼릭 링크 사용 금지: {relative}')
        if not directory.exists():
            continue
        if not directory.is_dir():
            raise ValueError(f'출력 경로가 폴더가 아닙니다: {relative}')
        for path in sorted(directory.iterdir()):
            if path.is_symlink():
                raise ValueError(f'출력 항목 심볼릭 링크 사용 금지: {path.relative_to(root)}')
            paths.append(path)
    return paths

def reset(root: Path, *, yes: bool = False, stdin=None, stdout=None) -> dict:
    from stage_v3 import refresh, assignments, load, STAGES
    root = root.resolve()
    if not (root/'race/scripts/stage_v3.py').is_file():
        raise ValueError('유효한 대회장 폴더가 아닙니다')
    targets = generated_paths(root)
    out = sys.stdout if stdout is None else stdout
    print('[v28 RESET] 다음 측정 기록을 삭제합니다:', file=out)
    for path in targets:
        print('  '+str(path.relative_to(root)), file=out)
    print('[v28 RESET] 팀 소스·맵 메시·설정 파일은 보존합니다.',file=out)
    if not yes:
        instream = sys.stdin if stdin is None else stdin
        if not instream.isatty() and stdin is None:
            raise ValueError('대화형 터미널에서 RESET을 입력하거나 --yes 옵션을 사용하십시오')
        print('[v28 RESET] 모든 기록을 지우고 새 맵을 추첨하려면 RESET 입력: ',end='',file=out,flush=True)
        if instream.readline().strip() != 'RESET':
            raise ValueError('초기화가 취소됐습니다. 기존 기록과 맵 유지')
    # Select the next valid draw before deleting old results.
    import random
    previous = root/'race/results/v3_maps.json'
    old = load(previous) if previous.is_file() else None
    options = list(assignments())
    if old:
        candidates = [o for o in options if all(o[k]!=old['maps'][k] for k in STAGES)]
        options = candidates or [o for o in options if o!=old['maps']] or options
    selected = random.SystemRandom().choice(options)
    for path in targets:
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
    class Chosen:
        def choice(self, choices):
            assert selected in choices
            return selected
    data = refresh(root/'race/results/v3_maps.json', rng=Chosen())
    print('[v28 RESET] 초기화 완료, 새 맵 배정:',json.dumps(data['maps'],ensure_ascii=False),file=out)
    return data

def main(argv=None):
    p=argparse.ArgumentParser(description='맵 재추첨과 대회·모의주행 기록 초기화')
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    p.add_argument('--yes',action='store_true',help='확인 없이 초기화')
    args=p.parse_args(argv)
    try:
        reset(args.root,yes=args.yes)
        return 0
    except (OSError,ValueError,KeyError,TypeError) as e:
        print('[v28 RESET] 실패:',e,file=sys.stderr)
        return 2

if __name__=='__main__':
    sys.exit(main())
