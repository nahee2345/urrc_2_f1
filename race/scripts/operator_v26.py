#!/usr/bin/env python3
"""Operator-only spawn and DNF mailbox for active URRC official races.

Operator spawn uses the original race_manager.spawn function, preserving
PosePublisher/base_link timing, launch context and grid placement.
No team package is edited. Requests are queued for the race process to apply.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import re
import sys
import time
import uuid

TEAM = re.compile(r'^team_(?:0[1-9]|1[0-2])$')
ACTIVE = Path('race/results/operator_v26_active.json')

class AutoDNF(Exception):
    """The only individual automatic DNF reasons: camera/sensor or base_link timing link."""


def atomic(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name('.'+path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        temp.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        os.replace(temp, path)
    finally:
        temp.unlink(missing_ok=True)


def read(path: Path):
    return json.loads(path.read_text(encoding='utf-8'))


def alive(pid):
    try: os.kill(int(pid), 0); return True
    except (OSError, ValueError, TypeError): return False


class OperatorSession:
    def __init__(self, root: Path, stage: str, track: str, entries, run_dir: Path):
        self.root = Path(root).resolve()
        self.path = self.root / ACTIVE
        self.run_dir = Path(run_dir).resolve()
        if not self.run_dir.is_relative_to((self.root/'race/results/runs').resolve()):
            raise ValueError('운영자 명령 디렉터리가 경기 결과 폴더 외부입니다')
        if self.path.exists():
            prior=read(self.path)
            if alive(prior.get('pid')):
                raise RuntimeError('기존 대회 프로세스가 아직 실행 중입니다: PID '+str(prior['pid']))
            self.path.unlink()
        self.token = uuid.uuid4().hex
        self.queue = self.run_dir/'operator_commands_v26'
        self.queue.mkdir(parents=True,exist_ok=True)
        teams=[e['entry_id'] for e in entries]
        if not teams or any(not TEAM.fullmatch(e) for e in teams):
            raise ValueError('team_01~team_12 이외의 팀은 운영자 제어 불가')
        self.info=dict(schema=26,pid=os.getpid(),session=self.token,stage=stage,track=track,
                       directory=str(self.queue.relative_to(self.root)),teams=teams,
                       status={team:'AWAITING_SPAWN' for team in teams})
        atomic(self.path, self.info)
        print('[v26] 운영자 스폰: 별도 터미널에서 bash race_v24.sh spawn team_XX',flush=True)
        print('[v26] 운영자 제거: 별도 터미널에서 bash race_v24.sh dnf team_XX --reason "사유"',flush=True)

    def status(self, team, value):
        if team in self.info['status']:
            self.info['status'][team]=value
            atomic(self.path,self.info)

    def commands(self, team, accepted):
        queued=[]
        for file in sorted(self.queue.glob('*.json')):
            try: data=read(file)
            except (OSError,ValueError): continue
            if data.get('session')==self.token and data.get('team')==team and data.get('action') in accepted:
                queued.append((file,data))
        return queued

    def consume(self, team, accepted):
        msgs=self.commands(team, accepted)
        if not msgs:return None
        # DNF takes priority over an earlier unprocessed spawn request.
        if 'dnf' in accepted:
            msgs.sort(key=lambda fd:(fd[1].get('action')!='dnf',fd[0].name))
        file, data=msgs[0]
        os.replace(file,file.with_suffix('.accepted'))
        return data

    def wait_spawn(self, team, check_world):
        print('[v26] '+team+' 스폰 지시 대기 중 (자동 스폰하지 않음)',flush=True)
        while True:
            cmd=self.consume(team,{'spawn','dnf'})
            if cmd:
                return cmd
            check_world()
            time.sleep(.25)

    def wait_dnf(self, team, check_world):
        print('[v26] '+team+' 운영자 수동 DNF 지시 대기 중',flush=True)
        while True:
            cmd=self.consume(team,{'dnf'})
            if cmd:
                return str(cmd.get('reason') or '운영자 수동 DNF')
            check_world()
            time.sleep(.25)

    def requested_dnf(self, team):
        cmd=self.consume(team,{'dnf'})
        return str(cmd.get('reason') or '운영자 수동 DNF') if cmd else None

    def close(self):
        try:
            if self.path.exists() and read(self.path).get('session')==self.token:
                self.path.unlink()
        except (OSError,ValueError):pass


def command(root: Path, action: str, team: str, reason: str) -> int:
    root=root.expanduser().resolve()
    if not TEAM.fullmatch(team):
        raise ValueError('팀 이름은 team_01~team_12 형식이어야 합니다')
    profile=read(root/'race/v24_profile.json')
    if profile.get('mode')!='official':
        raise ValueError('urrc_22 공식 대회 환경에서만 사용 가능합니다')
    active=root/ACTIVE
    if not active.exists():
        raise ValueError('진행 중인 공식 경기가 없습니다. 먼저 예선·본선·결승을 실행하십시오')
    info=read(active)
    if not alive(info.get('pid')):
        raise ValueError('경기 프로세스가 종료되었습니다. 이전 명령을 실행하지 않습니다')
    if team not in info.get('teams',[]):
        raise ValueError('현재 경기 출전팀이 아닙니다: '+team)
    if action=='spawn' and info.get('status',{}).get(team) not in ('AWAITING_SPAWN',):
        raise ValueError('현재 스폰 지시를 받을 수 없는 상태입니다: '+str(info.get('status',{}).get(team)))
    if action=='dnf' and info.get('status',{}).get(team) in ('FINISHED','DNF'):
        raise ValueError('이미 완주 또는 DNF 처리된 팀입니다')
    directory=(root/info['directory']).resolve()
    if not directory.is_relative_to((root/'race/results/runs').resolve()):
        raise ValueError('운영 명령 경로가 결과 폴더 밖입니다')
    if not directory.is_dir():raise ValueError('경기 명령 디렉터리가 없습니다')
    cmd=dict(schema=26,session=info['session'],team=team,action=action,
             reason=reason.strip() if action=='dnf' else '',timestamp=time.time())
    target=directory/(f'{time.time_ns():020d}_{uuid.uuid4().hex}.json')
    atomic(target,cmd)
    print(f'[v26] {action.upper()} 요청 접수: {team} / {info["stage"]} / {info["track"]}')
    print('경기 터미널에서 실제 스폰·DNF 완료 메시지를 확인하십시오. 요청 접수와 실제 처리 완료는 다릅니다.')
    return 0


def main(argv=None):
    p=argparse.ArgumentParser(description='URRC v26 운영자 수동 스폰·DNF 제어')
    p.add_argument('action',choices=('spawn','dnf','status'))
    p.add_argument('team',nargs='?')
    p.add_argument('--reason',default='운영자 수동 DNF')
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    a=p.parse_args(argv)
    try:
        if a.action=='status':
            path=a.root.expanduser().resolve()/ACTIVE
            if not path.is_file():raise ValueError('실행 중인 경기가 없습니다')
            print(json.dumps(read(path),indent=2,ensure_ascii=False))
            return 0
        if not a.team:raise ValueError('팀 이름이 필요합니다: team_01')
        return command(a.root,a.action,a.team,a.reason)
    except (ValueError,OSError,KeyError,TypeError) as exc:
        print('[v26] 운영자 명령 오류:',exc,file=sys.stderr)
        return 2

if __name__=='__main__':raise SystemExit(main())
