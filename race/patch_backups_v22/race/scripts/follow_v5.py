#!/usr/bin/env python3
"""Gazebo GUI camera follow controller; no vehicle or world modifications."""
import argparse
import csv
import os
from pathlib import Path
import re
import select
import subprocess
import sys
import termios
import tty

TEAM_RE = re.compile(r'team_\d{2}$')
KEY_GRID = {**{str(n):n for n in range(1,10)}, '0':10, '[':11, ']':12}


def read_order(path):
    with open(path, encoding='utf-8-sig',newline='') as f:
        lines=(l for l in f if l.strip() and not l.lstrip().startswith('#'))
        rows=list(csv.reader(lines))
    if rows and [x.strip().lower() for x in rows[0][:2]]==['grid','team']:
        rows=rows[1:]
    result={}
    used=set()
    for row in rows:
        if len(row)!=2: raise ValueError('그리드 파일 형식: grid,team')
        n=int(row[0]); team=row[1].strip()
        if n not in range(1,13) or n in result or team in used or not TEAM_RE.fullmatch(team):
            raise ValueError(f'그리드 중복 또는 팀 이름 오류: {row}')
        result[n]=team; used.add(team)
    return result


def gz(args, timeout=7):
    return subprocess.run(['gz',*args],capture_output=True,text=True,timeout=timeout)


def services():
    p=gz(['service','-l'])
    if p.returncode: raise RuntimeError('Gazebo 서비스 목록 조회 실패: '+p.stderr.strip())
    return set(p.stdout.splitlines())


def service(name,reqtype,request):
    p=gz(['service','-s',name,'--reqtype',reqtype,'--reptype','gz.msgs.Boolean',
          '--timeout','3000','--req',request])
    if p.returncode or not re.search(r'data:\s*true\b',p.stdout):
        raise RuntimeError(f'{name} 오류: {p.stdout.strip()} {p.stderr.strip()}')


def follow(team,view,distance,height,available):
    if not TEAM_RE.fullmatch(team):raise ValueError('팀 ID는 team_01 형식이어야 합니다.')
    if '/gui/follow' not in available:
        raise RuntimeError('Gazebo GUI Camera Tracking의 /gui/follow 서비스가 없습니다. Gazebo GUI 플러그인을 확인하십시오.')
    # Gazebo GUI Follow is based on the scene camera, not an added race sensor.
    # Offset is world-relative for some Gazebo releases; turning-car alignment must be verified.
    dx=-distance if view=='rear' else distance
    if '/gui/follow/offset' in available:
        service('/gui/follow/offset','gz.msgs.Vector3d',f'x: {dx} y: 0 z: {height}')
    service('/gui/follow','gz.msgs.StringMsg',f'data: "{team}"')
    print(f'[FOLLOW] {team} / {"후방" if view=="rear" else "전방"} 관전 (offset x={dx}, z={height})',flush=True)


def interactive(mapping,view,distance,height,available):
    old=None
    if sys.stdin.isatty():
        old=termios.tcgetattr(sys.stdin.fileno()); tty.setcbreak(sys.stdin.fileno())
    current=None
    print('키: 1~9,0=그리드 1~10 / [=11 / ]=12 / R=후방 / F=전방 / Q=종료. 키 입력은 이 터미널에서.',flush=True)
    try:
        while True:
            k=sys.stdin.read(1).lower()
            if not k or k=='q':return
            if k in KEY_GRID:
                n=KEY_GRID[k]
                if n not in mapping:
                    print(f'[GRID {n}] 배정되지 않음',flush=True);continue
                current=mapping[n]
            elif k=='r':view='rear'
            elif k=='f':view='front'
            else:continue
            if current:
                try:follow(current,view,distance,height,available)
                except RuntimeError as e: print('[ERROR]',e,file=sys.stderr,flush=True)
            else:print('먼저 1~0으로 차량을 선택하십시오.',flush=True)
    finally:
        if old is not None:termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,old)


def main(argv=None):
    p=argparse.ArgumentParser(description='URRC F1 Gazebo GUI Follow controller')
    p.add_argument('--order',type=Path,help='그리드 순서 TXT: grid,team')
    p.add_argument('--team',help='직접 선택할 기존 팀 모델명')
    p.add_argument('--grid',type=int,choices=range(1,13),help='그리드 번호 직접 선택 (--order 필요)')
    p.add_argument('--view',choices=['rear','front'],default='rear')
    p.add_argument('--distance',type=float,default=2.5)
    p.add_argument('--height',type=float,default=1.3)
    p.add_argument('--once',action='store_true',help='한 번 설정 후 종료')
    p.add_argument('--check',action='store_true',help='Gazebo GUI 서비스 확인')
    a=p.parse_args(argv)
    try:
        if a.distance<=0 or a.height<=0:raise ValueError('거리와 높이는 양수여야 합니다.')
        mapping=read_order(a.order) if a.order else {}
        if a.team and not TEAM_RE.fullmatch(a.team):raise ValueError('팀 ID 형식: team_01')
        available=services()
        if a.check:
            print('follow:', '/gui/follow' in available, 'offset:', '/gui/follow/offset' in available)
            return 0 if '/gui/follow' in available else 2
        if a.grid is not None:
            if a.grid not in mapping:raise ValueError(f'그리드 {a.grid}번의 참가팀이 없습니다.')
            a.team=mapping[a.grid]
        if a.team:follow(a.team,a.view,a.distance,a.height,available)
        if a.once:return 0 if a.team else 2
        if not mapping:
            if a.team:
                print('그리드 파일이 없어서 키별 팀 선택은 사용할 수 없습니다. --order를 지정하십시오.')
                return 0
            raise ValueError('그리드 TXT --order 또는 --team을 지정하십시오.')
        interactive(mapping,a.view,a.distance,a.height,available)
        return 0
    except (ValueError,RuntimeError,OSError,subprocess.TimeoutExpired) as e:
        print('ERROR:',e,file=sys.stderr);return 2

if __name__=='__main__':sys.exit(main())
