#!/usr/bin/env python3
"""URRC independent read-only LAN spectator. Reads base_link audit logs only.

The HTTP server contains no Gazebo/ROS transport, control publishers, shell
commands, or writing endpoints. Never modifies racing/team files.
"""
from __future__ import annotations
import argparse
import csv
import hmac
import json
import math
import os
from pathlib import Path
import re
import secrets
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

STAGES = {'qualifying', 'semi_a', 'semi_b', 'final'}
TRACKS = {'monza', 'hungaroring', 'monaco', 'spa', 'cota', 'baku'}
TEAM_RE = re.compile(r'team_(?:0[1-9]|1[0-2])\Z')
MAX_READ = 8 * 1024 * 1024


def safe_json(path: Path) -> dict:
    try:
        data=json.loads(path.read_text(encoding='utf-8'))
        return data if isinstance(data,dict) else {}
    except (OSError,ValueError,UnicodeError):
        return {}


def active_teams(session: dict, poses: dict) -> list[str]:
    """Only spawned, measured, not DNF/finished teams exist in the track."""
    statuses=session.get('status', {})
    if not isinstance(statuses,dict):return []
    return [team for team in session.get('teams', [])
            if isinstance(team,str) and TEAM_RE.fullmatch(team)
            and statuses.get(team) in ('SPAWNED','RUNNING') and team in poses]


def rank_active(active: list[str], events: list[dict], results: list[dict]) -> list[dict]:
    """Read-only sorting by real gate crossings; fall back to grid for ties.

    Same sampled checkpoints cannot prove exact instantaneous order. Ranks are
    explicitly provisional and are never used to change official results.
    """
    byteam = {t:{'team':t,'crossings':0,'last_crossing':math.inf,'laps':0,
                 'status':'RUNNING','grid':i+1} for i,t in enumerate(active)}
    for ev in events:
        t=ev.get('entry_id')
        if ev.get('type')!='crossing' or t not in byteam:continue
        stamp=ev.get('time')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp):continue
        byteam[t]['crossings']+=1
        byteam[t]['last_crossing']=float(stamp)
        if ev.get('gate')==0:
            byteam[t]['laps']=max(0,byteam[t]['laps']+(byteam[t]['crossings']>1))
    # The active grid (operator session teams) already has grid order, but sorting
    # by completed checkpoints is reliable enough for provisional camera switching.
    rows=sorted(byteam.values(),key=lambda x:(-x['crossings'],x['last_crossing'],x['grid']))
    for idx,row in enumerate(rows,1):row['rank']=idx
    return rows


class RaceData:
    def __init__(self,root:Path):
        self.root=root.resolve()
        self.lock=threading.Lock()
        self.audit_path=None
        self.offset=0
        self.partial=b''
        self.poses={}
        self.events=[]
        self.prev={}
        self.tracks={}

    def _reset(self,path):
        self.audit_path=path
        self.offset=0
        self.partial=b''
        self.poses={}
        self.events=[]
        self.prev={}

    def _pull(self,path:Path):
        if not path.is_file():return
        try:
            with path.open('rb') as f:
                f.seek(0,2); size=f.tell()
                if size<self.offset:self._reset(path)
                f.seek(self.offset)
                # Drain incremental log without ever loading unbounded history.
                data=f.read(MAX_READ)
                self.offset=f.tell()
            parts=(self.partial+data).split(b'\n')
            self.partial=parts.pop()[-4096:]
            for ln in parts:
                if len(ln)>4096:continue
                try:ev=json.loads(ln)
                except (UnicodeError,ValueError):continue
                typ=ev.get('type')
                team=ev.get('entry_id')
                if not isinstance(team,str) or not TEAM_RE.fullmatch(team):continue
                if typ=='pose':
                    sample=ev.get('sample')
                    if not isinstance(sample,list) or len(sample)<4:continue
                    if any(not isinstance(x,(float,int)) or not math.isfinite(x) for x in sample[:4]):continue
                    stamp,x,y,z=[float(x) for x in sample[:4]]
                    old=self.poses.get(team)
                    heading=old.get('heading') if old else None
                    if old is not None and (x-old['x'])**2+(y-old['y'])**2 > 0.0004:
                        heading=math.atan2(y-old['y'],x-old['x'])
                    self.poses[team]={'team':team,'sim_time':stamp,'x':x,'y':y,'z':z,
                                      'heading':heading,'received':time.monotonic()}
                elif typ=='crossing':
                    self.events.append(ev)
                elif typ=='arm':
                    self.events.append({'type':'arm','entry_id':team})
            # A single race should stay well under this limit; preserve latest
            # checkpoint history for current lap ranks in very long events.
            if len(self.events)>150000:self.events=self.events[-120000:]
        except OSError:return

    def state(self) -> dict:
        with self.lock:
            info=safe_json(self.root/'race/results/operator_v26_active.json')
            stage=info.get('stage')
            track=info.get('track')
            if stage not in STAGES or track not in TRACKS:
                self._reset(None)
                return {'state':'waiting','message':'진행 중인 공식 경기가 없습니다','teams':[],'leader':None}
            # Reject stale session if it is not an active race controller.
            try:
                os.kill(int(info.get('pid')),0)
            except (OSError,ValueError,TypeError):
                self._reset(None)
                return {'state':'waiting','message':'경기 종료 또는 운영자 프로세스 중지','teams':[],'leader':None}
            payload=safe_json(self.root/f'race/results/{stage}.json')
            audit=payload.get('audit_directory')
            path=None
            if isinstance(audit,str) and audit:
                base=(self.root/'race/results/runs').resolve()
                candidate=(self.root/audit).resolve()
                if candidate.is_relative_to(base):path=candidate/'timing.jsonl'
            if path!=self.audit_path:self._reset(path)
            if path is not None:self._pull(path)
            valid_teams=[t for t in info.get('teams',[]) if isinstance(t,str) and TEAM_RE.fullmatch(t)]
            # Only expose recently observed poses for actively spawned cars.
            allowed={t:self.poses[t] for t in valid_teams if t in self.poses}
            live=active_teams(info,allowed)
            ranked=rank_active(live,self.events,payload.get('results',[]))
            ranks={x['team']:x['rank'] for x in ranked}
            statuses=info.get('status', {})
            result_by_team={x.get('entry_id'):x for x in payload.get('results',[]) if isinstance(x,dict)}
            rows=[]
            for team in valid_teams:
                row={'team':team,'status':str(statuses.get(team,'WAITING')),
                     'rank':ranks.get(team),'available':team in live,
                     'pose':None}
                if team in allowed:
                    pos={k:v for k,v in allowed[team].items() if k not in ('received','team')}
                    row['pose']=pos
                if team in result_by_team:
                    row['laps']=result_by_team[team].get('laps',0)
                rows.append(row)
            leader=ranked[0]['team'] if ranked else None
            return {'state':'running' if live else 'waiting_vehicles',
                    'stage':stage,'track':track,'leader':leader,'teams':rows,
                    'ranks_provisional':True,
                    'note':'모든 순위는 base_link 체크포인트 기준 잠정 표시입니다.'}

    def track(self,name:str) -> dict:
        if name not in TRACKS:raise ValueError('지원하지 않는 맵')
        with self.lock:
            if name in self.tracks:return self.tracks[name]
            path=self.root/f'src/urrc_track_gazebo/tracks/processed/{name}_centerline.csv'
            rows=[]
            with path.open('r',encoding='utf-8',newline='') as f:
                for row in csv.DictReader(f):
                    try:
                        x,y,z,w=(float(row[k]) for k in ('x_m','y_m','z_m','road_width_m'))
                    except (ValueError,KeyError):continue
                    if all(math.isfinite(v) for v in (x,y,z,w)) and 0<w<100:
                        rows.append([round(x,3),round(y,3),round(z,3),round(w,2)])
                    if len(rows)>100000:break
            if not rows:raise ValueError('트랙 좌표가 없습니다')
            step=max(1,len(rows)//2400)
            sampled=rows[::step]
            if sampled[-1]!=rows[-1]:sampled.append(rows[-1])
            self.tracks[name]={'track':name,'centerline':sampled}
            return self.tracks[name]


class WebServer(ThreadingHTTPServer):
    daemon_threads=True
    allow_reuse_address=True
    def __init__(self,addr,handler,root,token):
        super().__init__(addr,handler)
        self.state_source=RaceData(root)
        self.token=token
        self.index=(Path(__file__).resolve().parent.parent/'spectator/index.html').read_bytes()


class Handler(BaseHTTPRequestHandler):
    server_version='URRCReadOnlySpectator'
    def log_message(self,format,*args):
        # Never log spectator access tokens.
        sys.stderr.write('[spectator] request '+str(self.command)+' (URL omitted)\n')

    def reply(self,code,body:bytes,kind='application/json; charset=utf-8'):
        self.send_response(code)
        self.send_header('Content-Type',kind)
        self.send_header('Content-Length',str(len(body)))
        self.send_header('Cache-Control','no-store')
        self.send_header('Referrer-Policy','no-referrer')
        self.send_header('X-Content-Type-Options','nosniff')
        self.send_header('X-Frame-Options','DENY')
        self.send_header('Content-Security-Policy',"default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; connect-src 'self'; img-src 'self' data:; base-uri 'none'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url=urlsplit(self.path)
        keys=parse_qs(url.query).get('key',[])
        # Constant-time authorization for GET, no mutation possible.
        if not keys or not hmac.compare_digest(keys[0],self.server.token):
            self.reply(403,b'{"error":"invalid spectator access key"}')
            return
        if url.path in ('/','/index.html'):
            self.reply(200,self.server.index,'text/html; charset=utf-8')
        elif url.path=='/api/state':
            self.reply(200,json.dumps(self.server.state_source.state(),ensure_ascii=False,allow_nan=False).encode('utf-8'))
        elif url.path=='/api/track':
            name=parse_qs(url.query).get('track',[''])[0]
            try:payload=self.server.state_source.track(name)
            except (ValueError,OSError):
                self.reply(404,b'{"error":"unknown track"}')
                return
            self.reply(200,json.dumps(payload,ensure_ascii=False).encode('utf-8'))
        else:
            self.reply(404,b'{"error":"not found"}')

    def do_POST(self):self.reply(405,b'{"error":"read only"}')
    def do_PUT(self):self.reply(405,b'{"error":"read only"}')
    def do_DELETE(self):self.reply(405,b'{"error":"read only"}')
    def do_PATCH(self):self.reply(405,b'{"error":"read only"}')


def guess_ip():
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            s.connect(('192.0.2.1',9));return s.getsockname()[0]
    except OSError:return '대회장PC_IP'


def main(argv=None):
    p=argparse.ArgumentParser(description='URRC 보기 전용 LAN Follow/자유 관전 서버')
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    p.add_argument('--host',default='0.0.0.0')
    p.add_argument('--port',type=int,default=8765)
    p.add_argument('--key',default=None,help='관전용 키 (기본: 매 실행마다 임의 생성)')
    a=p.parse_args(argv)
    root=a.root.expanduser().resolve()
    if safe_json(root/'race/v24_profile.json').get('mode')!='official':
        p.error('공식 urrc_22 배포본에만 설치하십시오')
    if not 1024<=a.port<=65535:p.error('port는 1024~65535')
    key=a.key or secrets.token_urlsafe(20)
    if len(key)<12:p.error('관전 키는 최소 12자')
    try:
        server=WebServer((a.host,a.port),Handler,root,key)
    except OSError as exc:
        p.error('서버 실행 실패: '+str(exc))
    print('[관전] 관전 전용 서버: 맵·차량·ROS 조작 API 없음',flush=True)
    print('[관전] 아래 링크를 같은 Wi-Fi 참가자에게 전달',flush=True)
    print(f'http://{guess_ip()}:{a.port}/?key={key}',flush=True)
    print('[관전] 이 서버를 이용하더라도 Gazebo/ROS 제어 포트는 관전 Wi-Fi에서 따로 차단해야 합니다.',flush=True)
    try:server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:pass
    finally:server.server_close()
    return 0

if __name__=='__main__':raise SystemExit(main())
