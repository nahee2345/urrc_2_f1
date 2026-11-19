#!/usr/bin/env python3
"""v27 independent results, map and grid policy (no ROS, no team-file edits)."""
from __future__ import annotations
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

STAGES=('qualifying','semi_a','semi_b','final')
TRACKS={'monza','hungaroring','monaco','spa','cota','baku'}


def load_json(path:Path)->dict:
    value=json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value,dict):raise ValueError(f'JSON 객체가 아닙니다: {path}')
    return value


def atomic_json(path:Path,value:dict)->None:
    path.parent.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(dir=path.parent,prefix='.'+path.name+'.')
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as f:
            json.dump(value,f,indent=2,ensure_ascii=False);f.write('\n');f.flush();os.fsync(f.fileno())
        os.replace(name,path)
    finally:
        if os.path.exists(name):os.unlink(name)


def validated_maps(root:Path)->dict:
    data=load_json(root/'race/results/v3_maps.json').get('maps')
    if not isinstance(data,dict) or set(data)!=set(STAGES):
        raise ValueError('공식 맵 배정이 없습니다. refresh-maps 실행 후 확인하십시오.')
    if any(t not in TRACKS for t in data.values()) or len(set(data.values()))!=4:
        raise ValueError('예선·본선 A/B·결승 맵은 모두 달라야 합니다.')
    if data['qualifying']=='monaco':raise ValueError('예선 Monaco 제외 규칙 위반')
    if data['semi_a']==data['semi_b']:raise ValueError('본선 A/B 같은 맵 사용 불가')
    if set((data['semi_a'],data['semi_b']))!={'cota','baku'}:
        raise ValueError('본선 A/B는 길이가 유사한 COTA·Baku로 배정하십시오.')
    return data


def stage_track(root:Path,stage:str,requested:str|None)->str:
    assigned=validated_maps(root)[stage]
    if requested is not None and requested!=assigned:
        raise ValueError(f'{stage}: 공식 배정 맵은 {assigned}입니다. --track {requested}로 교체할 수 없습니다.')
    return assigned


def circuit_length(root:Path,track:str)->float:
    """Centerline 3D length from processed CSV, including last->first closure."""
    path=root/'src/urrc_track_gazebo/tracks/processed'/f'{track}_centerline.csv'
    with path.open(encoding='utf-8',newline='') as f:
        rows=csv.DictReader(f)
        points=[]
        for r in rows:
            pt=tuple(float(r[k]) for k in ('x_m','y_m','z_m'))
            if not all(math.isfinite(n) for n in pt):raise ValueError('centerline에 비유한 좌표가 있습니다')
            points.append(pt)
    if len(points)<3:raise ValueError(f'{track}: centerline 표본 부족')
    return sum(math.dist(a,b) for a,b in zip(points,points[1:]+points[:1]))


def complete_rows(root:Path,stage:str)->tuple[dict,list[dict]]:
    payload=load_json(root/'race/results'/f'{stage}.json')
    if payload.get('valid') is not True or payload.get('state')!='complete' or payload.get('provisional'):
        raise ValueError(f'{stage}: 확정된 공식 결과가 아닙니다')
    required_laps=5 if stage=='final' else 3
    if payload.get('laps')!=required_laps:
        raise ValueError(f'{stage}: {required_laps}랩 공식 기록이 아닙니다')
    rows=payload.get('results')
    if not isinstance(rows,list) or any(not isinstance(row,dict) for row in rows):
        raise ValueError(f'{stage}: 순위 목록이 잘못되었습니다')
    if any(row.get('rank_provisional') for row in rows):raise ValueError(f'{stage}: 잠정 순위 존재')
    if any(row.get('status')=='finished' and row.get('laps')!=required_laps for row in rows):
        raise ValueError(f'{stage}: 완료 랩 수가 {required_laps}가 아닌 완주 기록 존재')
    if sorted(row.get('rank') for row in rows)!=list(range(1,len(rows)+1)):
        raise ValueError(f'{stage}: 순위 누락·중복')
    return payload, sorted(rows,key=lambda x:x['rank'])


def finalist_plan(root:Path)->dict:
    maps=validated_maps(root)
    qualifying, q=complete_rows(root,'qualifying')
    if qualifying.get('track')!=maps['qualifying']:
        raise ValueError('예선 결과와 배정된 예선 맵이 다릅니다.')
    # Never advance a disqualified / failed qualifier.
    eligible=[row for row in q if row.get('status')=='finished']
    if len(eligible)<2:raise ValueError('예선 완주자 2명 이상 필요')
    groups=('semi_a',) if len(eligible)<=3 else ('semi_a','semi_b')
    semis={}
    for stage in groups:
        payload,rows=complete_rows(root,stage)
        if payload.get('track')!=maps[stage]:raise ValueError(stage+': 맵 배정과 결과 불일치')
        expected={r['entry_id'] for r in eligible if len(eligible)<=3 and stage=='semi_a' or
                  len(eligible)>3 and r['rank']%2==(1 if stage=='semi_a' else 0)}
        if {r['entry_id'] for r in rows}!=expected:raise ValueError(stage+': 참가팀이 예선 배정과 불일치')
        semis[stage]=(payload,rows)
    lengths={stage:circuit_length(root,maps[stage]) for stage in groups}
    # Manual approval is still required even when the top-three can be calculated.
    common=sum(lengths.values())/len(lengths)
    candidates=[]
    for stage,(payload,rows) in semis.items():
        finished=[r for r in rows if r.get('status')=='finished' and isinstance(r.get('time_seconds'),(int,float))
                  and math.isfinite(r['time_seconds']) and r['time_seconds']>0]
        for r in finished[:3]:
            secs=float(r['time_seconds'])
            candidates.append({'team':r['entry_id'],'group':stage,'group_rank':r['rank'],
               'seconds':secs,'track':maps[stage],'track_length_m':round(lengths[stage],3),
               'normalized_seconds':round(secs*common/lengths[stage],6)})
    if len(candidates)<2:raise ValueError('결승 진출 가능한 본선 완주자 2팀 이상 필요')
    if len(set(c['team'] for c in candidates))!=len(candidates):raise ValueError('중복 결승 참가팀')
    candidates.sort(key=lambda c:(c['normalized_seconds'],c['group_rank'],c['team']))
    result_files=[root/'race/results'/f'{s}.json' for s in ('qualifying',*groups)]
    source_hash={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in result_files}
    return {'version':27,'comparison':'본선 총시간을 같은 3랩 기준 코스 길이로 선형 환산 (참고용 추정)',
       'reference_lap_length_m':round(common,3),'maps':maps,'source_sha256':source_hash,
       'grid':[c['team'] for c in candidates], 'details':candidates}


def approve(root:Path)->dict:
    planned=finalist_plan(root)
    dest=root/'race/results/finalists_v27.json'
    if dest.exists():
        raise ValueError('이미 결승 승인 파일이 있습니다. 덮어쓰지 않습니다.')
    if (root/'race/results/final.json').exists():
        raise ValueError('결승 기록이 이미 있습니다. 진출팀을 변경할 수 없습니다.')
    atomic_json(dest,planned)
    return planned


def approved_grid(root:Path)->list[str]:
    dest=root/'race/results/finalists_v27.json'
    approved=load_json(dest)
    current=finalist_plan(root)
    if approved!=current:raise ValueError('본선 기록·맵 변경 감지. 기존 결승 승인과 불일치')
    return approved['grid']


def ranked_qualifying(root:Path,payload:dict)->Path:
    """Fast-to-slow one-folder official qualifying outputs, including DQ last."""
    folder=root/'race/results/qualifying_ranked'
    folder.mkdir(parents=True,exist_ok=True)
    rows=payload.get('results') or []
    if not isinstance(rows,list):raise ValueError('예선 순위 목록 형식 오류')
    rows=sorted(rows,key=lambda x:x['rank'])
    expected={f'{r["rank"]:02d}_{r["entry_id"]}.json' for r in rows}
    for old in folder.glob('??_team_*.json'):
        if old.name not in expected:old.unlink()
    for r in rows:
        atomic_json(folder/f'{r["rank"]:02d}_{r["entry_id"]}.json',r)
    text=['URRC F1 예선 3랩 순위','상태: '+str(payload.get('state'))+' (전체 참가팀 완료 전 잠정)',
          '순위 | 팀 | 상태 | 완료 랩 | 총시간(초)']
    for r in rows:
        seconds=r.get('time_seconds')
        val=f'{seconds:.3f}' if isinstance(seconds,(int,float)) and math.isfinite(seconds) else '-'
        text.append(f'{r["rank"]} | {r["entry_id"]} | {r["status"]} | {r.get("laps",0)} | {val}')
    (folder/'rankings.txt').write_text('\n'.join(text)+'\n',encoding='utf-8')
    with (folder/'rankings.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['rank','team','status','laps','seconds'])
        for r in rows:w.writerow([r['rank'],r['entry_id'],r.get('status'),r.get('laps'),r.get('time_seconds')])
    return folder
