#!/usr/bin/env python3
"""Operator inspection matrix for team_01..team_11. Static checks separate from Gazebo live proof."""
from __future__ import annotations
import argparse,csv,json,math,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path

TEAMS=tuple('team_%02d'%i for i in range(1,12))
LIMIT=(0.55,0.25,0.18)


def conservative_envelope(model:Path):
    """Known axis-aligned primitives: detect out-of-spec combined footprint.

    Mesh geometry, sensor extrusions, rotations, and articulated URDF cannot
    be certified by a static AABB. They remain REVIEW for manual inspection.
    """
    from base_link_check_v6 import extents,xyz
    doc=ET.parse(model).getroot()
    if doc.tag=='robot':return dict(status='REVIEW',reason='URDF의 관절·링크 변환/센서 돌출은 수동 검증 필요')
    sdf=doc if doc.tag=='model' else doc.find('model')
    if sdf is None:return dict(status='REVIEW',reason='SDF model 없음')
    boxes=[];uncertain=False
    for link in sdf.findall('link'):
        pose=link.findtext('pose') or ''
        components=[float(v) for v in pose.split()]
        if any(abs(v)>1e-6 for v in components[3:]):uncertain=True
        origin=xyz(pose)
        for item in list(link.findall('collision'))+list(link.findall('visual')):
            itempose=item.findtext('pose') or ''
            vals=[float(v) for v in itempose.split()]
            if any(abs(v)>1e-6 for v in vals[3:]):uncertain=True
            ex=extents(item.find('geometry'))
            if ex is None:
                uncertain=True
                continue
            center=xyz(itempose)
            boxes.append(([origin[i]+center[i] for i in range(3)],ex))
        if link.findall('sensor'):uncertain=True
    if not boxes:return dict(status='REVIEW',reason='측정 가능한 collision/visual 기본 도형 없음')
    lower=[min(p[i]-ex[i]/2 for p,ex in boxes) for i in range(3)]
    upper=[max(p[i]+ex[i]/2 for p,ex in boxes) for i in range(3)]
    size=[round(upper[i]-lower[i],5) for i in range(3)]
    if not uncertain and any(size[i]>LIMIT[i]+1e-6 for i in range(3)):
        return dict(status='FAIL',size_m=size,reason='전체 기본 형상 외접 크기가 규격 초과')
    if uncertain:return dict(status='REVIEW',size_m=size,reason='메시·회전·센서/부속물 미확인: 수동 전체 외형 검사 필수')
    return dict(status='PARTIAL_PASS',size_m=size,reason='기본 도형 외접 크기 검증. 센서 돌출 수동 확인')

def inspect(root:Path,live:bool=False,track:str='monza',seconds:int=120):
    sys.path.insert(0,str(root/'race/scripts'))
    from vehicle_preflight_v16 import check_entries
    reports=check_entries(root,conversion=True)
    by_team={r['team']:r for r in reports}
    result=[]
    for team in TEAMS:
        r=by_team.get(team) or dict(team=team,status='MISSING',errors=[],warnings=[],checks={})
        status=r['status']
        checks=r.get('checks') or {}
        base=checks.get('base_link') or {}
        center=base.get('center_m')
        if isinstance(center,list) and len(center)==3 and any(abs(v)>0.01 for v in center):
            status='FAIL'
            r['errors'].append('base_link와 collision 외접 중심 3D 오차 1cm 초과: '+str(center))
        # Do not mistake no automatic dimension errors for a proven full model envelope.
        shape=dict(status='NOT_SUBMITTED')
        if status not in ('EMPTY','MISSING'):
            try:
                manifest=json.loads((root/'src/team'/team/'race_entry.json').read_text(encoding='utf-8'))
                shape=conservative_envelope(root/'src/team'/team/manifest['vehicle_model_file'])
                if shape['status']=='FAIL':
                    status='FAIL'
                    r['errors'].append(shape['reason']+' '+str(shape.get('size_m')))
            except (OSError,ValueError,KeyError,ET.ParseError) as exc:
                shape={'status':'REVIEW','reason':str(exc)}
        row=dict(team=team,static=status,base_link=base.get('status','UNVERIFIED'),
                 dimensions=shape['status'],measured_envelope_m=shape.get('size_m'),
                 dimensions_note=shape.get('reason'),
                 spawn='NOT_TESTED',camera='NOT_TESTED',lidar='NOT_TESTED',timing='NOT_TESTED',
                 errors=r['errors'],warnings=r['warnings'])
        if live and status not in ('EMPTY','MISSING','FAIL'):
            cmd=['bash',str(root/'race_v24.sh'),'compatibility','--team',team,
                 '--tracks',track,'--live','--seconds',str(seconds),'--require-synced']
            print('[실주행 검증]',team,track,flush=True)
            ret=subprocess.run(cmd,cwd=root,check=False).returncode
            row['spawn']=row['camera']=row['lidar']=row['timing']='OBSERVED_TEST_PASS' if ret==0 else 'FAIL_OR_INCOMPLETE'
            row['live_note']='호환성 검사 전체 성공 여부이며 항목별 개별 인증은 해당 검사 보고서 확인 필요'
        result.append(row)
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description='team_01~team_11 차량 전체 정적 검사, 선택적 Gazebo 실제 주행 검증')
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    p.add_argument('--live',action='store_true',help='팀별 Gazebo 모의주행 검사(실제로 모델 스폰)')
    p.add_argument('--track',choices=['monza','hungaroring','monaco','spa','cota','baku'],default='monza')
    p.add_argument('--seconds',type=int,default=120)
    x=p.parse_args(argv);root=x.root.expanduser().resolve()
    try:
        rows=inspect(root,x.live,x.track,x.seconds)
        dest=root/'race/results/inspections';dest.mkdir(parents=True,exist_ok=True)
        (dest/'v27_teams_01_11.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        with (dest/'v27_teams_01_11.csv').open('w',encoding='utf-8',newline='') as f:
            w=csv.DictWriter(f,fieldnames=['team','static','base_link','dimensions','spawn','camera','lidar','timing'],extrasaction='ignore');w.writeheader();w.writerows(rows)
        print('\n팀      | 정적 | base_link | 크기 | 스폰 | 센서·계측')
        for r in rows:
            print(f'{r["team"]:<8}| {r["static"]:<10}| {r["base_link"]:<16}| {r["dimensions"]:<17}| {r["spawn"]:<12}| {r["timing"]}')
        print('보고서: race/results/inspections/v27_teams_01_11.csv, .json')
        print('정적 검사는 실제 크기·스폰·센서 연결 성공을 보증하지 않습니다.')
        return 2 if any(r['static'] in ('FAIL','EMPTY','MISSING') or (x.live and r['spawn']=='FAIL_OR_INCOMPLETE') for r in rows) else 0
    except (OSError,ValueError,ImportError,KeyError) as exc:
        print('[v27 검진 오류]',exc,file=sys.stderr);return 2

if __name__=='__main__':raise SystemExit(main())
