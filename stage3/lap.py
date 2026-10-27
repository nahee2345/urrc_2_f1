"""흰색 출발선 기준 랩 판정. 체크포인트는 유효성 검사에만 사용한다.

이 모듈의 월드 좌표는 운영자 감독에만 사용하며 AI 운전 입력이 아니다.
"""
from __future__ import annotations
import csv
import json
import math
from pathlib import Path


def _signed_distance(sample,gate):
    return ((sample[1]-gate["x"])*math.cos(gate["yaw"])
            +(sample[2]-gate["y"])*math.sin(gate["yaw"]))


def crossing(previous,current,gate):
    """정방향 선분-유한 폭 출발선 교차만 승인한다."""
    d0,d1=_signed_distance(previous,gate),_signed_distance(current,gate)
    if not d0<0<=d1 or current[0]<=previous[0]:return None
    f=-d0/(d1-d0)
    x=previous[1]+(current[1]-previous[1])*f
    y=previous[2]+(current[2]-previous[2])*f
    z=previous[3]+(current[3]-previous[3])*f
    side=-(x-gate["x"])*math.sin(gate["yaw"])+(y-gate["y"])*math.cos(gate["yaw"])
    if abs(side)>gate["half_width"] or abs(z-gate["z"])>gate.get("height_tolerance",0.6):
        return None
    return previous[0]+(current[0]-previous[0])*f


def gates_from_upstream(root:Path,track="monza",spacing_m=10.):
    """원본 centerline/metadata를 읽기만 한다. 원본 Python 파일 수정 금지."""
    if not track.isalnum() or not 0<spacing_m<=100:
        raise ValueError("잘못된 트랙/체크포인트 설정")
    base=Path(root)/"src/urrc_track_gazebo/tracks/processed"
    meta=json.loads((base/(track+".json")).read_text())
    with (base/(track+"_centerline.csv")).open(newline="") as f:
        rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(f)]
    if len(rows)<4:raise ValueError("트랙 경로 점 부족")
    def g(r):return {"x":r["x_m"],"y":r["y_m"],"z":r["z_m"],
                     "yaw":r["yaw_rad"],"half_width":r["road_width_m"]/2+0.15,
                     "height_tolerance":0.6}
    finish={**meta["start_finish"],"half_width":rows[0]["road_width_m"]/2+0.15,
            "height_tolerance":0.6}
    cps=[];distance=0.;next_at=spacing_m
    for a,b in zip(rows,rows[1:]):
        distance+=math.dist((a["x_m"],a["y_m"],a["z_m"]),
                            (b["x_m"],b["y_m"],b["z_m"]))
        if distance>=next_at:
            cps.append(g(b));next_at+=spacing_m
    if cps and distance-(next_at-spacing_m)<spacing_m*.5:cps.pop()
    if len(cps)<3:raise ValueError("경로 검사 지점 부족")
    return finish,cps


class LapSupervisor:
    """출발선 최초 통과=0랩; 다음 정방향 통과마다 검증된 랩 추가."""
    def __init__(self,finish,checkpoints,target=3,max_gap=.05,max_step=3.):
        if target<=0 or not checkpoints:raise ValueError("랩 또는 체크포인트 조건 오류")
        self.finish,self.checkpoints=finish,list(checkpoints)
        self.target,self.max_gap,self.max_step=target,max_gap,max_step
        self.previous=None;self.active=False;self.started_at=None
        self.first_crossing=None;self.last_finish=None
        self.raw_finish_crossings=0;self.laps=0;self.lap_seconds=[]
        self.next_checkpoint=0;self.rejected=0;self.invalid_reasons=[]
        self.collision_count=0;self.finished=False;self.contact_monitor_ready=False

    def arm(self,at):
        if self.previous is None or at<self.previous[0] or at-self.previous[0]>self.max_gap:
            raise ValueError("현재 유효한 물리 차량 위치가 없습니다")
        if _signed_distance(self.previous,self.finish)>=0:
            raise ValueError("출발선 뒤에서만 출발 가능합니다")
        self.active=True;self.started_at=at

    def contact(self,kind,stamp_s):
        """노면·바퀴 접촉은 정상. 장벽/차량 접촉은 실패 기록."""
        if kind in ("wall","vehicle","obstacle","unknown_collision") and self.active:
            self.collision_count+=1;self.invalid_reasons.append(f"충돌:{kind}@{stamp_s:.3f}")
            self.active=False

    def update(self,sample):
        if len(sample)!=4 or any(not math.isfinite(v) for v in sample):
            raise ValueError("유효하지 않은 base_link 계측")
        p=self.previous
        if p is not None:
            gap=sample[0]-p[0]
            if gap<=0:raise ValueError("시간 중복 또는 역행")
            if self.active and (gap>self.max_gap+1e-9 or math.dist(sample[1:],p[1:])>self.max_step):
                self.active=False
                self.invalid_reasons.append("계측 불연속")
                raise ValueError("계측 불연속: 랩 검증을 중단합니다")
        self.previous=sample
        if not self.active or p is None:return []
        events=[]
        # 체크포인트를 지나갔어도 출발선 교차가 아니면 랩수는 증가하지 않는다.
        # 정상적인 점 사이에서는 여러 gate를 순서대로 지나갈 수 있도록 반복.
        last=-math.inf
        for _ in range(len(self.checkpoints)):
            if self.first_crossing is None or self.next_checkpoint>=len(self.checkpoints):
                break
            at=crossing(p,sample,self.checkpoints[self.next_checkpoint])
            if at is None or at<=last:break
            last=at
            events.append({"type":"checkpoint","index":self.next_checkpoint+1,"time":at})
            self.next_checkpoint+=1
        at=crossing(p,sample,self.finish)
        if at is not None:
            self.raw_finish_crossings+=1
            if self.first_crossing is None:
                self.first_crossing=at;self.last_finish=at
                self.next_checkpoint=0
                events.append({"type":"approach","lap":0,"time":at})
            elif self.next_checkpoint==len(self.checkpoints):
                self.laps+=1;self.lap_seconds.append(at-self.last_finish)
                self.last_finish=at;self.next_checkpoint=0
                events.append({"type":"lap","lap":self.laps,"time":at})
                if self.laps>=self.target:self.finished=True;self.active=False
            else:
                self.rejected+=1;self.next_checkpoint=0
                self.last_finish=at
                self.invalid_reasons.append(f"체크포인트 부족 출발선 재통과@{at:.3f}")
                events.append({"type":"rejected_finish","time":at})
        return events

    def status(self):
        return {"status":"PASS_THREE_LAPS" if self.finished and self.collision_count==0 and self.rejected==0 and self.contact_monitor_ready else ("PASS_LAPS_ONLY" if self.finished and self.collision_count==0 and self.rejected==0 else "NOT_PASSED"),
                "target_laps":self.target,"laps":self.laps,
                "finish_crossings":self.raw_finish_crossings,
                "first_crossing_at":self.first_crossing,
                "lap_times_seconds":list(self.lap_seconds),
                "checkpoints_passed_current_lap":self.next_checkpoint,
                "rejected_crossings":self.rejected,
                "collisions":self.collision_count,
                "invalid_reasons":list(self.invalid_reasons)}
