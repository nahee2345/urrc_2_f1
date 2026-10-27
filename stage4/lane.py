"""AI-01: 오직 주행용 실제 RGB 이미지에서 차도 중심을 찾는 경량 기준선.

원본 몬자 v10.8 아스팔트(짙은 회색)와 주변 녹갈색 노면의
색·명암 차이를 이용하며, 시뮬레이터 정답 pose/centerline을 참조하지 않는다.
실제 카메라에서 교정/재검증 전에는 안정적 완주 성능을 보장하지 않는다.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from stage3.vision import rgb_from_ros


@dataclass(frozen=True)
class LaneEstimate:
    valid:bool
    error:float=0.0     # +오른쪽, -왼쪽, 중심 기준
    confidence:float=0.0
    samples:int=0
    reason:str=""

def _asphalt(r,g,b):
    """검증 대상 원본 v10.8 기본 아스팔트 텍스처용 보수적 픽셀 마스크."""
    value=(int(r)+int(g)+int(b))/3
    return (17<=value<=67 and abs(int(r)-int(g))<=23
            and abs(int(r)-int(b))<=24 and abs(int(g)-int(b))<=22)

def _segments(flags,min_run=4):
    a=None
    for i,flag in enumerate(list(flags)+[False]):
        if flag and a is None:a=i
        if not flag and a is not None:
            if i-a>=min_run:yield a,i
            a=None

def estimate_lane(width:int,height:int,rgb:bytes,prior:float=0.0):
    if width<40 or height<30 or len(rgb)!=width*height*3:
        raise ValueError("잘못된 카메라 영상 크기")
    if not math.isfinite(prior) or not -.99<=prior<=.99:
        raise ValueError("잘못된 이전 도로 중심")
    center=(prior+1)*width/2
    estimates=[]
    # 수평형 카메라는 화면 상단 대부분이 하늘이므로 하단 중심부만 사용.
    for f in (.62,.69,.76,.83,.90):
        y=min(height-1,int(height*f))
        # 위/아래 인접 행의 노이즈를 다수결로 완화.
        hit=[]
        for x in range(width):
            votes=0
            for dy in (-1,0,1):
                py=min(height-1,max(0,y+dy));i=(py*width+x)*3
                if _asphalt(*rgb[i:i+3]):votes+=1
            hit.append(votes>=2)
        options=list(_segments(hit,min_run=max(4,width//35)))
        if not options:continue
        # 직전 중심에 가장 가까운 도로 구간. 거리가 지나치게 멀면 추적 실패.
        start,end=min(options,key=lambda seg:(abs((seg[0]+seg[1])/2-center),
                                             -(seg[1]-seg[0])))
        span=end-start
        if span<width*.07 or abs((start+end)/2-center)>width*.39:
            continue
        measured=(start+end)/2
        row_score=min(1.0,span/(width*.24))
        estimates.append((measured,row_score))
        center=.64*center+.36*measured
    if len(estimates)<3:
        return LaneEstimate(False,samples=len(estimates),reason="연속 도로 경계 부족")
    average=sum(x*score for x,score in estimates)/sum(score for _,score in estimates)
    confidence=min(1.0,len(estimates)/5)*sum(z for _,z in estimates)/len(estimates)
    error=max(-1.,min(1.,(average-width/2)/(width/2)))
    if confidence<.32:
        return LaneEstimate(False,error=error,confidence=confidence,
                            samples=len(estimates),reason="도로 분할 신뢰도 부족")
    return LaneEstimate(True,error=error,confidence=confidence,
                        samples=len(estimates),reason="카메라 기반 도로 구간 추정")


def estimate_ros(msg,prior=0.):
    width,height,rgb=rgb_from_ros(msg)
    return estimate_lane(width,height,rgb,prior)
