"""AI-01 실제 RGB 이미지의 Monaco 도로 중심 추정 기준선.

도로의 절대 RGB 색상을 단일 값으로 가정하지 않는다. 그림자 아래 어두운
중립색 아스팔트를 인식하고 녹갈색 바깥 지형은 색 편차로 배제한다.
차체 위치/센터라인/운영자 pose는 절대로 참조하지 않는다.
가까운 화면의 연결된 도로 구간과 내부 양측 경계를 확인하지 못하면 안전상 정지한다.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from stage3.vision import rgb_from_ros


@dataclass(frozen=True)
class LaneEstimate:
    valid:bool
    error:float=0.0
    confidence:float=0.0
    samples:int=0
    reason:str=""


def _asphalt(r,g,b):
    """어두운 중립색 아스팔트와 녹갈색 노면을 구별한다.

    Monaco의 조명·고저차·그림자 영향을 고려하여 Monza 초기의 17~67
    고정 밝기 범위보다 넓게 잡되, 녹색/갈색 물체는 blue 채널로 거른다.
    """
    brightness=(int(r)+int(g)+int(b))/3.
    spread=max(r,g,b)-min(r,g,b)
    return (8<=brightness<=112 and
            spread<=max(16.,brightness*.28) and
            b>=r-9 and b>=g-9)


def _segments(flags,min_run=4):
    start=None
    for i,flag in enumerate(list(flags)+[False]):
        if flag and start is None:
            start=i
        if not flag and start is not None:
            if i-start>=min_run:
                yield start,i
            start=None


def _row_segments(width,height,rgb,y):
    """3개 인접 행의 다수결로 그림자/흰 가로선 노이즈 완화."""
    flags=[]
    for x in range(width):
        yes=0
        for dy in (-1,0,1):
            py=max(0,min(height-1,y+dy))
            offset=(py*width+x)*3
            if _asphalt(*rgb[offset:offset+3]):
                yes+=1
        flags.append(yes>=2)
    return list(_segments(flags,max(3,width//60)))


def estimate_lane(width:int,height:int,rgb:bytes,prior:float=0.):
    if width<40 or height<30 or len(rgb)!=width*height*3:
        raise ValueError("잘못된 카메라 영상 크기")
    if not math.isfinite(prior) or not -0.99<=prior<=0.99:
        raise ValueError("잘못된 이전 도로 중심")

    # 가까운 바닥부터 도로를 따라 원거리로 연결. 급커브에서 먼 행을 먼저
    # 탐색하면 가장 가까운 다른 도로 구간/장벽 그림자를 잘못 고를 수 있다.
    # 실제 Monaco RGB 원본 프레임에서 고저차로 차도가 화면 77~85% 구간에
    # 좁게 보이는 상황을 확인. 기존 7개 희소 행은 모두 지나쳐 검출 실패했음.
    # 실제 Monaco 실측: 하단 시야에서 도로 폭이 90%를 넘으며
    # 기존 16행 스캔은 정지 영상의 유효 행 3개만 남겼음.
    rows=tuple(.95-.01*k for k in range(34))
    anchor=max(width*.06,min(width*.94,(prior+1.)*width/2.))
    centers=[]
    near_count=0
    bounded_count=0
    previous_selected_row=None
    longest_streak=0
    streak=0
    for k,frac in enumerate(rows):
        y=min(height-1,int(height*frac))
        candidates=[]
        for left,right in _row_segments(width,height,rgb,y):
            span=right-left
            midpoint=(left+right)/2.
            # 차로 전체가 화면을 채우면 경계를 볼 수 없어 신뢰 불가.
            # 도로가 화면 끝에서 잘려 폭이 90~97%인 구간도 허용.
            # 단색 화면(폭 100%)은 도로로 인정하지 않음.
            if not width*.055<=span<=width*.97:
                continue
            difference=abs(midpoint-anchor)
            if difference>width*.42:
                continue
            # 작은 검정 그림자보다 폭이 확보된 도로 구간 우선.
            cost=difference/width-.12*min(1.,span/(width*.26))
            candidates.append((cost,midpoint,span,left>0 and right<width))
        if not candidates:
            continue
        _,midpoint,span,bounded=min(candidates,key=lambda item:item[0])
        if centers and abs(midpoint-centers[-1][0])>width*.34:
            continue
        if frac>=.79:
            near_count+=1
        if bounded:
            bounded_count+=1
        # 도로가 몇 개의 연속 영상 행에 걸쳐 보이지 않으면 작은 그림자로 간주.
        if previous_selected_row is not None and k==previous_selected_row+1:
            streak+=1
        else:
            streak=1
        longest_streak=max(longest_streak,streak)
        previous_selected_row=k
        # 먼 행을 약간 강조해 Monaco 코너 접근 전에 선회.
        weight=(1+.03*k)*min(1.,span/(width*.23))
        centers.append((midpoint,weight,span))
        anchor=.57*anchor+.43*midpoint

    if len(centers)<4 or near_count<2 or longest_streak<3 or bounded_count<2:
        return LaneEstimate(False,samples=len(centers),
                            reason="연속 도로 경계 부족")
    total=sum(item[1] for item in centers)
    if total<=0:
        return LaneEstimate(False,samples=len(centers),reason="도로 가중치 부족")
    average=sum(c*w for c,w,_ in centers)/total
    # 부분 시야에서 5~6개 인접 행만 보이는 것이 실제 정상 Monaco 도로.
    # 전체 화면에서 보이지 않는 하늘 행까지 나눠 신뢰도를 낮추지 않는다.
    coverage=min(1.,len(centers)/6.)
    width_score=sum(min(1.,span/(width*.23)) for _,_,span in centers)/len(centers)
    confidence=coverage*width_score
    error=max(-1.,min(1.,(average-width/2.)/(width/2.)))
    if confidence<.32:
        return LaneEstimate(False,error=error,confidence=confidence,
                            samples=len(centers),reason="도로 분할 신뢰도 부족")
    return LaneEstimate(True,error=error,confidence=confidence,
                        samples=len(centers),reason="실제 RGB 연속 도로 구간 추정")


def estimate_ros(msg,prior=0.):
    width,height,rgb=rgb_from_ros(msg)
    return estimate_lane(width,height,rgb,prior)
