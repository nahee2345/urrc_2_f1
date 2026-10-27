"""ROS sensor_msgs/Image의 실제 RGB 픽셀로 5개 빨간등을 확인한다.

조건: 5개 지정 영역이 연속해서 빨간색 → 동일 영역 5개 모두 어두움.
이미지 대신 /race/start 신호만으로 출발하는 우회는 허용하지 않는다.
"""
from __future__ import annotations
from dataclasses import dataclass,field
from typing import Sequence
import math


def rgb_from_ros(msg):
    """ROS Image를 표준 RGB 바이트열로 변환한다. 인코딩 검사를 생략하지 않는다."""
    width,height,step=int(msg.width),int(msg.height),int(msg.step)
    encoding=str(msg.encoding).lower()
    channels={"rgb8":3,"bgr8":3,"rgba8":4,"bgra8":4}
    if encoding not in channels:
        raise ValueError(f"지원하지 않는 카메라 포맷: {encoding}")
    n=channels[encoding]
    if width<=0 or height<=0 or step<width*n:
        raise ValueError("잘못된 이미지 크기 또는 행 간격")
    raw=bytes(msg.data)
    if len(raw)<step*height:
        raise ValueError("센서 이미지 데이터가 잘렸습니다")
    result=bytearray(width*height*3)
    is_bgr=encoding.startswith("bgr")
    for y in range(height):
        offset=y*step
        for x in range(width):
            k=offset+x*n
            dst=(y*width+x)*3
            if is_bgr:
                result[dst:dst+3]=(raw[k+2],raw[k+1],raw[k])
            else:
                result[dst:dst+3]=raw[k:k+3]
    return width,height,bytes(result)


def roi_red_ratios(width:int,height:int,rgb:bytes,rois:Sequence):
    """정규화된 5개 ROI에서 강한 빨강 픽셀 비율을 계산한다."""
    if len(rois)!=5: raise ValueError("빨간 신호등 ROI는 정확히 5개여야 합니다")
    if len(rgb)!=width*height*3 or width<5 or height<5:
        raise ValueError("잘못된 RGB 프레임")
    out=[]
    for roi in rois:
        if len(roi)!=4 or any(not math.isfinite(float(v)) for v in roi):
            raise ValueError("잘못된 ROI")
        x0,y0,x1,y1=map(float,roi)
        if not 0<=x0<x1<=1 or not 0<=y0<y1<=1:
            raise ValueError("ROI 좌표 범위 오류")
        left=max(0,min(width-1,int(x0*width)))
        right=max(left+1,min(width,int(math.ceil(x1*width))))
        top=max(0,min(height-1,int(y0*height)))
        bottom=max(top+1,min(height,int(math.ceil(y1*height))))
        bright=0;total=0
        for y in range(top,bottom):
            for x in range(left,right):
                k=(y*width+x)*3
                r,g,b=rgb[k:k+3]
                total+=1
                if r>=175 and r>=2*max(g,1) and r>=2*max(b,1):
                    bright+=1
        out.append(bright/total)
    return out


@dataclass
class LightOutGate:
    """전원 재가동·카메라 끊김·허가 철회 시 fail-closed 상태 유지."""
    rois:Sequence
    threshold_on:float=0.10
    threshold_off:float=0.02
    stable_frames:int=3
    max_gap_s:float=0.45
    state:str="WAIT_FIVE_RED"
    count:int=0
    last_time:float|None=None
    released_at:float|None=None
    last_ratios:list=field(default_factory=list)

    def __post_init__(self):
        if len(self.rois)!=5: raise ValueError("ROI 5개 필요")
        if not 0<=self.threshold_off<self.threshold_on<=1:
            raise ValueError("빨강 감지 기준치 오류")
        if self.stable_frames<2 or self.max_gap_s<=0:
            raise ValueError("연속 프레임 또는 시간 간격 오류")

    def reset(self):
        self.state="WAIT_FIVE_RED"
        self.count=0
        self.last_time=None
        self.released_at=None

    def update(self,width,height,rgb,stamp_s,permission):
        # 원본 운영자는 신호등 소등 직전에 /race/start=True를 발행한다.
        # 따라서 False 상태에서도 '5개 빨간불 점등'의 카메라 증거는 수집해야 한다.
        # 다만 False일 때 실제 출발은 절대 허가하지 않는다.
        if not permission and self.state=="RELEASED":
            self.reset()
        if not math.isfinite(stamp_s) or (self.last_time is not None and
            (stamp_s<=self.last_time or stamp_s-self.last_time>self.max_gap_s)):
            self.reset()
            # 시각 역행/큰 공백 이후 첫 프레임만으로 이전 상태 복원 금지
            self.last_time=stamp_s if math.isfinite(stamp_s) else None
            return False
        self.last_time=stamp_s
        self.last_ratios=roi_red_ratios(width,height,rgb,self.rois)
        if self.state=="RELEASED":
            return bool(permission)
        if self.state=="WAIT_FIVE_RED":
            if all(r>=self.threshold_on for r in self.last_ratios):
                self.count+=1
                if self.count>=self.stable_frames:
                    self.state="WAIT_ALL_OFF";self.count=0
            else:self.count=0
        elif self.state=="WAIT_ALL_OFF":
            if permission and all(r<=self.threshold_off for r in self.last_ratios):
                self.count+=1
                if self.count>=self.stable_frames:
                    self.state="RELEASED"
                    self.released_at=stamp_s
                    return True
            else:self.count=0
        return False
