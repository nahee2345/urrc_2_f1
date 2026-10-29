"""Monaco 출발 신호등: 소등 프레임 대비 새로 켜진 빨간 렌즈 5개만 찾는다.

주변의 빨간 표식/차량·장식 등 정적인 빨간 물체는 차분 마스크에서 제거.
검출 후보가 과다하거나 5등의 배열·크기를 확신할 수 없으면 fail-closed.
기존 무기준 프레임 API도 단위 검사 호환용으로 보존한다.
"""
from __future__ import annotations
from itertools import combinations
import math


def red_components(width, height, rgb, baseline=None):
    if width < 80 or height < 60 or len(rgb) != width*height*3:
        raise ValueError("카메라 이미지가 올바르지 않습니다")
    if baseline is not None and len(baseline) != len(rgb):
        raise ValueError("소등 영상과 점등 영상의 해상도/길이가 다릅니다")
    n = width*height
    mask = bytearray(n)
    for i in range(n):
        k = i*3
        r,g,b = rgb[k:k+3]
        if not (r >= 160 and r > max(g*1.8,b*1.8) and g < 120 and b < 120):
            continue
        if baseline is not None:
            r0,g0,b0 = baseline[k:k+3]
            # 단순한 빨간색이 아니라 교정 도중 실제 밝아진 빨강이어야 한다.
            if r-r0 < 48 or (r-max(g,b))-(r0-max(g0,b0)) < 48:
                continue
        mask[i] = 1
    seen = bytearray(n)
    blobs = []
    for i in range(n):
        if not mask[i] or seen[i]:
            continue
        q = [i]
        seen[i] = 1
        left=right=i%width
        top=bottom=i//width
        for pt in q:
            y,x = divmod(pt,width)
            left=min(left,x);right=max(right,x)
            top=min(top,y);bottom=max(bottom,y)
            for dx,dy in ((0,-1),(0,1),(-1,0),(1,0)):
                xx=x+dx; yy=y+dy
                if not (0<=xx<width and 0<=yy<height):
                    continue
                nxt=yy*width+xx
                if mask[nxt] and not seen[nxt]:
                    seen[nxt]=1;q.append(nxt)
        area=len(q)
        if area>=2 and right-left+1<=width*.12 and bottom-top+1<=height*.14:
            blobs.append((left,top,right,bottom,area))
    return blobs


def five_lens_geometry(blobs, width, height):
    """다섯 원형 램프는 거의 같은 y·크기, 일정한 x 간격으로 일렬 배치."""
    if len(blobs)!=5:
        return None
    ordered=sorted(blobs,key=lambda x:x[0]+x[2])
    xs=[(a[0]+a[2])/2 for a in ordered]
    ys=[(a[1]+a[3])/2 for a in ordered]
    sizes=[a[4] for a in ordered]
    gaps=[xs[i+1]-xs[i] for i in range(4)]
    if min(gaps)<2.:
        return None
    gap=sum(gaps)/4.
    if max(abs(v-gap) for v in gaps) > max(2.5, gap*.47):
        return None
    if max(ys)-min(ys)>max(5.,height*.065):
        return None
    if min(sizes)<2 or max(sizes)>min(sizes)*5.:
        return None
    # 적합도가 같을 때 무작정 잘못된 5개를 고르지 않도록 점수 차 검사.
    score=(max(abs(v-gap) for v in gaps)/max(gap,1.)+
           (max(ys)-min(ys))/max(5.,height*.065)*.5+
           (max(sizes)/min(sizes)-1.)*.08)
    return score,ordered


def detect_five_red_lenses(width,height,rgb,baseline=None):
    blobs=red_components(width,height,rgb,baseline)
    if len(blobs)<5:
        raise ValueError(
            f"빨간 렌즈 5개가 보이지 않습니다: {'소등 대비 점등' if baseline is not None else '단일 화면'} "
            f"검출 후보 {len(blobs)}개")
    if baseline is None and len(blobs)!=5:
        # 소등 비교 없는 기존 모드에서는 빨간 배경 물체를 배제할 증거가 없다.
        raise ValueError(f"소등 영상 없이 빨간 그룹 {len(blobs)}개가 보입니다. 차분 교정이 필요합니다")
    if len(blobs)>18:
        raise ValueError("새로 켜진 빨간 후보가 너무 많아 교정을 거부합니다")
    candidates=[]
    for group in combinations(blobs,5):
        candidate=five_lens_geometry(group,width,height)
        if candidate is not None:
            candidates.append(candidate)
    if not candidates:
        raise ValueError(f"빨간 후보 {len(blobs)}개 중 신호등 5개 일렬 배열을 확인할 수 없습니다")
    candidates.sort(key=lambda x:x[0])
    if len(candidates)>1 and candidates[1][0]-candidates[0][0]<.14:
        raise ValueError(f"빨간 후보 {len(blobs)}개에서 두 개 이상의 5등 배열이 비슷해 교정 거부")
    _,ordered=candidates[0]
    rois=[]
    for left,top,right,bottom,_ in ordered:
        # 렌즈 주변 과도한 배경을 ROI에 포함하지 않도록 1픽셀 여백.
        pad=1
        roi=[max(0,(left-pad)/width),max(0,(top-pad)/height),
             min(1,(right+pad+1)/width),min(1,(bottom+pad+1)/height)]
        if roi[2]<=roi[0] or roi[3]<=roi[1]:
            raise ValueError("빨간 렌즈 ROI 계산 오류")
        rois.append(roi)
    return rois
