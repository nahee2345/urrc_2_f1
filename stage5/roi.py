"""Monaco 5개 빨간 렌즈 실제 카메라 픽셀에서 ROI를 추출한다.

운영자 신호등을 모두 켠 이미지만 유효. 눈으로 확인하지 않은 4개 이하,
불규칙한 붉은 점 또는 FOV 밖 렌즈는 fail-closed.
"""
from __future__ import annotations
import statistics


def detect_five_red_lenses(width,height,rgb):
    if width<80 or height<60 or len(rgb)!=width*height*3:
        raise ValueError("카메라 이미지가 올바르지 않습니다")
    n=width*height
    mask=bytearray(n)
    for y in range(height):
        for x in range(width):
            idx=y*width+x
            k=idx*3
            r,g,b=rgb[k:k+3]
            if r>=160 and r>max(g*1.8,b*1.8) and g<120 and b<120:
                mask[idx]=1
    seen=bytearray(n)
    blobs=[]
    for k in range(n):
        if not mask[k] or seen[k]:continue
        queue=[k];seen[k]=1
        minx=maxx=k%width;miny=maxy=k//width;total=0
        for at in queue:
            y,x=divmod(at,width)
            total+=1
            minx=min(minx,x);maxx=max(maxx,x)
            miny=min(miny,y);maxy=max(maxy,y)
            for dy,dx in ((0,-1),(0,1),(-1,0),(1,0)):
                xx=x+dx;yy=y+dy
                if not(0<=xx<width and 0<=yy<height):continue
                j=yy*width+xx
                if mask[j] and not seen[j]:
                    seen[j]=1;queue.append(j)
        if total>=2 and maxx-minx+1<=width*.12 and maxy-miny+1<=height*.14:
            blobs.append((minx,miny,maxx,maxy,total))
    # 작은 장면에서는 5개 렌즈만 뚜렷하게 보이도록 조건을 강화.
    if len(blobs)!=5:
        raise ValueError(f"빨간 렌즈 5개가 확인되지 않았습니다: 검출된 그룹 {len(blobs)}개")
    blobs.sort(key=lambda x:x[0])
    centers=[(b[0]+b[2])/2 for b in blobs]
    cy=[(b[1]+b[3])/2 for b in blobs]
    if not all(centers[i+1]-centers[i]>=2 for i in range(4)):
        raise ValueError("빨간등의 수평 배열이 확인되지 않았습니다")
    if max(cy)-min(cy)>max(5,height*.065):
        raise ValueError("빨간등의 수직 높이가 일정하지 않습니다")
    areas=[x[4] for x in blobs]
    if max(areas)>min(areas)*5:
        raise ValueError("렌즈의 크기 편차가 과도합니다")
    rois=[]
    for x0,y0,x1,y1,_ in blobs:
        pad=2
        roi=[max(0,(x0-pad)/width),
             max(0,(y0-pad)/height),
             min(1,(x1+pad+1)/width),
             min(1,(y1+pad+1)/height)]
        if roi[2]-roi[0]<=0 or roi[3]-roi[1]<=0:
            raise ValueError("잘못된 ROI")
        rois.append(roi)
    return rois
