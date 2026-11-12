"""Camera red-lamp candidates from raw RGB8/BGR8 ROS Image bytes.

A conservative lightweight detector: five lamps must appear as separated bright
red clusters on a near-horizontal row in the upper part of the image.
No world state or timing topics used. This function never infers "green".
"""
def count_red_lamps(data, width, height, step, encoding):
    if encoding.lower() not in ("rgb8", "bgr8") or width < 100 or height < 50:
        raise ValueError("Unsupported camera image")
    if step < 3 * width or len(data) < step * height:
        raise ValueError("Truncated camera frame")
    red_index = 0 if encoding.lower() == "rgb8" else 2
    blue_index = 2 if red_index == 0 else 0
    view = memoryview(data)
    # Roof/sky region; avoid red sidewalls on the lower field of view.
    # The gantry is ahead, high in the image. Avoid large red sidewalls.
    xlo,xhi=int(width*.08),int(width*.72)
    ylo,yhi=int(height*.12),int(height*.31)
    ys = [0] * height
    for y in range(ylo,yhi,2):
        base = y*step
        for x in range(xlo,xhi,2):
            i = base + 3*x
            red = view[i+red_index]
            green = view[i+1]
            blue = view[i+blue_index]
            if red >= 180 and green <= 90 and blue <= 90 and red >= green*2:
                ys[y] += 1
    if max(ys[ylo:yhi] or [0]) < 2:
        return distant_lamps(view,width,height,step,red_index,blue_index)
    # Find scanline with greatest count and restrict pixels to 12% of the
    # image height around it to reject unrelated red objects.
    center = max(range(ylo,yhi),key=lambda y:ys[y])
    band = max(8,int(height*.035))
    histogram = [0]*width
    for y in range(max(ylo,center-band),min(yhi,center+band+1),2):
        base=y*step
        for x in range(xlo,xhi,2):
            i=base+3*x
            red,green,blue=view[i+red_index],view[i+1],view[i+blue_index]
            if red>=180 and green<=90 and blue<=90 and red>=2*green:
                histogram[x]+=1
    # Collapse separate narrow bright-red blobs, split by at least one empty
    # two-pixel column. Reject huge spans such as track sidewalls.
    segments=[]
    start=None
    end=None
    for x in range(0,width,2):
        if histogram[x]>=1:
            if start is None:start=x
            end=x
        elif start is not None:
            if 2 <= (end-start+2) <= max(18,int(width*.07)):
                segments.append((start,end))
            start=end=None
    if start is not None and 2 <= (end-start+2) <= max(18,int(width*.07)):
        segments.append((start,end))
    # All bright lamps are in a horizontal row, equally spaced (within
    # perspective tolerance). Require neighboring groups to be separated.
    if len(segments)<4:
        return max(min(3,len(segments)),distant_lamps(view,width,height,step,red_index,blue_index))
    centers=[(lo+hi)/2 for lo,hi in segments]
    # Check the longest valid sequence first. Otherwise a real five-lamp
    # pattern is incorrectly returned as four when its first four match.
    for count in (5, 4):
        for i in range(len(centers) - count + 1):
            sample=centers[i:i+count]
            gaps=[sample[k+1]-sample[k] for k in range(count-1)]
            mean=sum(gaps)/len(gaps)
            if mean>=2 and all(abs(d-mean)<=max(5,mean*.6) for d in gaps):
                return count
    return max(min(3,len(segments)),distant_lamps(view,width,height,step,red_index,blue_index))

def distant_lamps(view,width,height,step,red_index,blue_index):
    """Perspective-tolerant small-lamp row detector for rear grid positions.

    Search image upper-middle, but only recognize 4/5 small red marks
    separated at approximately equal gaps. Broad red crash barriers cannot
    count as lamp marks. Does not depend on track coordinate or grid metadata.
    """
    xlo,xhi=int(width*.22),int(width*.78)
    ylo,yhi=int(height*.07),int(height*.60)
    for y in range(ylo,yhi,2):
        groups=[]
        start=None
        last=None
        for x in range(xlo,xhi,2):
            i=y*step+3*x
            r,g,b=view[i+red_index],view[i+1],view[i+blue_index]
            red=(r>=185 and r>=g*2 and r>=b*2 and g<=100 and b<=100)
            if red:
                if start is None:start=x
                last=x
            elif start is not None:
                if 2<=last-start+2<=max(22,int(width*.035)):
                    groups.append((start+last)/2)
                start=last=None
        if start is not None and 2<=last-start+2<=max(22,int(width*.035)):
            groups.append((start+last)/2)
        for size in (5,4):
            for k in range(len(groups)-size+1):
                points=groups[k:k+size]
                gaps=[points[i+1]-points[i] for i in range(size-1)]
                mean=sum(gaps)/len(gaps)
                if 3<=mean<=width*.11 and all(abs(d-mean)<=max(3,mean*.4) for d in gaps):
                    return size
    return 0
