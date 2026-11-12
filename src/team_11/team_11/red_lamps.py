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
    ylo, yhi = int(height * .05), int(height * .65)
    xs = [0] * width
    ys = [0] * height
    for y in range(ylo,yhi,2):
        base = y*step
        for x in range(0,width,2):
            i = base + 3*x
            red = view[i+red_index]
            green = view[i+1]
            blue = view[i+blue_index]
            if red >= 180 and green <= 90 and blue <= 90 and red >= green*2:
                xs[x] += 1
                ys[y] += 1
    if max(ys[ylo:yhi] or [0]) < 2:
        return 0
    # Find scanline with greatest count and restrict pixels to 12% of the
    # image height around it to reject unrelated red objects.
    center = max(range(ylo,yhi),key=lambda y:ys[y])
    band = max(8,int(height*.035))
    histogram = [0]*width
    for y in range(max(ylo,center-band),min(yhi,center+band+1),2):
        base=y*step
        for x in range(0,width,2):
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
        return min(5,len(segments))
    centers=[(lo+hi)/2 for lo,hi in segments]
    for i in range(len(centers)):
        for j in range(i+3,min(len(centers),i+5)):
            sample=centers[i:j+1]
            if not (4 <= len(sample) <= 5):continue
            gaps=[sample[k+1]-sample[k] for k in range(len(sample)-1)]
            mean=sum(gaps)/len(gaps)
            if mean>=2 and all(abs(d-mean)<=max(5,mean*.6) for d in gaps):
                return len(sample)
    return min(3,len(segments))
