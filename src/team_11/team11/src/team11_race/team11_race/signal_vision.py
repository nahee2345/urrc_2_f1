"""Own front RGB camera used ONLY to recognize the official five red lamps."""
import numpy as np


def red_lamp_count(rgb, encoding="rgb8"):
    """Find separated red luminous columns in the upper 85% of the frame.

    Five detections are required before recognizing the later all-off phase.
    This detector is deliberately conservative: uncertainty means STOP.
    """
    if rgb.ndim!=3 or rgb.shape[2]<3:
        return 0
    image=rgb[::2,::2,:3]
    if encoding.lower().startswith("bgr"):
        r,g,b=image[:,:,2],image[:,:,1],image[:,:,0]
    else:
        r,g,b=image[:,:,0],image[:,:,1],image[:,:,2]
    r=r.astype(np.int16)
    g=g.astype(np.int16)
    b=b.astype(np.int16)
    # Include broad vertical range because the camera is not pre-aimed from
    # any particular official starting grid.
    red=(r>=105)&(r>=g*1.6)&(r>=b*1.6)&((r-g)>=55)
    red=red[:int(red.shape[0]*.85)]
    xscore=red.sum(axis=0)
    active=np.flatnonzero(xscore>=3)
    if active.size==0:return 0
    # Connected red-light columns must be spatially separated.
    gaps=np.where(np.diff(active)>2)[0]+1
    groups=np.split(active,gaps)
    groups=[p for p in groups if p.size>=2]
    # Prevent large red advertising boards / red painted car body surfaces
    # from being mistaken for a light.
    groups=[p for p in groups if p.size<max(20,red.shape[1]//12)]
    if not groups:return 0
    centers=[int(round(float(p.mean()))) for p in groups]
    ypos=[]
    for c in centers:
        yy=np.flatnonzero(red[:,max(0,c-2):min(red.shape[1],c+3)].any(axis=1))
        ypos.append(float(np.median(yy)) if yy.size else 1e6)
    max_aligned=0
    for y in ypos:
        aligned=sum(abs(v-y)<=max(8,red.shape[0]*.05) for v in ypos)
        max_aligned=max(max_aligned,aligned)
    return min(5,max_aligned)


def decode_rgb(frame):
    """Decode sensor_msgs/Image without depending on OpenCV."""
    if frame.encoding.lower() not in ("rgb8","bgr8"):
        return None
    if frame.height<=0 or frame.width<=0 or frame.step<frame.width*3:
        return None
    raw=np.frombuffer(frame.data,dtype=np.uint8)
    if raw.size < frame.height*frame.step:
        return None
    return raw[:frame.height*frame.step].reshape(
        (frame.height,frame.step))[:,:frame.width*3].reshape(
        (frame.height,frame.width,3))


class SignalGate:
    def __init__(self):
        self.seen_five=False
        self.red_frames=0
        self.off_frames=0
        self.green=False
        self.frames=0
        self.last_count=-1

    def reset(self):
        self.__init__()

    def observe(self,frame):
        self.frames+=1
        count=red_lamp_count(frame)
        self.last_count=count
        if self.green:
            return True
        if not self.seen_five:
            self.red_frames=self.red_frames+1 if count==5 else 0
            if self.red_frames>=2:
                self.seen_five=True
                self.off_frames=0
        else:
            self.off_frames=self.off_frames+1 if count==0 else 0
            if self.off_frames>=4:
                self.green=True
        return self.green
