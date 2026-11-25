"""Five-red -> verified background return. No signal topic, timer, or pose input."""
from collections import deque
import itertools
import cv2
import numpy as np

class FiveRedDetector:
    def __init__(self):
        self.reset()
    def reset(self):
        self.history=deque(maxlen=100)
        self.boxes=None
        self.reference=None
        self.context=None
        self.all_red_count=0
        self.off_count=0
        self.released=False
        self.reason='WAIT_FIVE_RED'
        self.red_count=0
    @staticmethod
    def red_mask(rgb):
        a=rgb.astype(np.int16)
        return ((a[:,:,0]>120)&(a[:,:,0]>a[:,:,1]*2+35)&(a[:,:,0]>a[:,:,2]*2+35)).astype(np.uint8)*255
    @staticmethod
    def group(mask):
        n,_,stats,centers=cv2.connectedComponentsWithStats(mask)
        candidates=[]
        for i in range(1,n):
            x,y,w,h,area=stats[i]
            if 5<=w<=70 and 6<=h<=70 and area>=30 and .35<w/h<2.0:
                candidates.append((x,y,w,h))
        for group in itertools.combinations(candidates[:30],5):
            boxes=sorted(group)
            x=np.array([b[0]+b[2]/2 for b in boxes]);y=np.array([b[1]+b[3]/2 for b in boxes])
            sizes=np.array([b[2:] for b in boxes]);spacing=np.diff(x)
            if (np.ptp(y)<max(4,np.mean(sizes[:,1])*.30) and np.min(spacing)>np.mean(sizes[:,0])*1.12
                and np.std(spacing)<np.mean(spacing)*.12 and np.max(sizes)/np.min(sizes)<2.0):
                return boxes
        return None
    def update(self,rgb):
        if self.released:return True,'RELEASED'
        mask=self.red_mask(rgb)
        if self.boxes is None:
            boxes=self.group(mask)
            if boxes is None:
                self.history.append(rgb.copy())
                return False,'WAIT_FIVE_RED'
            x0=max(0,boxes[0][0]-18);x1=min(rgb.shape[1],boxes[-1][0]+boxes[-1][2]+18)
            y0=max(0,min(b[1] for b in boxes)-16);y1=min(rgb.shape[0],max(b[1]+b[3] for b in boxes)+16)
            # Find a recent, genuinely all-dark baseline at the same five lens sites.
            for previous in reversed(self.history):
                old=self.red_mask(previous)
                if all(np.mean(old[y:y+h,x:x+w]>0)<.04 for x,y,w,h in boxes):
                    self.reference=previous.copy();break
            if self.reference is None:return False,'NO_DARK_BASELINE'
            self.boxes=boxes
            self.context=(x0,y0,x1,y1)
        fractions=[float(np.mean(mask[y+2:y+h-2,x+2:x+w-2]>0)) for x,y,w,h in self.boxes]
        self.red_count=sum(f>.65 for f in fractions)
        if self.red_count==5:
            self.all_red_count+=1;self.off_count=0
            self.reason='FIVE_RED_CONFIRMED' if self.all_red_count>=3 else 'CONFIRMING_FIVE_RED'
            return False,self.reason
        if self.all_red_count<3:
            self.all_red_count=0
            return False,'WAIT_FIVE_RED'
        # Verify the whole housing AND each lens returns to its own earlier pixels.
        # Disappearance, shifted view, a blank/covered panel, and partial reds fail closed.
        x0,y0,x1,y1=self.context
        diff=np.abs(rgb.astype(np.int16)-self.reference.astype(np.int16)).mean(axis=2)
        patch=diff[y0:y1,x0:x1]
        lens_errors=[float(np.mean(diff[y:y+h,x:x+w])) for x,y,w,h in self.boxes]
        match=(max(fractions)<.04 and max(lens_errors)<9 and np.mean(patch)<5
               and np.mean(patch>20)<.025)
        if match:self.off_count+=1
        else:self.off_count=0
        if self.off_count>=3:
            self.released=True
            return True,'LIGHTS_OUT_CONFIRMED'
        return False,'VERIFYING_OFF' if match else 'OCCLUDED_OR_PARTIAL'
