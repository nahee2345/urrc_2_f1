"""Camera-only start-signal change detector.

A fixed bank of red lenses and red trackside advertisements are NOT signal
activity.  Detect pixels that become brighter/redder than the stable dark
image observed before the starter operates. No Gazebo or world-state topic is
used for the start decision.
"""
from collections import deque
import numpy as np


class SignalVision:
    def __init__(self, *, calibration_frames=12, roi=(.20, .80, .10, .70)):
        self.calibration_frames=int(calibration_frames)
        self.roi=roi  # (left, right, top, bottom): camera image fractions
        self.samples=deque(maxlen=self.calibration_frames)
        self.reference=None
        self.max_dynamic_red=0
        self.peak_blobs=0
        self.last_blob_count=0
        self.last_strength=0.
        self.last_coverage=0.
        self.locked=False
        self.frames=0

    @property
    def ready(self):
        return self.reference is not None

    def _roi(self, image):
        if image is None or getattr(image, 'ndim', 0)!=3 or image.shape[2]<3:
            raise ValueError('RGB image must be HxWx3')
        h,w=image.shape[:2]
        x0=max(0,int(w*self.roi[0]));x1=min(w,int(w*self.roi[1]))
        y0=max(0,int(h*self.roi[2]));y1=min(h,int(h*self.roi[3]))
        if x1-x0<32 or y1-y0<16:
            raise ValueError('RGB signal ROI is too small')
        # Fixed 2x subsampling yields consistent pixels without storing HD video.
        return np.ascontiguousarray(image[y0:y1:2,x0:x1:2,:3])

    def observe(self, rgb):
        import cv2
        frame=self._roi(rgb)
        self.frames+=1
        if self.reference is None:
            self.samples.append(frame.copy())
            if len(self.samples)>=self.calibration_frames:
                self.reference=np.median(np.stack(self.samples),axis=0).astype(np.float32)
                self.samples.clear()
            return 0,0.
        ref=self.reference
        # Important: subtract static red objects AND absolute brightness,
        # not just count red areas in the entire camera view.
        r=frame[:,:,0].astype(np.float32)
        g=frame[:,:,1].astype(np.float32)
        b=frame[:,:,2].astype(np.float32)
        old_r=ref[:,:,0]
        old_dom=old_r-np.maximum(ref[:,:,1],ref[:,:,2])
        dom=r-np.maximum(g,b)
        boost=r-old_r
        dom_boost=dom-old_dom
        mask=((boost>38.) & (dom>32.) & (dom_boost>12.) &
              (r>85.) & (r>g*1.20) & (r>b*1.20)).astype(np.uint8)
        # Suppress single-frame render speckle; connected lamps remain.
        count,_,stats,centroids=cv2.connectedComponentsWithStats(mask,8)
        blobs=[]
        for k in range(1,count):
            x,y,w,h,area=[int(q) for q in stats[k]]
            if 3<=area<=3500 and .32<=w/max(h,1)<=4.2:
                # Intensity measured on changed red pixels, never static barriers.
                energy=float(np.maximum(0.,boost[y:y+h,x:x+w][mask[y:y+h,x:x+w]>0]).sum())
                blobs.append((float(centroids[k,0]),float(centroids[k,1]),energy))
        # Look for a horizontal bank; unrelated bright objects cannot jointly
        # claim 4 real red lenses without occupying roughly the same row.
        best=(0,0.)
        for _,cy,_ in blobs:
            bank=[p for p in blobs if abs(p[1]-cy)<=9.]
            if len(bank)>=2:
                # Ignore duplicated portions of the same wide red patch.
                bank=sorted(bank)
                distinct=[bank[0]]
                for p in bank[1:]:
                    if p[0]-distinct[-1][0]>=3.:
                        distinct.append(p)
                bank=distinct
            strength=sum(p[2] for p in bank)
            if len(bank)>best[0] or (len(bank)==best[0] and strength>best[1]):
                best=(len(bank),strength)
        self.last_blob_count=best[0]
        self.last_strength=best[1]
        self.last_coverage=float(np.mean(mask>0))
        self.peak_blobs=max(self.peak_blobs,best[0])
        if best[0]:
            self.locked=True
        # Adjust slowly ONLY before any illuminated signal has been observed.
        # It also removes startup exposure adjustment (observed in v4 logs).
        if not self.locked and self.frames>self.calibration_frames+6:
            self.reference=(.94*self.reference+.06*frame.astype(np.float32))
        return best
