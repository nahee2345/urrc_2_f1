"""TEAM11 camera start light measurement without fixed-width five-bin assumptions.

The five LEDs occupy only a small fraction of the image. Count actual red
evidence in the gantry ROI rather than demanding one red lamp in each of
five equal screen columns. The official external start gate is still required.
"""
from __future__ import annotations
from dataclasses import dataclass


def red_evidence(data, width: int, height: int, step: int, encoding: str, stride: int = 2) -> int:
    """Count sampled red pixels (0 means dark), or -1 for malformed frames."""
    encoding = str(encoding).lower()
    if (width < 160 or height < 80 or stride < 1 or
            encoding not in ("rgb8", "bgr8") or step < width*3 or
            len(data) < step*height):
        return -1
    red_i, green_i, blue_i = (0,1,2) if encoding=="rgb8" else (2,1,0)
    # A broad ROI catches the high gantry on different grid slots; ignore
    # screen edges and the lower bumper/road where opponent paint may appear.
    xmin,xmax=int(width*.08),int(width*.92)
    ymin,ymax=int(height*.08),int(height*.78)
    active=0
    for y in range(ymin,ymax,stride):
        row=y*step
        for x in range(xmin,xmax,stride):
            i=row+3*x
            r,g,b=int(data[i+red_i]),int(data[i+green_i]),int(data[i+blue_i])
            if r>=105 and r>=g*1.65 and r>=b*1.65 and r-g>=65:
                active+=1
    return active


@dataclass
class LightTransition:
    """Require actual red evidence followed by stable darkness.

    Never grants permission by itself: external race gate must also be TRUE.
    """
    min_on_pixels: int = 6
    max_off_pixels: int = 2
    red_frames: int = 0
    dark_frames: int = 0
    saw_red: bool = False
    started: bool = False
    peak: int = 0
    last_red: int = -1
    valid_frames: int = 0

    def update(self, red_count: int) -> bool:
        if red_count < 0:
            return self.started
        self.valid_frames += 1
        self.last_red=red_count
        self.peak=max(self.peak,red_count)
        if self.started:
            return True
        if not self.saw_red:
            if red_count >= self.min_on_pixels:
                self.red_frames+=1
                if self.red_frames>=2:
                    self.saw_red=True
            else:
                self.red_frames=0
            return False
        threshold=max(self.max_off_pixels,int(self.peak*.07))
        if red_count<=threshold:
            self.dark_frames+=1
        else:
            self.dark_frames=0
        if self.dark_frames>=4:
            self.started=True
        return self.started
