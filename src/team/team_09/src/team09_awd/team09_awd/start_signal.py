"""Five observed red lamps, then all five dark at their original image positions.

The camera alone supplies the optical decision. No world state, host flag or
elapsed-time fallback can turn this detector to GO.
"""
from copy import deepcopy
import math
import cv2
import numpy as np
from .config import SETTINGS


class StartSignal:
    def __init__(self, config=None):
        self.cfg = deepcopy(SETTINGS["start_signal"] if config is None else config)
        self.state = "WAIT_RED"
        self.reason = "WAIT_FIVE_RED"
        self.last = -1.
        self.boxes = None
        self.pending = None
        self.reference = None
        self.background = None
        self.on_values = []
        self.on_frames = 0
        self.off_frames = 0
        self.seen_five = -1.
        self.red_count = 0
        self.dark_count = 0
        self.red_ratios = []
        self.value_fractions = []
        self.scene_delta = None
        self.scene_nonblack = None
        self.frame_gap_s = None
        self.go_stamp = None

    def reset(self, reason="RESET"):
        self.__init__(self.cfg)
        self.reason = reason

    def _row(self, mask):
        p = self.cfg
        h, w = mask.shape
        stats = cv2.connectedComponentsWithStats(mask.astype("uint8"), 8)[2]
        candidates = []
        margin = p["search_margin_fraction"]
        for x, y, bw, bh, area in stats[1:]:
            if (p["blob_min_area_fraction"]*h*w <= area <= p["blob_max_area_fraction"]*h*w
                    and p["blob_min_height_fraction"]*h <= min(bw, bh)
                    and max(bw, bh) <= p["blob_max_height_fraction"]*h
                    and p["blob_min_aspect"] < bw/bh < p["blob_max_aspect"]
                    and margin*w < x < (1-margin)*w and margin*h < y < (1-margin)*h):
                candidates.append((int(x), int(y), int(bw), int(bh)))
        candidates.sort()
        self.red_count = min(5, len(candidates))
        for i in range(max(0, len(candidates)-4)):
            row = candidates[i:i+5]
            cx = np.array([x+bw/2 for x,y,bw,bh in row])
            cy = np.array([y+bh/2 for x,y,bw,bh in row])
            gaps = np.diff(cx)
            if (np.ptp(cy) < max(b[3] for b in row)*p["row_height_fraction"]
                    and min(gaps) > p["min_gap_width_fraction"]*w
                    and max(gaps) < min(gaps)*p["gap_ratio_max"]):
                return row
        return None

    @staticmethod
    def _same_row(a, b):
        return a is not None and all(
            abs((x+bw/2)-(X+BW/2)) <= min(bw,BW)/2
            and abs((y+bh/2)-(Y+BH/2)) <= min(bh,BH)/2
            for (x,y,bw,bh),(X,Y,BW,BH) in zip(a,b))

    def update(self, image, stamp):
        if self.state == "GO":
            return self.state
        if not math.isfinite(stamp):
            self.reset("INVALID_STAMP")
            return self.state
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or not image.size:
            self.reset("INVALID_IMAGE")
            return self.state
        if stamp < self.last:
            self.reset("CAMERA_TIME_RESET")
            self.last = stamp
            return self.state
        if stamp == self.last:
            self.reason = "DUPLICATE_STAMP"
            return self.state
        self.frame_gap_s = None if self.last < 0 else stamp-self.last
        gap = self.frame_gap_s is not None and self.frame_gap_s > self.cfg["max_frame_gap_s"]
        self.last = stamp
        if gap:
            self.on_frames = self.off_frames = 0
            self.pending = None
        if self.reference is not None and image.shape != self.reference.shape:
            self.reset("IMAGE_SIZE_CHANGED")
            self.last = stamp
            return self.state
        p = self.cfg
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = (((hsv[:,:,0] < p["red_hue_band"]) | (hsv[:,:,0] > 180-p["red_hue_band"]))
                & (hsv[:,:,1] > p["min_red_saturation"]) & (hsv[:,:,2] > p["min_red_value"]))
        if self.boxes is None:
            row = self._row(mask)
            if row is None:
                self.on_frames = 0
                self.pending = None
                self.reason = "WAIT_FIVE_RED"
                return self.state
            self.on_frames = self.on_frames+1 if self._same_row(self.pending, row) else 1
            self.pending = row
            self.reason = "CONFIRM_FIVE_RED"
            if self.on_frames < p["on_frames"]:
                return self.state
            self.boxes = row
            self.reference = image.copy()
            self.on_values = [float(hsv[y:y+bh,x:x+bw,2].mean()) for x,y,bw,bh in row]
            self.background = np.ones(mask.shape, bool)
            pad = max(1, round(image.shape[0]*p["background_padding_height_fraction"]))
            for x,y,bw,bh in row:
                self.background[max(0,y-pad):y+bh+pad, max(0,x-pad):x+bw+pad] = False
            self.seen_five = stamp
            self.state = "FIVE_RED"
            self.reason = "FIVE_RED_LOCKED"
            return self.state
        if stamp-self.seen_five > p["lock_timeout_s"]:
            self.reset("FIVE_RED_LOCK_EXPIRED")
            self.last = stamp
            return self.state
        self.red_ratios = [float(mask[y:y+bh,x:x+bw].mean()) for x,y,bw,bh in self.boxes]
        values = [float(hsv[y:y+bh,x:x+bw,2].mean()) for x,y,bw,bh in self.boxes]
        self.value_fractions = [v/max(on, 1.) for v,on in zip(values,self.on_values)]
        self.red_count = sum(r >= p["off_red_ratio"] for r in self.red_ratios)
        self.dark_count = sum(r < p["off_red_ratio"] and v <= p["off_value_fraction"]
                              for r,v in zip(self.red_ratios,self.value_fractions))
        delta = cv2.absdiff(image, self.reference)
        self.scene_delta = float(delta[self.background].mean())
        self.scene_nonblack = float((hsv[:,:,2][self.background] > p["black_value_max"]).mean())
        if self.scene_nonblack < p["scene_min_nonblack_fraction"]:
            self.reason = "BLACK_FRAME"
            self.off_frames = 0
        elif self.scene_delta > p["scene_delta_limit"]:
            self.reason = "SCENE_CHANGED"
            self.off_frames = 0
        elif self.red_count == 5:
            self.seen_five = stamp
            self.reason = "FIVE_RED_LOCKED"
            self.off_frames = 0
        elif gap:
            # A gap is never an off observation. Require new consecutive images.
            self.reason = "FRAME_GAP_RECONFIRM"
        elif self.dark_count == 5:
            self.off_frames += 1
            self.reason = "CONFIRM_ALL_FIVE_OFF"
            if self.off_frames >= p["off_frames"]:
                self.state = "GO"
                self.reason = "ALL_FIVE_OFF_CONFIRMED"
                self.go_stamp = stamp
        else:
            self.off_frames = 0
            self.reason = "RED_REMAINS" if self.red_count else "LENSES_NOT_DIM"
        return self.state

    def diagnostics(self):
        return dict(signal_reason=self.reason, red_count=self.red_count, dark_count=self.dark_count,
                    on_frames=self.on_frames, off_frames=self.off_frames,
                    lamp_red_ratios=self.red_ratios, lamp_value_fractions=self.value_fractions,
                    scene_delta=self.scene_delta, scene_nonblack_fraction=self.scene_nonblack,
                    camera_frame_gap_s=self.frame_gap_s, camera_go_stamp_s=self.go_stamp,
                    lamp_boxes=self.boxes)

    def annotate(self, image):
        out = image.copy()
        for i, (x,y,w,h) in enumerate(self.boxes or self.pending or []):
            color = (0,255,0) if self.state == "GO" else (0,180,255)
            cv2.rectangle(out, (x,y), (x+w,y+h), color, 2)
            cv2.putText(out, str(i+1), (x,max(14,y-5)), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
        text = f"{self.state} {self.reason} red={self.red_count}/5 dark={self.dark_count}/5 off={self.off_frames}"
        cv2.putText(out, text, (6,22), cv2.FONT_HERSHEY_SIMPLEX, .45, (0,255,255), 1)
        return out
