"""OpenCV camera geometry and visual start gate. No LiDAR or track-map inputs."""
from dataclasses import dataclass, field
import math
import cv2
import numpy as np


@dataclass
class VisionConfig:
    horizon: float = 5.0
    far_local_boundary_fit: bool = False
    spacing: float = 0.10
    bev_near: float = 0.50
    bev_half_width: float = 3.0
    bev_resolution: float = 0.025
    road_roi_top: float = 0.42
    traffic_roi_bottom: float = 0.35
    camera_y: float = 0.0
    camera_roll: float = 0.0
    camera_x: float = 0.195
    camera_height: float = 0.22
    camera_pitch: float = 0.10
    camera_yaw: float = 0.0
    road_hsv_low: list = field(default_factory=lambda: [0, 0, 20])
    road_hsv_high: list = field(default_factory=lambda: [179, 140, 175])
    white_hsv_low: list = field(default_factory=lambda: [0, 0, 110])
    white_hsv_high: list = field(default_factory=lambda: [179, 100, 255])
    red1_low: list = field(default_factory=lambda: [0, 110, 100])
    red1_high: list = field(default_factory=lambda: [12, 255, 255])
    red2_low: list = field(default_factory=lambda: [168, 110, 100])
    red2_high: list = field(default_factory=lambda: [179, 255, 255])
    yellow_low: list = field(default_factory=lambda: [18, 100, 140])
    yellow_high: list = field(default_factory=lambda: [38, 255, 255])
    green_low: list = field(default_factory=lambda: [40, 140, 150])
    green_high: list = field(default_factory=lambda: [95, 255, 255])
    light_min_area: float = 4.0
    light_max_area: float = 1500.0
    light_min_value: int = 150
    light_off_max_value: int = 130
    light_confirm_frames: int = 3
    light_row_tolerance: float = 0.6
    light_size_ratio_max: float = 2.0
    required_red_lights: int = 5
    min_boundary_samples: int = 8
    min_track_width: float = 0.55
    max_track_width: float = 4.0
    width_history_sec: float = 2.0
    enable_single_wall_fallback: bool = True
    allow_partial_trajectory: bool = True
    partial_trajectory_confidence: float = .85
    boundary_fit_order: int = 2
    wall_support_dilate_px: int = 15
    boundary_near_fit_span: float = 0.8
    wall_min_vertical_ratio: float = 0.85
    wall_min_height_px: int = 6


class TrafficLightDetector:
    def __init__(self, config):
        self.c = config
        self.state = 'UNKNOWN'
        self.pending = 'UNKNOWN'
        self.count = 0
        self.full_red_count = 0
        self.armed_boxes = []
        self.released = False
        self.boxes = []
        self.row_reference = None
        self.background_reference = None
        self.reason = 'unarmed'

    def _components(self, mask):
        count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
        out = []
        for i in range(1, count):
            x, y, w, h, area = stats[i]
            if (self.c.light_min_area <= area <= self.c.light_max_area and
                    0.45 <= w / max(h, 1) <= 2.0):
                out.append((int(x), int(y), int(w), int(h)))
        # A gantry is a row, not arbitrary red pixels in the image.
        best = []
        for box in out:
            y = box[1] + box[3] / 2
            row = [b for b in out
                   if abs(b[1]+b[3]/2-y) <= max(3, min(b[3],box[3])*self.c.light_row_tolerance)
                   and max(b[3],box[3])/min(b[3],box[3]) <= self.c.light_size_ratio_max
                   and max(b[2],box[2])/min(b[2],box[2]) <= self.c.light_size_ratio_max]
            if len(row) > len(best):
                best = row
        return sorted(best, key=lambda b: b[0])

    def update(self, hsv):
        roi = hsv[:int(hsv.shape[0] * self.c.traffic_roi_bottom)]
        red = cv2.inRange(roi, np.array(self.c.red1_low), np.array(self.c.red1_high))
        red |= cv2.inRange(roi, np.array(self.c.red2_low), np.array(self.c.red2_high))
        red[roi[:, :, 2] < self.c.light_min_value] = 0
        masks = {'RED': red,
                 'YELLOW': cv2.inRange(roi, np.array(self.c.yellow_low), np.array(self.c.yellow_high)),
                 'GREEN': cv2.inRange(roi, np.array(self.c.green_low), np.array(self.c.green_high))}
        boxes = {color: self._components(mask) for color, mask in masks.items()}
        red_boxes = boxes['RED']
        full_red = len(red_boxes) == self.c.required_red_lights
        if full_red:
            centres=np.array([x+w/2 for x,y,w,h in red_boxes]);gaps=np.diff(centres)
            full_red=bool(len(gaps) and min(gaps)>0 and max(gaps)<=2.*min(gaps))
        background=cv2.resize(hsv[:,:,2],(32,16),interpolation=cv2.INTER_AREA)
        # Ignore the lamps themselves when comparing the stationary scene.
        background_mask=np.ones(background.shape,dtype=bool)
        for x,y,w,h in (self.armed_boxes or red_boxes):
            x0=max(0,int((x-w)*32/roi.shape[1]));x1=min(32,int((x+2*w)*32/roi.shape[1])+1)
            y0=max(0,int((y-h)*16/hsv.shape[0]));y1=min(16,int((y+2*h)*16/hsv.shape[0])+1)
            background_mask[y0:y1,x0:x1]=False
        self.boxes = red_boxes if full_red else []
        if full_red:
            row=np.asarray(red_boxes,dtype=float)
            stable=self.row_reference is not None and self.row_reference.shape==row.shape and np.max(abs(row-self.row_reference))<=3.
            self.full_red_count=self.full_red_count+1 if stable else 1
            self.row_reference=row
            if self.full_red_count >= self.c.light_confirm_frames:
                self.armed_boxes = red_boxes[:self.c.required_red_lights]
                self.released = False
                self.background_reference=background.copy()
                self.reason='five_red_confirmed'
        else:
            self.full_red_count = 0
        known_red = False
        if self.armed_boxes and not self.released:
            for x,y,w,h in self.armed_boxes:
                patch = red[y:y+h,x:x+w]
                if patch.size and np.count_nonzero(patch)/patch.size > .15:
                    known_red = True
                    if not full_red:
                        self.boxes.append((x,y,w,h))
        # A red wall fragment is not a traffic lamp. During countdown, follow
        # the already observed lamp locations; after release require a new row.
        raw = 'RED' if full_red or known_red else next(
            (color for color in ('YELLOW','GREEN') if boxes[color]), 'UNKNOWN')
        if raw == 'UNKNOWN' and self.armed_boxes:
            dark = True
            for x, y, w, h in self.armed_boxes:
                patch = roi[y:y+h, x:x+w]
                if patch.size == 0 or np.median(patch[:, :, 2]) > self.c.light_off_max_value:
                    dark = False
            scene_visible=(self.background_reference is not None and background_mask.any() and
                np.median(abs(background.astype(float)-self.background_reference)[background_mask])<=35. and
                np.percentile(background[background_mask],75)>15.)
            if dark and scene_visible:
                raw = 'LIGHTS_OUT';self.reason='observed_lamps_off_scene_visible'
            elif dark:self.reason='scene_changed_or_blackout'
        if raw == self.pending:
            self.count += 1
        else:
            self.pending, self.count = raw, 1
        if self.count >= self.c.light_confirm_frames:
            self.state = raw
            if raw == 'LIGHTS_OUT':
                self.released = True
        return self.state


class CameraVision:
    def __init__(self, config):
        self.c = config
        self.light = TrafficLightDetector(config)
        self.width = None
        self.width_time = -math.inf
        self.homography = None
        self.previous_center = 0.0
        self.previous_path = None

    def project(self, points, k):
        """Ground points in +X forward/+Y left -> ROS optical image pixels."""
        points = np.asarray(points, dtype=float)
        from .camera_mount import rotation
        ground=np.column_stack((points,np.zeros(len(points))))
        # camera_height is ground-relative; path origin is the vehicle centre.
        local=(ground-np.array([self.c.camera_x,self.c.camera_y,self.c.camera_height]))@rotation(self.c.camera_roll,self.c.camera_pitch,self.c.camera_yaw)
        forward,left,up=local.T
        return np.column_stack((k[0,2]-k[0,0]*left/forward,k[1,2]-k[1,1]*up/forward))

    def _fit(self, samples):
        if len(samples) < self.c.min_boundary_samples:
            return None
        a = np.asarray(samples, dtype=float)
        if np.ptp(a[:, 0]) < 0.7:
            return None
        for _ in range(3):
            coef = np.polyfit(a[:, 0], a[:, 1], self.c.boundary_fit_order)
            error = np.abs(a[:, 1] - np.polyval(coef, a[:, 0]))
            keep = error <= max(0.04, 3 * float(np.median(error)))
            if keep.all() or keep.sum() < self.c.min_boundary_samples:
                break
            a = a[keep]
        return coef

    def _evaluate_boundary(self, fit, samples, x):
        if self.c.far_local_boundary_fit:
            from .far_geometry import local_boundary
            return local_boundary(samples,x,self.project(samples,self.intrinsics))
        values = np.polyval(fit, x)
        samples = np.asarray(sorted(samples), dtype=float)
        # Do not extrapolate a quadratic from a far, visible wall back to the
        # bonnet. Extend its near tangent, robustly excluding isolated paint.
        local = samples[samples[:, 0] <= samples[0, 0]+self.c.boundary_near_fit_span]
        slopes = [(b[1]-a[1])/(b[0]-a[0]) for i,a in enumerate(local) for b in local[i+1:]
                  if b[0]-a[0] >= .15]
        if slopes:
            slope = float(np.median(slopes))
            intercept = float(np.median(local[:, 1]-slope*local[:, 0]))
            tangent = intercept+slope*x
            mix = np.clip((x-samples[0,0]) / self.c.boundary_near_fit_span, 0, 1)
            mix = mix*mix*(3-2*mix)
            values = tangent*(1-mix)+values*mix
        return values

    def process(self, bgr, k, distortion, now):
        c = self.c
        self.intrinsics=k
        image = cv2.undistort(bgr, k, distortion) if np.any(distortion) else bgr.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        light = self.light.update(hsv)
        ground = np.float32([[c.bev_near, c.bev_half_width], [c.bev_near, -c.bev_half_width],
                             [c.horizon, -c.bev_half_width], [c.horizon, c.bev_half_width]])
        width = int(round(2 * c.bev_half_width / c.bev_resolution))
        height = int(round((c.horizon - c.bev_near) / c.bev_resolution))
        destination = np.float32([[0, height-1], [width-1, height-1], [width-1, 0], [0, 0]])
        self.homography = cv2.getPerspectiveTransform(self.project(ground, k).astype(np.float32), destination)
        road = cv2.inRange(hsv, np.array(c.road_hsv_low), np.array(c.road_hsv_high))
        white_wall = cv2.inRange(hsv, np.array(c.white_hsv_low), np.array(c.white_hsv_high))
        red_wall = cv2.inRange(hsv, np.array(c.red1_low), np.array(c.red1_high))
        red_wall |= cv2.inRange(hsv, np.array(c.red2_low), np.array(c.red2_high))
        top = int(image.shape[0] * c.road_roi_top)
        red_wall[:top] = 0
        # White starting-grid paint is on the road, not a wall. Where red/white
        # wall panels exist, require white support near the red wall material.
        if np.count_nonzero(red_wall) > 100:
            size = max(3, c.wall_support_dilate_px)
            support = cv2.dilate(red_wall, np.ones((size, size), np.uint8))
            count, labels, stats, _ = cv2.connectedComponentsWithStats(white_wall)
            tall = (stats[:, cv2.CC_STAT_HEIGHT] >= c.wall_min_height_px) & (
                stats[:, cv2.CC_STAT_HEIGHT] >= c.wall_min_vertical_ratio*stats[:, cv2.CC_STAT_WIDTH])
            tall[0] = False
            # A shaded white panel can satisfy the gray-road HSV range.
            # Keep its whole connected component when it touches a red panel;
            # keeping just a narrow red-neighbourhood would invent a second
            # road run inside the middle of the white wall.
            supported = tall.copy()
            supported[np.unique(labels[support > 0])] = True
            supported[0] = False
            support |= (supported[labels].astype(np.uint8)*255)
            white_wall &= support
        wall = white_wall | red_wall
        road[wall > 0] = 0
        road[:top] = 0
        valid = np.full(road.shape, 255, np.uint8)
        valid[:top] = 0
        warp = lambda a: cv2.warpPerspective(a, self.homography, (width, height), flags=cv2.INTER_LINEAR if c.far_local_boundary_fit else cv2.INTER_NEAREST)
        bev, valid_bev, wall_bev = warp(road), warp(valid), warp(wall)
        bev = cv2.morphologyEx(bev, cv2.MORPH_CLOSE, np.ones((3, 5), np.uint8))
        left_samples, right_samples = [], []
        reference = self.previous_center
        for row in range(height-1, -1, -2):
            xs = np.flatnonzero(bev[row] > (127 if c.far_local_boundary_fit else 0))
            if len(xs) < 3:
                continue
            runs = np.split(xs, np.flatnonzero(np.diff(xs) > 1) + 1)
            runs = [run for run in runs if len(run) * c.bev_resolution >= c.min_track_width * 0.6]
            if not runs:
                continue
            row_x = c.horizon-row/(height-1)*(c.horizon-c.bev_near)
            reference = float(np.interp(row_x,self.previous_path[:,0],self.previous_path[:,1])) if self.previous_path is not None else 0.0
            center_px = (c.bev_half_width - reference) / c.bev_resolution
            run = min(runs, key=lambda r: max(float(r[0])-center_px, center_px-float(r[-1]), 0))
            x = c.horizon - row / (height-1) * (c.horizon-c.bev_near)
            lo, hi = int(run[0]), int(run[-1])
            left_px,right_px=float(lo),float(hi)
            if c.far_local_boundary_fit:
                if lo>0 and bev[row,lo]!=bev[row,lo-1]:left_px=lo-1+(127-float(bev[row,lo-1]))/(float(bev[row,lo])-float(bev[row,lo-1]))
                if hi<width-1 and bev[row,hi]!=bev[row,hi+1]:right_px=hi+(float(bev[row,hi])-127)/(float(bev[row,hi])-float(bev[row,hi+1]))
            if lo > 3 and valid_bev[row, lo-3] and wall_bev[row, max(0, lo-4):lo+2].any():
                left_samples.append((x, c.bev_half_width - left_px * 2*c.bev_half_width/(width-1)))
            if hi < width-4 and valid_bev[row, hi+3] and wall_bev[row, hi:hi+5].any():
                right_samples.append((x, c.bev_half_width - right_px * 2*c.bev_half_width/(width-1)))
        left_fit, right_fit = self._fit(left_samples), self._fit(right_samples)
        observed = (left_fit is not None, right_fit is not None)
        x = np.arange(c.spacing, c.horizon+c.spacing/2, c.spacing)
        if c.far_local_boundary_fit and all(observed):
            lo=max(min(p[0] for p in a) for a in (left_samples,right_samples))
            hi=min(max(p[0] for p in a) for a in (left_samples,right_samples))
            if hi-lo>=c.spacing*3:x=np.arange(math.ceil(lo/c.spacing)*c.spacing,hi+1e-8,c.spacing)
        left = self._evaluate_boundary(left_fit, left_samples, x) if left_fit is not None else None
        right = self._evaluate_boundary(right_fit, right_samples, x) if right_fit is not None else None
        confidence = 0.0
        reason = 'missing_camera_boundaries_or_recent_width'
        if all(observed):
            slope = (np.polyval(np.polyder(left_fit), x) + np.polyval(np.polyder(right_fit), x))/2
            physical_width = (left-right)/np.sqrt(1+slope*slope)
            okay = (physical_width >= c.min_track_width) & (physical_width <= c.max_track_width)
            reason = 'inconsistent_camera_wall_width'
            # Assess width where BOTH walls were actually measured. Far
            # polynomial extrapolation cannot invalidate the observed bend.
            observed_near=max(min(p[0] for p in left_samples),min(p[0] for p in right_samples))
            observed_far=min(max(p[0] for p in left_samples),max(p[0] for p in right_samples))
            supported_width=(x>=observed_near-c.spacing*.5)&(x<=observed_far+c.spacing*.5)
            if supported_width.sum()>=3 and okay[supported_width].mean() >= 0.9:
                measured = float(np.median(physical_width[okay&supported_width]))
                self.width = measured if self.width is None else 0.8*self.width+0.2*measured
                self.width_time = now
                confidence = 1.0
                reason = 'both_camera_walls' 
        elif c.enable_single_wall_fallback and any(observed) and self.width is not None and now-self.width_time <= c.width_history_sec:
            fit = left_fit if observed[0] else right_fit
            lateral_width = self.width * np.sqrt(1+np.polyval(np.polyder(fit), x)**2)
            if observed[0]:
                right = left-lateral_width
            else:
                left = right+lateral_width
            confidence = 0.68 * max(.65, 1-.35*(now-self.width_time)/c.width_history_sec)
            reason = 'one_camera_wall_with_recent_width' 
        path = None
        if confidence and left is not None and right is not None:
            y = (left+right)/2
            lateral_limit = c.bev_half_width if all(observed) else c.bev_half_width+(self.width or 0.0)
            observed_samples=[a for a,seen in zip((left_samples,right_samples),observed) if seen]
            supported_end=min(max(p[0] for p in a) for a in observed_samples) if observed_samples else 0.
            full_supported=supported_end>=c.horizon-c.spacing*.5
            if (full_supported and np.isfinite(y).all() and np.all(left>right) and np.max(np.abs(y)) <= lateral_limit and
                    np.max(np.abs(np.diff(y))) <= 0.30):
                path = np.column_stack((x, y))
                self.previous_center = float(np.interp(1.0, x, y))
            else:
                # Reject an unobserved far extrapolation without discarding
                # the supported forward part of a tight bend.
                good=np.isfinite(y)&np.isfinite(left)&np.isfinite(right)&(np.abs(y)<=lateral_limit)&(left>right)
                good[1:] &= np.abs(np.diff(y))<=.30
                bad=np.flatnonzero(~good);end=int(bad[0]) if len(bad) else len(x)
                samples=[a for a,seen in zip((left_samples,right_samples),observed) if seen]
                supported=min(max(point[0] for point in a) for a in samples) if samples else 0.
                end=min(end,int(np.searchsorted(x,supported+c.spacing*.5,side='right')))
                measured=(end>=3 and x[end-1]>=c.bev_near+c.spacing and
                    all(sum(point[0]<=x[end-1]+c.spacing*.5 for point in a)>=c.min_boundary_samples for a in samples))
                if c.allow_partial_trajectory and measured:
                    x=x[:end];left=left[:end];right=right[:end];y=y[:end]
                    path=np.column_stack((x,y));self.previous_center=float(np.interp(1.,x,y))
                    if all(observed):confidence*=c.partial_trajectory_confidence
                    reason='partial_camera_trajectory'
                else:
                    confidence = 0.0
                    reason = 'invalid_camera_trajectory_geometry'

        # A failed image must not keep seeding the next image with a stale
        # vehicle-frame lateral reference. Recover from the current visible road.
        self.previous_path = path.copy() if path is not None else None
        if path is None:
            self.previous_center = 0.0
        bev_image = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
        if path is not None:
            for values, color in ((left, (255, 100, 0)), (right, (0, 200, 255)), ((left+right)/2, (0, 255, 0))):
                pixels = np.int32(np.column_stack(((c.bev_half_width-values)/(2*c.bev_half_width)*(width-1),
                                                  (c.horizon-x)/(c.horizon-c.bev_near)*(height-1))))
                cv2.polylines(bev_image, [pixels], False, color, 2)
            for points, color in ((np.column_stack((x,left)), (255,100,0)), (np.column_stack((x,right)), (0,200,255)), (path,(0,255,0))):
                points = points[points[:,0] > c.camera_x+.05]
                pixels = self.project(points, k)
                pixels = pixels[np.isfinite(pixels).all(axis=1)]
                if len(pixels):
                    cv2.polylines(image, [np.clip(pixels, -10000, 10000).astype(np.int32)], False, color, 2)
        cv2.rectangle(image, (0,top), (image.shape[1]-1,image.shape[0]-1), (100,180,255), 1)
        cv2.putText(image, 'ROAD / BEV ROI', (8,top+18), cv2.FONT_HERSHEY_SIMPLEX, .45, (100,180,255), 1)
        cv2.rectangle(image, (0, 0), (image.shape[1]-1, int(image.shape[0]*c.traffic_roi_bottom)), (255,0,255), 1)
        for bx, by, bw, bh in self.light.boxes:
            cv2.rectangle(image, (bx, by), (bx+bw, by+bh), (0,0,255), 1)
        cv2.putText(image, f'{light}  camera path={confidence:.2f}', (10, 22), cv2.FONT_HERSHEY_SIMPLEX, .55, (0,255,0), 2)
        return dict(track_width=self.width,track_width_age=now-self.width_time if self.width is not None else None,path=path, left=np.column_stack((x,left)) if path is not None else None,
                    right=np.column_stack((x,right)) if path is not None else None,
                    observed=observed, confidence=confidence, light=light, image=image, bev=bev_image,
                    left_samples=left_samples, right_samples=right_samples, reason=reason)
