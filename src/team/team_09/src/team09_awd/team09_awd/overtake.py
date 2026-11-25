"""Local pass candidates for a compact LiDAR return between observed walls."""
import math
import numpy as np
from .config import G
from .core import compensate
from .right_wall import follow_rollout, smooth_step, sweep


class Overtake:
    def __init__(self):
        self.side = 0
        self.last_seen = -math.inf
        self.pose = None
        self.center = None
        self.hits = 0

    def plan(self, points, walls, x, y, previous, speed, settings, stamp, pose):
        if stamp < self.last_seen or stamp - self.last_seen > .65:
            self.__init__()
        if any(w is None for w in walls):
            return None
        right, left = walls
        horizon = min(x[-1], right[-1, 0], left[-1, 0])
        q = points[(points[:, 0] > .15) & (points[:, 0] < horizon-.4)]
        def wall(w, pos):
            return np.interp(pos, w[:, 0], w[:, 1])
        q = q[(q[:, 1] > wall(right, q[:, 0])+.17) &
              (q[:, 1] < wall(left, q[:, 0])-.17)]
        expected = None if self.center is None else compensate([self.center], self.pose, pose)[0]
        target = None
        if len(q) >= 3:
            angles = np.arctan2(q[:, 1], q[:, 0])
            q = q[np.argsort(angles)]
            gaps = np.linalg.norm(np.diff(q, axis=0), axis=1)
            candidates = []
            for group in np.split(q, np.flatnonzero(gaps > .18)+1):
                if len(group) < 3:
                    continue
                size = np.ptp(group, axis=0)
                center = (group.min(axis=0)+group.max(axis=0))*.5
                if max(size) < .075 or size[0] > .8 or size[1] > .55:
                    continue
                if expected is not None and np.linalg.norm(center-expected) < .75:
                    candidates.append((0., center))
                elif abs(center[1]-np.interp(center[0], x, y)) < .38:
                    candidates.append((center[0], center))
            if candidates:
                target = min(candidates, key=lambda c:c[0])[1]
        if target is not None and stamp != self.last_seen:
            matched = expected is not None and np.linalg.norm(target-expected) < .75
            if not matched:
                self.side = 0
            self.hits = self.hits+1 if matched else 1
            self.center, self.pose, self.last_seen = target, pose, stamp
        elif target is None:
            target = expected
        if self.hits < 2 or target is None or target[0] < -.55:
            return None
        tx, ty = target
        keep = x <= horizon
        px, base = x[keep], y[keep]
        choices = []
        for side in ((self.side,) if self.side else (1, -1)):
            # Enter before the rear bumper, stay alongside, then blend back.
            offset = ty + side*(G.half_width + .36) - np.interp(tx, px, base)
            if side*offset < 0:
                offset = 0.
            enter = smooth_step(px/max(.7, tx-.7))
            leave = smooth_step((px-max(1., tx+1.4))/2.5)
            path = base + offset*enter*(1.-leave)
            k, s, poses = follow_rollout(px, path, previous, speed, settings)
            if len(s) < 2:
                continue
            free, hit = sweep(points, s, poses, settings["sweep_margin_m"])
            xx, yy = poses[:hit+1, :2].T
            beside = abs(xx-tx) < G.half_length+.45
            if (xx[-1] < max(1.2, tx+1.) or not np.any(beside)
                    or np.min(side*(yy[beside]-ty)) < G.half_width+.16
                    or np.any(yy-wall(right, xx) < G.half_width+.04)
                    or np.any(wall(left, xx)-yy < G.half_width+.04)):
                continue
            choices.append((free-.25*abs(offset), side, path, k, s, poses, free, hit))
        if choices:
            _, self.side, *solution = max(choices, key=lambda v:v[0])
            return (px, *solution)
        return None
