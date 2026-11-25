"""Bounded queues pair RGB/depth by sensor stamp, not callback arrival order."""
from collections import deque


class RGBDPairer:
    def __init__(self, tolerance=.04, size=8):
        self.tolerance=tolerance; self.size=size
        self.rgb=deque(maxlen=size); self.depth=deque(maxlen=size)
        self.latest={"rgb":None,"depth":None}; self.dropped=0

    def add(self, kind, stamp, data):
        previous=self.latest[kind]
        if previous is not None and stamp < previous:
            self.rgb.clear(); self.depth.clear(); self.latest={"rgb":None,"depth":None}
        if previous == stamp: return None
        self.latest[kind]=stamp
        queue=getattr(self,kind); queue.append((stamp,data))
        other=self.depth if kind=="rgb" else self.rgb
        if not other: return None
        candidate=min(other, key=lambda x:abs(x[0]-stamp))
        if abs(candidate[0]-stamp)>self.tolerance:
            while other and other[0][0]<stamp-self.tolerance:
                other.popleft(); self.dropped+=1
            return None
        queue.pop(); other.remove(candidate)
        return (stamp,data,*candidate) if kind=="rgb" else (*candidate,stamp,data)


class AccelerationGuard:
    """Only withhold positive acceleration; preserve existing braking limits.

    Uses an actual clearance trend and wall-heading evidence, plus route quality
    loss. It never extrapolates an old route or adds an independent steering cmd.
    """
    def __init__(self):
        self.last=None; self.release_after=None; self.closing=0.; self.reason="CLEAR"

    def apply(self, proposed, previous, speed, result, stamp, cfg):
        if not cfg["reaccel_guard"]: return proposed,"OFF"
        route=getattr(result,"route_kind","WAIT")
        fresh=self.last is None or stamp>self.last[0]
        if self.last and fresh:
            dt=stamp-self.last[0]
            self.closing=(self.last[1]-result.clearance)/dt if 0<dt<=cfg["guard_max_sample_gap_s"] else 0.
        lost = bool(self.last and self.last[2]!="REACTIVE" and route=="REACTIVE")
        poor = result.wall_quality < cfg["guard_wall_quality"]
        near = result.clearance < cfg["guard_clearance_m"]
        shrinking = self.closing>cfg["guard_closing_mps"]
        # centre_error is centreline position in body coordinates. A positive
        # heading with the vehicle to the RIGHT of the centre means its forward
        # axis points farther toward the right boundary (same-sign product).
        points_out = result.centre_error*result.heading > 0 and abs(result.heading)>cfg["guard_heading_rad"]
        risky = abs(speed)>=cfg["guard_min_speed_mps"] and (lost or (near and (shrinking or points_out or poor)))
        if risky:
            self.release_after=stamp+cfg["guard_release_s"]
            self.reason="ROUTE_LOSS" if lost else "CLOSING_SIDE" if shrinking else "OUTWARD_HEADING" if points_out else "WALL_UNCERTAIN"
        elif self.release_after is not None and stamp>=self.release_after:
            self.release_after=None; self.reason="CLEAR"
        if fresh: self.last=(stamp,result.clearance,route)
        if self.release_after is not None and proposed>previous and proposed>0:
            return max(0.,min(proposed,previous,abs(speed))),self.reason
        return proposed,self.reason
