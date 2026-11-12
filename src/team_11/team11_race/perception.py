"""Race-use LiDAR perception; no Gazebo competitor pose or timer subscriptions.

Converts real front/rear laser clusters into short-lived world-coordinate
tracks. Repeated observations are required before a cluster is considered a
vehicle candidate. Conservative filters exclude extensive wall segments.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, hypot, isfinite, atan2
from typing import Sequence


@dataclass
class Actor:
    id: int
    x: float
    y: float
    speed: float
    hits: int
    last_seen: float
    first_x: float
    first_y: float
    source: str
    moving_forward: bool = False
    weave_reversals: int = 0
    last_lateral_rate: int = 0
    prev_y: float = 0.0

    def local(self, ego: tuple[float, float, float, float]):
        ex, ey, _, yaw = ego
        dx, dy = self.x-ex, self.y-ey
        return (dx*cos(yaw)+dy*sin(yaw), -dx*sin(yaw)+dy*cos(yaw))


def extract_clusters(ranges: Sequence[float], angle_min: float,
                     angle_increment: float, range_min: float, range_max: float,
                     source: str) -> list[tuple[float, float]]:
    """Extract compact returns in ego body coordinates. +x forward, +y left."""
    if source not in ("front", "rear") or not isfinite(angle_increment) or angle_increment<=0:
        return []
    segments=[]
    active=[]
    previous=None

    def finish():
        nonlocal active
        if len(active)>=2:
            mx=sum(p[0] for p in active)/len(active)
            my=sum(p[1] for p in active)/len(active)
            spread=max(hypot(p[0]-mx,p[1]-my) for p in active)*2
            if .07<=spread<=1.15:
                segments.append((mx,my))
        active=[]

    for i,v in enumerate(ranges):
        r=float(v)
        if (not isfinite(r) or r<=max(.12,range_min) or
                r>=min(16.,range_max-.05)):
            finish()
            previous=None
            continue
        ang=angle_min+i*angle_increment
        if source=="front":
            p=(.187+r*cos(ang), r*sin(ang))
        else:
            p=(-.208-r*cos(ang), -r*sin(ang))
        # A wall generally spans a large physical surface; keep full runs
        # rather than fragmenting them into small 'fake cars'.
        if previous is not None and hypot(p[0]-previous[0],p[1]-previous[1])>.28:
            finish()
        active.append(p)
        previous=p
    finish()
    return segments


class LaserTracker:
    def __init__(self):
        self.tracks: list[Actor]=[]
        self.next_id=1
        self.frames=0
        self.last_front=0.
        self.last_rear=0.

    def update(self, ranges, amin, astep, rmin, rmax, source,
               ego: tuple[float,float,float,float], speed: float, now: float):
        if ego is None or not all(isfinite(float(v)) for v in ego):
            return
        self.frames+=1
        if source=="front":self.last_front=now
        elif source=="rear":self.last_rear=now
        else:return
        ex,ey,_,yaw=ego
        observations=[]
        for x,y in extract_clusters(ranges,amin,astep,rmin,rmax,source):
            observations.append((ex+x*cos(yaw)-y*sin(yaw),
                                 ey+x*sin(yaw)+y*cos(yaw)))
        used=set()
        for x,y in observations:
            candidate=None
            cost=1e9
            for track in self.tracks:
                if track.id in used or track.source!=source:
                    continue
                age=now-track.last_seen
                if age<=0 or age>.5:
                    continue
                dist=hypot(x-track.x,y-track.y)
                if dist<min(2.2,.40+age*25.) and dist<cost:
                    candidate=track
                    cost=dist
            if candidate is None:
                self.tracks.append(Actor(
                    self.next_id,x,y,0.,1,now,x,y,source,
                    prev_y=y))
                self.next_id+=1
                continue
            used.add(candidate.id)
            dt=now-candidate.last_seen
            forward=(x-candidate.x)*cos(yaw)+(y-candidate.y)*sin(yaw)
            if dt>=.03:
                measured=forward/dt
                if -2.<=measured<=27.:
                    candidate.speed=max(0.,min(25.,
                        .45*max(0.,measured)+.55*candidate.speed))
                lateral=(-sin(yaw)*(y*0+x-candidate.x)
                         +cos(yaw)*(y-candidate.y))/dt
                sign=1 if lateral>.55 else -1 if lateral<-.55 else 0
                if sign and candidate.last_lateral_rate and sign!=candidate.last_lateral_rate:
                    candidate.weave_reversals+=1
                if sign:candidate.last_lateral_rate=sign
            candidate.x=x
            candidate.y=y
            candidate.hits=min(99,candidate.hits+1)
            candidate.last_seen=now
            if candidate.hits>=3:
                dx=(candidate.x-candidate.first_x)*cos(yaw)
                dy=(candidate.y-candidate.first_y)*sin(yaw)
                candidate.moving_forward=(
                    candidate.moving_forward or
                    (dx+dy>=.35 and candidate.speed>=.5))
        self.tracks=[t for t in self.tracks if now-t.last_seen<.7]

    def visible(self, now: float, min_hits: int = 2) -> list[Actor]:
        return [t for t in self.tracks
                if t.hits>=min_hits and now-t.last_seen<=.40]

    def neighbor_motion(self, ego, now: float) -> bool:
        for actor in self.visible(now,min_hits=3):
            x,y=actor.local(ego)
            if (-3.5<=x<=4.5 and .45<=abs(y)<=3.5 and
                    actor.moving_forward):
                return True
        return False

    def neighbor_present(self, ego, now: float) -> bool:
        return any(-3.5<=x<=4.5 and .45<=abs(y)<=3.5
                   for actor in self.visible(now)
                   for x,y in [actor.local(ego)])
