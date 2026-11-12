"""Competition ROS race policy ported from TEAM11's three_lane C++ practice logic.

Decision priority: front imminent TTC<=3s, corner center merge, rear TTC<=5s,
otherwise keep the lane. Every maneuver requires a clear predicted trajectory;
blocked passing means following/braking, never an unsafe side jump.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import cos, sin, hypot, sqrt, inf
from typing import Iterable

from .planner import smoothstep5
from .perception import Actor

FRONT_TTC=3.
REAR_TTC=5.
CLOSING_MIN=2.
MIN_CHANGE=18.
CLEARANCE=1.05
CENTER_LANE=1
ENTER_CURVATURE=.030
EXIT_CURVATURE=.022


@dataclass(frozen=True)
class Decision:
    lane: int | None
    speed_cap: float
    reason: str
    target_id: int = -1
    ttc: float = inf


class RaceDirector:
    def __init__(self):
        self.corner_mode=False
        self.front_active=-1
        self.front_change_travel=-100.
        self.last_defense_travel=-100.
        self.last_change_travel=-100.
        self.last_status=""

    def _lane_near(self, route, lane: int, s: float, actor: Actor) -> bool:
        a=route.sample(lane,s)
        return hypot(actor.x-a[0],actor.y-a[1])<.56

    @staticmethod
    def _local_actor(actor: Actor,pose):
        return actor.local(pose)

    def _clear(self, planner, pose, target, actors, duration, initial_speed):
        if target not in range(3) or abs(target-planner.target)!=1 or planner.changing:
            return False
        lane=planner.target
        ex,ey,_,yaw=pose
        speed=max(3.,initial_speed)
        # Predicted candidate trajectory over the physical lane-change distance.
        for i in range(13):
            t=i/12.
            path_s=planner.last_s + duration*t
            ratio=smoothstep5(t)
            a=planner.route.sample(lane,path_s)
            b=planner.route.sample(target,path_s)
            px=a[0]+ratio*(b[0]-a[0])
            py=a[1]+ratio*(b[1]-a[1])
            elapsed=duration*t/speed
            for actor in actors:
                if actor.hits<2:
                    continue
                ax=actor.x+actor.speed*cos(yaw)*elapsed
                ay=actor.y+actor.speed*sin(yaw)*elapsed
                distance=hypot(px-ax,py-ay)
                if distance<CLEARANCE:
                    return False
                if i==12 and distance<2.0:
                    return False
        return True

    def choose(self,planner,pose,actors: Iterable[Actor],now: float)->Decision:
        actors=list(actors)
        route=planner.route
        s=planner.last_s
        if s is None:
            return Decision(None,0.,"NOT_LOCALIZED")
        speed=planner.speed
        travel=planner.travel
        lane=planner.target

        # Look ahead on the real road's center line, not the lateral lane-shift.
        k=max(route.curvature(s+a) for a in (6.,18.,30.))
        if not self.corner_mode and k>=ENTER_CURVATURE:
            self.corner_mode=True
        elif self.corner_mode and k<=EXIT_CURVATURE:
            self.corner_mode=False

        front=[]
        rear=[]
        for actor in actors:
            x,y=self._local_actor(actor,pose)
            if x>=.25 and x<=16.:
                front.append((x,actor))
            elif x<=-.25 and x>=-16.:
                rear.append((-x,actor))

        # Frontal threats follow the vehicle's CURRENT lane, not its centerline.
        own_front=[(gap,a) for gap,a in front
                   if self._lane_near(route,lane,s+gap,a)]
        own_front.sort(key=lambda x:x[0])
        lead=own_front[0] if own_front else None
        follow_cap=inf
        ttc=inf
        blocked=False
        if lead:
            gap,actor=lead
            follow_cap=sqrt(max(0.,actor.speed*actor.speed+
                                8.*max(0.,gap-10.)))
            closing=speed-actor.speed
            if closing>=CLOSING_MIN:
                ttc=max(0.,gap-1.)/closing
                blocked=ttc<=FRONT_TTC

        if planner.changing:
            return Decision(None,follow_cap,"COMMITTED_LANE_CHANGE",
                            lead[1].id if lead else -1,ttc)

        # The passing decision always outranks corner preparation and defense.
        if blocked and travel>=12. and travel-self.last_change_travel>=26.:
            if k<.080 or (lead and lead[1].speed<=1. and speed<=7.):
                options=([0,2] if lane==1 else [1])
                for next_lane in options:
                    if self._clear(planner,pose,next_lane,actors,MIN_CHANGE,
                                   max(3.,speed)):
                        self.front_active=lead[1].id
                        self.front_change_travel=travel
                        self.last_change_travel=travel
                        return Decision(next_lane,inf,"FRONT_PASS",lead[1].id,ttc)
        if blocked:
            return Decision(None,follow_cap,"FRONT_FOLLOW",lead[1].id,ttc)

        if self.front_active>=0:
            target=next((t for t in actors if t.id==self.front_active),None)
            if (target is None and travel-self.front_change_travel>MIN_CHANGE) or (
                target is not None and target.local(pose)[0]<-10.) or (
                    travel-self.front_change_travel>60.):
                self.front_active=-1

        # Center line is favored on the approach to any nontrivial bend.
        if self.corner_mode and lane!=CENTER_LANE:
            if travel>=12. and self.front_active<0 and self._clear(
                planner,pose,CENTER_LANE,actors,MIN_CHANGE,max(speed,7.)):
                self.last_change_travel=travel
                return Decision(CENTER_LANE,follow_cap,"CORNER_CENTER")
            return Decision(None,min(follow_cap,7.),"CORNER_WAIT")

        # No rear blocking when overtaking, cornering or immediately after
        # another maneuver. Verify closing TTC from a REAL rear laser track.
        if (not self.corner_mode and self.front_active<0 and
                travel>=12. and travel-self.last_change_travel>=26. and
                travel-self.last_defense_travel>=18.):
            candidates=[]
            for gap,actor in rear:
                if actor.source!="rear" or actor.hits<3:
                    continue
                closing=actor.speed-speed
                if closing<CLOSING_MIN:
                    continue
                time=(gap-1.)/closing
                if .0<time<=REAR_TTC:
                    candidates.append((time,actor))
            candidates.sort(key=lambda x:x[0])
            if candidates:
                time,threat=candidates[0]
                # A lane movement may only proceed through a clear adjacent
                # path. The fastest rear vehicle is NEVER treated as exempt
                # from the collision check.
                alternatives=[a for a in range(3) if abs(a-lane)==1]
                if threat.weave_reversals>=2:
                    alternatives.sort(key=lambda x: (
                        x==CENTER_LANE, -abs(x-lane)))
                else:
                    alternatives.sort(key=lambda x: (
                        -hypot(route.sample(x,s)[0]-threat.x,
                               route.sample(x,s)[1]-threat.y), x))
                for candidate in alternatives:
                    if self._clear(planner,pose,candidate,actors,MIN_CHANGE,
                                   max(speed,3.)):
                        self.last_defense_travel=travel
                        self.last_change_travel=travel
                        kind="REAR_ZIGZAG" if threat.weave_reversals>=2 else "REAR_DEFENSE"
                        return Decision(candidate,follow_cap,kind,threat.id,time)
                return Decision(None,follow_cap,"REAR_UNSAFE_HOLD",threat.id,time)

        return Decision(None,follow_cap,"KEEP_ROUTE")
