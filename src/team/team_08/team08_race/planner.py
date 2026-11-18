"""Self-contained six-map three-lane path follower; uses only own world pose.

World pose is obtained from the TEAM11 vehicle's own PosePublisher plugin.
Precomputed map following is for an authorized map-replay competition format.
"""
from __future__ import annotations
from bisect import bisect_right
import csv
import hashlib
import json
import math
from pathlib import Path

WHEELBASE=.34
MAX_STEER=.52       # +/-29.8 deg, below the agreed +/-30 deg bound.
MAX_STEER_RATE=1.57 # rad/s
MAX_YAW_RATE=1.2
MAX_LATERAL_A=2.5
MAX_ACCEL=2.0
MAX_BRAKE=4.0
MAP_SPEED_CAP=15.0
LANE_SHIFT=18.0


def clamp(v,lo,hi):
    return min(hi,max(lo,v))


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def smoothstep5(t):
    q=clamp(t,0.,1.)
    return q*q*q*(10.+q*(-15.+6.*q))


class RouteMap:
    def __init__(self,assets: Path):
        self.assets=Path(assets)
        self.meta=json.loads((self.assets/"track.json").read_text(encoding="utf-8"))
        raw=(self.assets/"centerline.csv").read_bytes()
        if self.meta.get("source_centerline_sha"):
            calculated=hashlib.sha1(b"blob "+str(len(raw)).encode()+bytes([0])+raw).hexdigest()
            if calculated!=self.meta["source_centerline_sha"]:
                raise ValueError("Embedded track geometry checksum changed")
        with (self.assets/"centerline.csv").open(newline="",encoding="utf-8") as stream:
            rows=list(csv.DictReader(stream))
        self.center=[(float(p["x_m"]),float(p["y_m"]),float(p["z_m"]),
                      float(p["yaw_rad"]),float(p["road_width_m"])) for p in rows]
        if len(self.center)<200:raise ValueError("Track missing points")
        self.routes=[]
        for lane in range(3):
            with (self.assets/f"lane_{lane}.csv").open(newline="",encoding="utf-8") as stream:
                route=list(csv.DictReader(stream))
            if len(route)!=len(self.center):
                raise ValueError(f"Lane {lane} sample count differs from centerline")
            data=[(float(p["distance_m"]),float(p["x_m"]),
                   float(p["y_m"]),float(p["z_m"])) for p in route]
            if any(b[0]<=a[0] for a,b in zip(data,data[1:])):
                raise ValueError(f"Lane {lane} distance not increasing")
            self.routes.append(data)
        self.samples=[p[0] for p in self.routes[1]]
        a,b=self.center[0],self.center[-1]
        self.length=self.samples[-1]+math.hypot(a[0]-b[0],a[1]-b[1])
        self.last_index=None

    def sample(self,lane: int,s: float):
        route=self.routes[lane]
        s=s%self.length
        idx=bisect_right(self.samples,s)
        if idx<=0:return route[0][1:4]
        left=route[idx-1]
        if idx==len(route):
            right=(self.length,*route[0][1:4])
        else:
            right=route[idx]
        fraction=clamp((s-left[0])/max(1e-6,right[0]-left[0]),0.,1.)
        return tuple(left[k]+fraction*(right[k]-left[k]) for k in (1,2,3))

    def nearest(self,x: float,y: float):
        # Global search at startup, then a local continuity window around
        # the last segment. This avoids jumping across crossing track sections.
        n=len(self.center)
        if self.last_index is None:
            indices=range(n)
        else:
            indices=((self.last_index+delta)%n for delta in range(-70,71))
        best=(float("inf"),0)
        for i in indices:
            a=self.center[i]
            dx,dy=x-a[0],y-a[1]
            sq=dx*dx+dy*dy
            if sq<best[0]:best=(sq,i)
        self.last_index=best[1]
        s=self.samples[best[1]]
        return s,math.sqrt(best[0]),self.center[best[1]][4]

    def curvature(self,s):
        a=self.sample(1,s)
        b=self.sample(1,s+3.)
        c=self.sample(1,s+6.)
        first=math.atan2(b[1]-a[1],b[0]-a[0])
        second=math.atan2(c[1]-b[1],c[0]-b[0])
        return abs(wrap(second-first))/max(.1,math.hypot(c[0]-b[0],c[1]-b[1]))


class Planner:
    def __init__(self,route: RouteMap,first_lane: int,max_speed=MAP_SPEED_CAP,
                 external_strategy: bool = False):
        self.route=route
        self.lane=first_lane
        self.target=first_lane
        self.changing=False
        self.change_distance=LANE_SHIFT
        self.travel=0.
        self.last_s=None
        self.speed=0.
        self.steer_right=0.
        self.max_speed=max_speed
        self.laps=0
        self.invalid_reason=None
        self.external_strategy=external_strategy
        self.offtrack_frames=0
        self.change_reason="INITIAL"
        self.change_count=0

    def request_lane(self,target: int,reason: str = "RACE_DIRECTOR")->bool:
        """Commit an adjacent lane change, then finish before any next choice."""
        if (target not in (0,1,2) or abs(target-self.target)!=1 or
                self.changing or self.invalid_reason):
            return False
        self.lane=self.target
        self.target=target
        self.changing=True
        self.change_distance=0.
        self.change_reason=reason
        self.change_count+=1
        return True

    def stop(self,reason):
        self.speed=0.
        self.invalid_reason=reason
        return 0.,0.,reason

    def _point(self,s,preview=0.):
        if not self.changing:
            return self.route.sample(self.target,s)
        progress=smoothstep5((self.change_distance+preview)/LANE_SHIFT)
        a=self.route.sample(self.lane,s)
        b=self.route.sample(self.target,s)
        return tuple(a[i]+progress*(b[i]-a[i]) for i in range(3))

    def step(self,pose: tuple[float,float,float,float],dt:float,
             traffic_speed_cap: float = float("inf")):
        if self.invalid_reason:
            return 0.,0.,self.invalid_reason
        if dt<=0. or dt>.25:
            return 0.,0.,"BAD_DT"
        x,y,z,yaw=pose
        if not all(math.isfinite(q) for q in pose):
            return self.stop("INVALID_POSE")
        s,error,road_width=self.route.nearest(x,y)
        safe_center=road_width/2.-.125-.08
        if safe_center<.1:
            return self.stop("ROAD_TOO_NARROW")
        # A single noisy sample must not latch a permanent OFF_TRACK,
        # while repeated physical road departures still stop the car.
        if error>safe_center+.03:
            self.offtrack_frames+=1
        else:
            self.offtrack_frames=0
        if self.offtrack_frames>=3:
            return self.stop("OFF_TRACK")
        if self.last_s is None:
            self.last_s=s
        else:
            advance=s-self.last_s
            if advance< -self.route.length/2.:advance+=self.route.length
            elif advance>self.route.length/2.:advance-=self.route.length
            if advance<-.50 or advance>max(1.,2.*self.max_speed*dt+.5):
                return self.stop("POSITION_JUMP_OR_REVERSE")
            self.travel+=max(0.,advance)
            self.last_s=s
            self.laps=int(self.travel/self.route.length)
            if self.changing:
                self.change_distance+=max(0.,advance)
                if self.change_distance>=LANE_SHIFT:
                    self.changing=False
                    self.lane=self.target
                    self.change_distance=LANE_SHIFT
        # Corner preview: merge side lane to the middle only if maneuver
        # remains feasible and 3->2/1->2 is a one-lane change.
        curvature=max(self.route.curvature(s+a) for a in (6.,18.,30.))
        if (not self.external_strategy and not self.changing and
                self.target!=1 and curvature>=.030):
            self.request_lane(1,"CORNER_CENTER")
        lookahead=clamp(2.5+.35*self.speed,3.,7.5)
        goal=self._point(s+lookahead,preview=lookahead)
        alpha=wrap(math.atan2(goal[1]-y,goal[0]-x)-yaw)
        distance=max(1.0,math.hypot(goal[0]-x,goal[1]-y))
        left_steer=math.atan2(2.*WHEELBASE*math.sin(alpha),distance)
        # +steer is right per operator contract; +yaw is left in Gazebo.
        desired_steer=-clamp(left_steer,-MAX_STEER,MAX_STEER)
        desired_steer=clamp(desired_steer,self.steer_right-MAX_STEER_RATE*dt,
                            self.steer_right+MAX_STEER_RATE*dt)
        current_curve=self.route.curvature(s)
        speed_cap=self.max_speed
        for d in range(0,37,3):
            k=self.route.curvature(s+d)
            if k<1e-5:continue
            corner_speed=min(self.max_speed, math.sqrt(MAX_LATERAL_A/k),
                             MAX_YAW_RATE/k)
            approach=math.sqrt(corner_speed**2+2.*MAX_BRAKE*max(0.,d-4.))
            speed_cap=min(speed_cap,approach)
        if curvature>.08:
            speed_cap=min(speed_cap,7.0)
        if abs(desired_steer)>.35:
            speed_cap=min(speed_cap,5.)
        elif abs(desired_steer)>.175:
            speed_cap=min(speed_cap,7.)
        if math.isfinite(traffic_speed_cap):
            speed_cap=min(speed_cap,max(0.,traffic_speed_cap))
        if self.changing:
            width=abs(float(self.route.meta["route_spacing_m"]))
            lane_speed=LANE_SHIFT*math.sqrt(MAX_LATERAL_A/max(.001,5.774*width))
            speed_cap=min(speed_cap,lane_speed)
        if self.speed<=0.:
            self.speed=clamp(speed_cap,0.,MAX_ACCEL*dt)
        else:
            self.speed=clamp(speed_cap,self.speed-MAX_BRAKE*dt,
                             self.speed+MAX_ACCEL*dt)
        self.speed=clamp(self.speed,0.,self.max_speed)
        rate_steer=math.atan2(MAX_YAW_RATE*WHEELBASE,max(.1,self.speed))
        self.steer_right=clamp(desired_steer,-min(MAX_STEER,rate_steer),
                               min(MAX_STEER,rate_steer))
        return self.speed,self.steer_right,"DRIVE"
