"""Portable three-route policy from own forward LaserScan only.

No track CSV, world model poses, GPS, timing or opponent state is used.
The three routes are inferred afresh from the measured drivable corridor.
All speed limits are TARGETS, not proof of physical stable driving.
"""
from dataclasses import dataclass
import math
import numpy as np

CAR_HALF_WIDTH = 0.125
EDGE_MARGIN = 0.11
MAX_SPEED = 15.0
FRONT_MIN = 10.0
CORNER_ENTER = 0.030
CORNER_EXIT = 0.022


def clamp(x, a, b):
    return max(a, min(b, x))


def quintic(x):
    t = clamp(x, 0.0, 1.0)
    return t*t*t*(10.0 + t*(-15.0 + 6.0*t))


def curve_speed(curvature, corner_mode):
    if not corner_mode:
        return MAX_SPEED
    return clamp(7.0 - 2.0*(abs(curvature)-CORNER_ENTER)/0.05, 5.0, 7.0)


def ramp_speed(last, target, dt, accel=2.0, decel=4.0):
    return clamp(target, max(0.0, last-decel*dt), last+accel*dt)


def change_distance(speed, lane_width):
    return max(18.0, max(0.0, speed)*math.sqrt(5.774*lane_width/2.5))


@dataclass
class Corridor:
    left: float
    right: float
    far_left: float
    far_right: float
    width: float
    spacing: float

    @property
    def center(self):
        return (self.left+self.right)*0.5

    @property
    def far_center(self):
        return (self.far_left+self.far_right)*0.5

    def lanes(self):
        c = self.center
        return [c+self.spacing, c, c-self.spacing]


@dataclass
class PlanningResult:
    speed: float
    steer: float    # competition sign convention: RIGHT positive
    lane: int       # 0/1/2 respectively left/middle/right
    corner: bool
    state: str
    ahead_m: float


def _bound_at(x, y, a, b):
    mask = (x>=a)&(x<=b)
    left = y[mask & (y>0.35)]
    right = y[mask & (y<-0.35)]
    # Close returns rather than mean averages avoid trusting invisible walls.
    return (float(np.quantile(left, .12)) if left.size>=3 else None,
            float(np.quantile(right, .88)) if right.size>=3 else None)


def interpret_scan(ranges, angle_min, angle_increment, range_min, range_max):
    """Own LaserScan -> front obstacle points and coarse road corridor."""
    data = np.asarray(ranges, dtype=np.float64)
    theta = angle_min + np.arange(data.size)*angle_increment
    valid = np.isfinite(data) & (data>=max(.1,range_min)) & (data<=range_max)
    valid &= data < 54.0  # no-return at max range is not a wall
    x = data[valid]*np.cos(theta[valid])
    y = data[valid]*np.sin(theta[valid])
    if not x.size:
        return None, x, y
    near = _bound_at(x,y,.35,2.5)
    far = _bound_at(x,y,2.5,7.0)
    if near[0] is None or near[1] is None:
        return None, x, y
    left,right=near
    width=left-right
    if not .70<=width<=9.:
        return None, x, y
    # Do not invent far boundaries; use nearest measured safe geometry
    # until the LiDAR can see around the corner.
    far_left=far[0] if far[0] is not None else left
    far_right=far[1] if far[1] is not None else right
    spacing=clamp((width-2*CAR_HALF_WIDTH-2*EDGE_MARGIN)*.5, .20, .75)
    corridor=Corridor(left,right,far_left,far_right,width,spacing)
    return corridor,x,y


def clearance(x, y, lateral, depth=50.0):
    mask=(x>.45)&(x<depth)&(np.abs(y-lateral)<.31)
    return float(np.min(x[mask])) if np.any(mask) else float("inf")


class ThreeLanePlanner:
    """Retains TEAM11 policy, but evaluates sensor-defined lanes, not map CSV."""
    def __init__(self):
        self.lane=None
        self.source_lane=None
        self.destination=None
        self.progress=0.0
        self.length=18.0
        self.corner=False
        self.last_speed=0.0
        self.last_front=None
        self.last_tick=None
        self.last_change_distance=100.0
        self.total_distance=0.0
        self.home=None
        self.mode="STOP"

    def reset(self):
        self.__init__()

    def update(self, ranges, angle_min, angle_increment, range_min,
               range_max, dt):
        dt = clamp(float(dt), 0.001, 0.2)
        corridor,x,y=interpret_scan(
            ranges,angle_min,angle_increment,range_min,range_max)
        if corridor is None:
            self.last_speed=ramp_speed(self.last_speed,0.0,dt)
            return PlanningResult(0.0,0.0,self.lane if self.lane is not None else 1,
                                  self.corner,"NO_ROAD",0.0)
        centers=corridor.lanes()
        if self.lane is None:
            # Centerline is in local ego frame. If it lies to the left, our
            # actual grid is on the right; vice versa.
            self.lane = 2 if corridor.center>.15 else 0 if corridor.center<-.15 else 1
            self.home=self.lane
        near_center=corridor.center
        far_center=corridor.far_center
        curvature=abs(2.0*(far_center-near_center)/(6.0**2))
        if not self.corner and curvature>=CORNER_ENTER:
            self.corner=True
        elif self.corner and curvature<=CORNER_EXIT:
            self.corner=False
        target=curve_speed(curvature,self.corner)
        if self.destination is not None:
            self.progress += self.last_speed*dt
            if self.progress>=self.length:
                self.lane=self.destination
                self.source_lane=None
                self.destination=None
                self.progress=self.length
        front_dist=clearance(x,y,centers[self.lane],50.0)
        front_arr = [clearance(x,y,c,50.) for c in centers]

        # Conservative front-collision speed cap with 10m headway.
        safe_stop=math.sqrt(max(0., 2.0*4.0*max(0.,front_dist-FRONT_MIN)))
        target=min(target,safe_stop)

        # Prevent zigzag. Do not begin another maneuver until finished.
        if self.destination is None and front_dist<35. and self.last_change_distance>24.:
            closing = 0.0
            if self.last_front is not None and math.isfinite(front_dist):
                closing=max(0.,(self.last_front-front_dist)/dt)
            ttc=(front_dist-2.0)/closing if closing>0.1 else float("inf")
            if closing>=2.0 and ttc<=6.0:
                for next_lane in (self.lane-1,self.lane+1):
                    if not 0<=next_lane<=2:
                        continue
                    # The middle lane naturally prefers left when both clear.
                    if next_lane==self.lane-1 or self.lane==0:
                        pass
                    if front_arr[next_lane]<max(12.,self.last_speed*2.0):
                        continue
                    # Cannot see a verified safe path around very tight turns.
                    if curvature>=.08:
                        continue
                    self.source_lane=self.lane
                    self.destination=next_lane
                    self.progress=0.
                    self.length=change_distance(self.last_speed,corridor.spacing)
                    self.last_change_distance=0.
                    break
        self.last_front=front_dist if math.isfinite(front_dist) else None
        self.last_change_distance+=self.last_speed*dt
        self.total_distance+=self.last_speed*dt

        if self.destination is not None:
            p=quintic(self.progress/max(self.length,1.))
            lateral=(1-p)*centers[self.source_lane]+p*centers[self.destination]
            self.mode="SHIFT"
        else:
            lateral=centers[self.lane]
            self.mode="CORNER" if self.corner else "CRUISE"

        # Pure pursuit on LiDAR-derived local corridor, never a preloaded map.
        lookahead=clamp(4.+self.last_speed*.4,4.,11.)
        look_y=lateral + clamp(far_center-near_center,-1.,1.)*.6
        geometric_curvature=2.*look_y/(lookahead**2+look_y**2)
        steer=clamp(-math.atan(.34*geometric_curvature),-.52,.52)
        if abs(steer)>math.radians(10.):
            target=min(target,7.)
        if abs(steer)>math.radians(20.):
            target=min(target,5.)
        self.last_speed=ramp_speed(self.last_speed,target,dt)
        return PlanningResult(self.last_speed,steer,self.destination
                              if self.destination is not None else self.lane,
                              self.corner,self.mode,front_dist)
