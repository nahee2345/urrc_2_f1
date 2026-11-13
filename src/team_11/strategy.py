"""TEAM 11 geometry, path following and tactical driving. No ROS dependency.

Inputs must be from real pose/velocity estimation in the *same* map frame.
Lane change and defensive moves always yield to collision / curvature limits.
"""
from __future__ import annotations
import csv
import math
from dataclasses import dataclass
from pathlib import Path

def clamp(x, lo, hi):
    return min(hi, max(lo, x))

def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))

@dataclass
class Vehicle:
    x: float
    y: float
    yaw: float
    speed: float
    stamp: float = 0.0
    name: str = ""

@dataclass
class Command:
    speed: float
    steer: float
    lane: int
    mode: str
    reason: str

class Track:
    def __init__(self, directory):
        self.lanes = {}
        for lane in (1, 2, 3):
            with (Path(directory) / f"lane_{lane}.csv").open(newline="") as f:
                rows = list(csv.DictReader(f))
            if len(rows) < 4:
                raise ValueError("Incomplete track")
            self.lanes[lane] = [
                {key: (row[key] == "True" if key == "safe_to_change_lane" else float(row[key]))
                 for key in ("x_m", "y_m", "z_m", "yaw_rad", "road_width_m",
                             "lane_offset_m", "safe_to_change_lane")}
                for row in rows]
        self.n = len(self.lanes[2])
        if any(len(v) != self.n for v in self.lanes.values()):
            raise ValueError("Lane samples are not aligned")
        self.ds = []
        self.arc = [0.0]
        center = self.lanes[2]
        for i in range(self.n):
            p, q = center[i], center[(i + 1) % self.n]
            segment = math.hypot(q["x_m"] - p["x_m"], q["y_m"] - p["y_m"])
            if segment < 1e-5:
                raise ValueError("Duplicate adjacent centerline points")
            self.ds.append(segment)
            self.arc.append(self.arc[-1] + segment)
        self.length = self.arc[-1]
        self.curvature = []
        for i in range(self.n):
            previous = center[(i - 1) % self.n]["yaw_rad"]
            following = center[(i + 1) % self.n]["yaw_rad"]
            distance = self.ds[(i - 1) % self.n] + self.ds[i]
            self.curvature.append(abs(wrap(following - previous)) / max(.01, distance))

    def nearest(self, x, y, last=None, window=70):
        indices = range(self.n) if last is None else (
            (last + k) % self.n for k in range(-window, window + 1))
        return min(indices, key=lambda i: (self.lanes[2][i]["x_m"] - x) ** 2 +
                                       (self.lanes[2][i]["y_m"] - y) ** 2)

    def advance(self, index, meters):
        remain = max(0.0, meters)
        i = index
        while remain > self.ds[i]:
            remain -= self.ds[i]
            i = (i + 1) % self.n
        return i

    def along(self, origin, other):
        return (self.arc[other] - self.arc[origin]) % self.length

    def lane_of(self, x, y, index):
        return min((1, 2, 3), key=lambda lane:
                   (self.lanes[lane][index]["x_m"] - x) ** 2 +
                   (self.lanes[lane][index]["y_m"] - y) ** 2)

    def corner(self, index, speed):
        # A lookahead horizon of three seconds; include transition into a corner.
        horizon = clamp(max(2.0, speed * 3.0), 2.0, 40.0)
        i = index
        distance = 0.0
        worst = 0.0
        while distance <= horizon:
            worst = max(worst, self.curvature[i])
            distance += self.ds[i]
            i = (i + 1) % self.n
        return worst

    def distance_to_corner(self, index, search_m=100.0, threshold=.08):
        """Forward track distance to next corner; 0 while inside a corner."""
        distance=0.0
        i=index
        for _ in range(self.n):
            if self.curvature[i]>=threshold:
                return distance
            if distance>search_m:
                break
            distance+=self.ds[i]
            i=(i+1)%self.n
        return math.inf

    def safe_change(self, index, speed):
        # Do not change lanes near a sharp bend, or where width is insufficient.
        end = self.advance(index, max(2.0, speed * 2.0))
        i = index
        for _ in range(self.n):
            if not self.lanes[2][i]["safe_to_change_lane"] or self.curvature[i] > .12:
                return False
            if i == end:
                break
            i = (i + 1) % self.n
        return True

class Planner:
    def __init__(self, track, spawn_lane=1, wheelbase=.32):
        self.track = track
        self.lane = spawn_lane
        self.target = spawn_lane
        self.mode = "JOIN"
        self.wheelbase = wheelbase
        self.last_index = None
        self.last_time = None
        self.last_steer = 0.0
        self.last_speed = 0.0
        self.lane_start = None
        self.pass_name = None
        self.passed_in_corner = False
        self.was_in_corner = False
        self.corner_exit_index = None

    def _relative(self, ego, other, ego_index):
        other_index = self.track.nearest(other.x, other.y)
        ahead = self.track.along(ego_index, other_index)
        if ahead <= self.track.length / 2:
            return ahead, other_index
        return -(self.track.length - ahead), other_index

    def _blocked(self, lane, ego, others, index, forward=4.0, backward=3.0):
        # All opponent poses occupy a safety envelope, including adjacent paths.
        for other in others:
            distance, oi = self._relative(ego, other, index)
            olane = self.track.lane_of(other.x, other.y, oi)
            if olane == lane and -backward < distance < forward:
                return True
        return False

    def _candidate(self, ego, others, index, avoid=None):
        choices = sorted((1, 2, 3), key=lambda j: (abs(j - self.lane), j))
        for lane in choices:
            if lane == self.lane or lane == avoid:
                continue
            if not self._blocked(lane, ego, others, index, 5.0, 4.0):
                return lane
        return None

    def _corner_speed(self, index, ego_speed):
        """8m/s at a bend, 10m/s after it, 15m/s on long straights.

        Braking begins before the bend using a 4m/s² approach envelope;
        speed ramps in step() still constrain the actual command change.
        """
        curvature = self.track.corner(index, ego_speed)
        distance = self.track.distance_to_corner(index)
        at_corner = self.track.curvature[index] >= .08
        if at_corner:
            self.was_in_corner = True
            self.corner_exit_index = None
        elif self.was_in_corner:
            self.was_in_corner = False
            self.corner_exit_index = index
        distance_since_exit = (self.track.along(self.corner_exit_index,index)
                               if self.corner_exit_index is not None else math.inf)
        long_straight = distance > 70.0 and distance_since_exit >= 12.0
        cruise = 15.0 if long_straight else 10.0
        # Five-metre buffer for speed convergence before the bend.
        approach = math.sqrt(8.0**2 + 2.0*4.0*max(0.0,distance-5.0))
        return curvature, min(cruise,approach)

    def step(self, ego, others, now, enabled=True):
        if not enabled:
            self.last_speed = 0.0
            self.last_steer = 0.0
            return Command(0.0, 0.0, self.lane, "WAIT_SIGNAL", "start gate closed")
        if not all(math.isfinite(v) for v in (ego.x, ego.y, ego.yaw, ego.speed, now)):
            return Command(0.0, 0.0, self.lane, "STOP", "nonfinite pose")
        if self.last_time is not None and now <= self.last_time:
            return Command(0.0, 0.0, self.lane, "STOP", "stale clock")
        dt = .05 if self.last_time is None else clamp(now - self.last_time, .01, .2)
        self.last_time = now
        index = self.track.nearest(ego.x, ego.y, self.last_index)
        if self.last_index is not None and math.hypot(
                ego.x - self.track.lanes[2][index]["x_m"],
                ego.y - self.track.lanes[2][index]["y_m"]) > 3.0:
            return Command(0.0, 0.0, self.lane, "STOP", "lost map localization")
        self.last_index = index
        curvature, corner_limit = self._corner_speed(index, max(ego.speed, 1.0))
        sharp = curvature > .12
        forward = []
        rear = []
        for car in others:
            if car.name == ego.name or now - car.stamp > .6 or car.stamp > now + .1:
                continue
            dist, oi = self._relative(ego, car, index)
            opponent_lane = self.track.lane_of(car.x, car.y, oi)
            closing = ego.speed - car.speed
            if 0.0 <= dist < 30 and (opponent_lane == self.lane or
                                    opponent_lane == self.target):
                forward.append((dist, car, closing))
            if -30 < dist < 0:
                rear.append((-dist, car, car.speed - ego.speed, opponent_lane))
        forward.sort(key=lambda x: x[0])
        rear.sort(key=lambda x: x[0])
        nearest = forward[0] if forward else None
        front_ttc = (nearest[0] - 1.0) / nearest[2] if nearest and nearest[2] > .05 else math.inf
        threat = next(((gap, car, closing, lane) for gap, car, closing, lane in rear
                       if closing > .05 and (gap - 1.0) / closing <= 5.0), None)
        can_change = self.track.safe_change(index, ego.speed) and not sharp
        reason = "follow lane"
        # Two-stage maneuver holds lane 2 for a *minimum* of two seconds.
        if self.target != self.lane and self.lane_start is not None:
            target_point = self.track.lanes[self.target][index]
            if math.hypot(ego.x - target_point["x_m"], ego.y - target_point["y_m"]) < .22:
                self.lane = self.target
                if self.lane == 2 and self.pass_name is not None:
                    self.lane_start = now
                else:
                    self.lane_start = None
        if self.mode == "PASS" and self.pass_name:
            target_car = next((o for o in others if o.name == self.pass_name), None)
            if target_car:
                longitudinal, _ = self._relative(ego, target_car, index)
                if longitudinal < -2.0:
                    self.pass_name = None
                    self.mode = "CRUISE"
        if self.mode == "DEFEND" and (not threat or sharp):
            self.mode = "CRUISE"
        if sharp:
            self.passed_in_corner = any(gap < 3.0 for gap, _, _, _ in rear)
            if self.mode == "DEFEND":
                self.mode = "CRUISE"
        # In corners the centre line has priority over tactical manoeuvres.
        # Do not force a return through an occupied centre lane.
        if sharp and self.target != 2 and not self._blocked(2, ego, others, index, 5.0, 4.0):
            self.target = 2
            self.lane_start = now
            self.mode = "CRUISE"
            self.pass_name = None
            reason = "corner: merge to centre lane"
        if can_change and self.target == self.lane:
            if self.mode == "PASS" and self.lane == 2 and self.pass_name is not None and (
                    self.lane_start is not None and now - self.lane_start >= 2.0):
                dest = self._candidate(ego, others, index)
                if dest is not None:
                    self.target = dest
                    self.lane_start = now
                    reason = "complete staged passing move"
            elif front_ttc <= 3.0 and nearest is not None:
                dest = self._candidate(ego, others, index)
                # Never cross an occupied middle lane to reach the far lane.
                if dest is not None and abs(dest - self.lane) == 2:
                    if self._blocked(2, ego, others, index, 5.0, 4.0):
                        dest = None
                    else:
                        dest = 2
                if dest is not None:
                    self.target = dest
                    self.pass_name = nearest[1].name
                    self.mode = "PASS"
                    self.lane_start = now
                    reason = "front TTC <= 3s"
            elif threat and not forward:
                # Defensive positioning is optional: the same safety guard as passing applies.
                _, _, _, predicted_lane = threat
                if predicted_lane != self.lane and abs(predicted_lane - self.lane) == 1:
                    if not self._blocked(predicted_lane, ego, others, index, 5.0, 5.0):
                        self.target = predicted_lane
                        self.mode = "DEFEND"
                        self.lane_start = now
                        reason = "rear TTC <= 5s"
        if self.mode == "JOIN":
            self.mode = "CRUISE"
        if sharp and reason != "corner: merge to centre lane":
            reason = "corner safety restriction"
        # Pure pursuit on target lane; gradually growing lookahead damps oscillations.
        lookahead = clamp(1.5 + 0.6 * max(ego.speed, 0.0), 1.5, 7.0)
        aim_index = self.track.advance(index, lookahead)
        point = self.track.lanes[self.target][aim_index]
        dx, dy = point["x_m"] - ego.x, point["y_m"] - ego.y
        local_y = -math.sin(ego.yaw) * dx + math.cos(ego.yaw) * dy
        distance_sq = max(.25, dx*dx + dy*dy)
        desired = math.atan2(2.0 * self.wheelbase * local_y, distance_sq)
        desired = clamp(desired, -math.radians(30), math.radians(30))
        steer = clamp(desired, self.last_steer - math.radians(90) * dt,
                      self.last_steer + math.radians(90) * dt)
        self.last_steer = steer
        desired_speed = corner_limit
        if nearest and (front_ttc <= 3.0 or self.mode == "PASS"):
            # Cap by stopping headway when path is not yet physically clear.
            if self.target == self.lane or self._blocked(self.target, ego, others, index):
                desired_speed = min(desired_speed, max(0.0, nearest[1].speed +
                                                    .5 * (nearest[0] - 2.0)))
        if self.mode in ("PASS", "DEFEND") and not sharp:
            desired_speed = min(15.0, desired_speed + .5)
        # Separate braking and acceleration slopes.
        desired_speed = clamp(desired_speed, self.last_speed - 4.0 * dt,
                              self.last_speed + 3.5 * dt)
        self.last_speed = desired_speed
        return Command(desired_speed, steer, self.target, self.mode, reason)
