"""TEAM 11: three wall-clear routes and persistent, smooth lane changes.

Uses immutable URRC centerline CSV generated from Monza/Hungaroring.
No road/mesh/world inputs are modified.
"""
from __future__ import annotations
import csv
import math
from dataclasses import dataclass
from pathlib import Path

TRACKS = ("monza", "hungaroring")
CAR_WIDTH_M = .25
WALL_MARGIN_M = .22
EXTRA_CLEARANCE_M = .12
BASE_SPEED_MPS = 10.0
STEER_LIMIT_DEG = 10.0
SLOW_FACTOR = .8
LANE_CHANGE_LENGTH_M = 9.0
WHEELBASE_M = .33


@dataclass(frozen=True)
class Waypoint:
    s: float
    x: float
    y: float
    z: float
    yaw: float
    width: float


def load_centerline(file):
    with Path(file).open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 50:
        raise ValueError("Insufficient centerline points")
    xyz = [(float(r["x_m"]), float(r["y_m"]), float(r["z_m"])) for r in rows]
    distance = [0.]
    for a, b in zip(xyz, xyz[1:]):
        distance.append(distance[-1] + math.dist(a, b))
    data = [
        Waypoint(s, *p, float(r["yaw_rad"]), float(r["road_width_m"]))
        for s, p, r in zip(distance, xyz, rows)
    ]
    if min(p.width for p in data) <= CAR_WIDTH_M + 2 * (WALL_MARGIN_M + EXTRA_CLEARANCE_M):
        raise ValueError("Road too narrow for three routes")
    return data


def lane_spacing(points):
    # Reported road width is full width, collision vehicle width is .25m.
    # Offset from centerline must leave outer wall clearance on both sides.
    return min(p.width for p in points) / 2 - CAR_WIDTH_M / 2 - WALL_MARGIN_M - EXTRA_CLEARANCE_M


def generate_lanes(points):
    spacing = lane_spacing(points)
    # left, middle, right ordered by signed offset
    lanes = []
    for offset in (spacing, 0., -spacing):
        lanes.append([
            (p.s, p.x - offset * math.sin(p.yaw),
             p.y + offset * math.cos(p.yaw), p.z, p.yaw)
            for p in points
        ])
    return lanes


def export_routes(points, folder):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    lanes = generate_lanes(points)
    for i, route in enumerate(lanes):
        with (folder / f"lane_{i}.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("s_m", "x_m", "y_m", "z_m", "yaw_rad"))
            writer.writerows(route)
    return lane_spacing(points)


def closest_centerline_index(points, x, y):
    return min(range(len(points)), key=lambda i: (points[i].x-x)**2 + (points[i].y-y)**2)


def closest_lane(points, x, y):
    lanes = generate_lanes(points)
    i = closest_centerline_index(points, x, y)
    return min(range(3), key=lambda j: (lanes[j][i][1]-x)**2 + (lanes[j][i][2]-y)**2)


def smoothstep5(t):
    t = max(0., min(1., t))
    return t*t*t*(10 + t*(-15 + 6*t))


def interpolated_offset(source, target, progress):
    return source + (target-source) * smoothstep5(progress / LANE_CHANGE_LENGTH_M)


def commanded_speed(steering_rad):
    return BASE_SPEED_MPS * (SLOW_FACTOR if abs(math.degrees(steering_rad)) > STEER_LIMIT_DEG else 1.)


def projected_obstacle(points, x, y):
    i = closest_centerline_index(points, x, y)
    p = points[i]
    dx, dy = x-p.x, y-p.y
    signed = -dx*math.sin(p.yaw) + dy*math.cos(p.yaw)
    return p.s, signed


def choose_neighbor(active_lane, blocked, lane_distance=None):
    """Never jump across blocked middle lane. Retain selected new lane."""
    if active_lane not in blocked:
        return active_lane
    options = [j for j in (active_lane-1, active_lane+1) if 0 <= j <= 2 and j not in blocked]
    if not options:
        return None
    if lane_distance is not None:
        return min(options, key=lambda j: (lane_distance[j], abs(j-active_lane)))
    return options[0]


def blocked_lanes(points, ego_s, obstacles, *, horizon_m=35., obstacle_radius=.25):
    lap = points[-1].s + math.dist(
        (points[-1].x, points[-1].y), (points[0].x, points[0].y))
    spacing = lane_spacing(points)
    blocked = set()
    for x, y in obstacles:
        obs_s, offset = projected_obstacle(points, x, y)
        forward = (obs_s - ego_s) % lap
        if not 1.0 < forward < horizon_m:
            continue
        for lane, lane_offset in enumerate((spacing, 0., -spacing)):
            if abs(offset-lane_offset) < obstacle_radius + CAR_WIDTH_M/2 + .12:
                blocked.add(lane)
    return blocked


class LaneDecision:
    """One-way changes; no forced return to the previous lane."""
    def __init__(self, lane):
        if lane not in (0, 1, 2):
            raise ValueError(lane)
        self.lane = lane
        self.target = lane
        self.progress_m = LANE_CHANGE_LENGTH_M

    def update(self, blocked, distance_m):
        self.progress_m = min(LANE_CHANGE_LENGTH_M,
                              self.progress_m + max(0., distance_m))
        # No second maneuver before first finishes.
        if self.progress_m < LANE_CHANGE_LENGTH_M:
            return self.target
        next_lane = choose_neighbor(self.target, blocked)
        if next_lane is None:
            return None  # brake instead of driving into occupied lane
        if next_lane != self.target:
            self.lane, self.target = self.target, next_lane
            self.progress_m = 0.
        return self.target

    def offset(self, spacing):
        offsets = (spacing, 0., -spacing)
        return interpolated_offset(offsets[self.lane],
                                   offsets[self.target], self.progress_m)
