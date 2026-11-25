"""LiDAR-only opponent detection, tracking, and conservative race strategy.

All inputs and outputs use the vehicle/LiDAR frame.  This module deliberately
has no access to world poses, maps, or other vehicles' topics.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from collections import deque
import math
from typing import Sequence

from .geometry import Point2, median
from .lidar_processing import ScanPoint
from .soft_centering import CorridorMeasurement
from .wall_tracker import PolynomialWall, WallObservation


@dataclass(frozen=True)
class OpponentCandidate:
    x: float
    y: float
    distance: float
    width: float
    bearing: float
    position: str
    beam_indices: tuple[int, ...]
    timestamp: float | None = None
    sources: tuple[str, ...] = ()
    extent: float = 0.0
    point_count: int = 0
    wall_distance: float | None = None
    tracking_only: bool = False
    radial_span: float = 0.0


@dataclass(frozen=True)
class TrackedOpponent:
    track_id: int
    x: float
    y: float
    distance: float
    width: float
    bearing: float
    relative_vx: float
    relative_vy: float
    closing_rate: float
    position: str
    age: int
    missed: int = 0
    last_observed_x: float = 0.0


@dataclass(frozen=True)
class RaceDecision:
    race_state: str
    selected_pass_side: str
    desired_lateral_offset: float
    speed_cap: float
    left_pass_clearance: float
    right_pass_clearance: float
    closest_front_distance: float
    closest_rear_distance: float
    front_closing_rate: float
    rear_closing_rate: float
    primary_opponent_x: float
    primary_opponent_y: float
    current_wall_clearance: float
    current_opponent_clearance: float
    predicted_pass_clearance: float
    remaining_pass_gap: float
    opponent_lateral_velocity: float
    pass_safety_state: str


def filter_selected_pass_obstacle(
    obstacles: Sequence[Point2],
    opponent: TrackedOpponent | None,
    sensor_x_m: float = 0.0,
    sensor_y_m: float = 0.0,
) -> tuple[Point2, ...]:
    """Remove only the selected car from primary-frame corridor-stop points."""

    if opponent is None:
        return tuple(obstacles)
    center = Point2(opponent.x, opponent.y)
    exclusion_radius = max(0.32, 0.5 * opponent.width + 0.20)
    return tuple(
        point for point in obstacles
        if Point2(point.x + sensor_x_m, point.y + sensor_y_m).distance(center)
        > exclusion_radius
    )


def mask_confirmed_opponents(
    points: Sequence[ScanPoint],
    opponents: Sequence[TrackedOpponent],
    longitudinal_margin_m: float = 0.20,
    lateral_margin_m: float = 0.14,
    nominal_length_m: float = 0.55,
    nominal_width_m: float = 0.25,
    sensor_x_m: float = 0.0,
    sensor_y_m: float = 0.0,
) -> tuple[list[ScanPoint], tuple[int, ...]]:
    """Remove confirmed opponent returns before the final wall fit.

    The mask is deliberately local to each temporally confirmed track.  With
    no confirmed tracks the original list object is copied unchanged, keeping
    the solo perception path numerically identical.
    """

    if not opponents:
        return list(points), ()
    half_length = 0.5 * max(0.10, nominal_length_m) + max(0.0, longitudinal_margin_m)
    kept: list[ScanPoint] = []
    removed: list[int] = []
    for item in points:
        masked = False
        for opponent in opponents:
            half_width = 0.5 * max(nominal_width_m, opponent.width)
            half_width += max(0.0, lateral_margin_m)
            if (
                abs(item.point.x + sensor_x_m - opponent.x) <= half_length
                and abs(item.point.y + sensor_y_m - opponent.y) <= half_width
            ):
                masked = True
                break
        if masked:
            removed.append(item.beam_index)
        else:
            kept.append(item)
    return kept, tuple(removed)


def translate_scan_points(
    points: Sequence[ScanPoint], x_m: float, y_m: float = 0.0
) -> list[ScanPoint]:
    """Translate a horizontal scan into the common vehicle/base frame."""

    return [
        ScanPoint(
            item.beam_index,
            item.range,
            Point2(item.point.x + x_m, item.point.y + y_m),
        )
        for item in points
    ]


def transform_scan_points(
    points: Sequence[ScanPoint], x_m: float, yaw_rad: float, y_m: float = 0.0
) -> list[ScanPoint]:
    """Rigid sensor-to-base transform, including a rear-facing scan's yaw."""
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    return [ScanPoint(item.beam_index, item.range,
                      Point2(x_m + cosine * item.point.x - sine * item.point.y,
                             y_m + sine * item.point.x + cosine * item.point.y))
            for item in points]


def translate_wall_observation(
    observation: WallObservation, x_m: float, y_m: float = 0.0
) -> WallObservation:
    """Express primary-frame polynomial walls in the vehicle/base frame."""

    def translated(wall: PolynomialWall | None) -> PolynomialWall | None:
        if wall is None:
            return None
        a, b, c = wall.coefficients
        # y_p = a + b*x_p + c*x_p^2; x_p = x_base - x_m.
        coefficients = (
            a - b * x_m + c * x_m * x_m + y_m,
            b - 2.0 * c * x_m,
            c,
        )
        return PolynomialWall(
            wall.side,
            coefficients,
            wall.x_min + x_m,
            wall.x_max + x_m,
            wall.point_count,
            wall.residual_m,
        )

    return WallObservation(
        translated(observation.left),
        translated(observation.right),
        observation.track_width_m,
    )


def fuse_opponent_candidates(
    primary: Sequence[OpponentCandidate],
    secondary: Sequence[OpponentCandidate],
    association_distance_m: float = 0.60,
    fragment_distance_m: float = 0.50,
) -> list[OpponentCandidate]:
    """Associate simultaneous sensor candidates into physical opponents.

    Candidate point counts are used as deterministic quality weights. An
    unmatched observation from either sensor remains eligible for tracking.
    """

    unmatched_secondary = set(range(len(secondary)))
    fused: list[OpponentCandidate] = []
    for first in sorted(primary, key=lambda item: item.distance):
        match = None
        match_distance = math.inf
        for index in unmatched_secondary:
            second = secondary[index]
            distance = math.hypot(first.x - second.x, first.y - second.y)
            if (distance <= association_distance_m and distance < match_distance
                    and abs(first.y - second.y) <= 0.30):
                match = index
                match_distance = distance
        if match is None:
            fused.append(first)
            continue
        unmatched_secondary.remove(match)
        second = secondary[match]
        first_quality = max(1, len(first.beam_indices))
        second_quality = max(1, len(second.beam_indices))
        total_quality = first_quality + second_quality
        x = (first_quality * first.x + second_quality * second.x) / total_quality
        y = (first_quality * first.y + second_quality * second.y) / total_quality
        best = first if first_quality >= second_quality else second
        if x > 0.30:
            position = "FRONT"
        elif x < -0.30:
            position = "REAR"
        else:
            position = "LEFT_SIDE" if y >= 0.0 else "RIGHT_SIDE"
        fused.append(OpponentCandidate(
            x=x,
            y=y,
            distance=math.hypot(x, y),
            width=max(first.width, second.width),
            bearing=math.atan2(y, x),
            position=position,
            beam_indices=best.beam_indices,
            timestamp=max((t for t in (first.timestamp, second.timestamp) if t is not None), default=None),
            sources=tuple(sorted(set(first.sources + second.sources))),
            extent=max(first.extent, second.extent),
            point_count=first.point_count + second.point_count,
            wall_distance=min((d for d in (first.wall_distance, second.wall_distance) if d is not None), default=None),
            tracking_only=first.tracking_only and second.tracking_only,
            radial_span=max(first.radial_span, second.radial_span),
        ))
    fused.extend(secondary[index] for index in sorted(unmatched_secondary))

    # A sparse F1 body can resolve as separate wheel/body fragments in one
    # sensor while the other sensor sees a single cluster. Coalesce fragments
    # within one vehicle footprint so one physical car cannot create two
    # temporal tracks. The 0.536 m length is NOT its lateral width: cars
    # abreast can safely have centres less than 0.50 m apart.
    coalesced: list[OpponentCandidate] = []
    for candidate in sorted(fused, key=lambda item: item.distance):
        match = next(
            (
                index for index, existing in enumerate(coalesced)
                if math.hypot(candidate.x - existing.x, candidate.y - existing.y)
                <= fragment_distance_m
                and abs(candidate.y - existing.y) <= 0.30
            ),
            None,
        )
        if match is None:
            coalesced.append(candidate)
            continue
        existing = coalesced[match]
        existing_quality = max(1, len(existing.beam_indices))
        candidate_quality = max(1, len(candidate.beam_indices))
        total_quality = existing_quality + candidate_quality
        x = (
            existing_quality * existing.x + candidate_quality * candidate.x
        ) / total_quality
        y = (
            existing_quality * existing.y + candidate_quality * candidate.y
        ) / total_quality
        if x > 0.30:
            position = "FRONT"
        elif x < -0.30:
            position = "REAR"
        else:
            position = "LEFT_SIDE" if y >= 0.0 else "RIGHT_SIDE"
        coalesced[match] = OpponentCandidate(
            x=x,
            y=y,
            distance=math.hypot(x, y),
            width=max(existing.width, candidate.width),
            bearing=math.atan2(y, x),
            position=position,
            beam_indices=tuple(sorted(set(
                (*existing.beam_indices, *candidate.beam_indices)
            ))),
            timestamp=max((t for t in (existing.timestamp, candidate.timestamp) if t is not None), default=None),
            sources=tuple(sorted(set(existing.sources + candidate.sources))),
            extent=max(existing.extent, candidate.extent),
            point_count=existing.point_count + candidate.point_count,
            wall_distance=min((d for d in (existing.wall_distance, candidate.wall_distance) if d is not None), default=None),
            tracking_only=existing.tracking_only and candidate.tracking_only,
            radial_span=max(existing.radial_span, candidate.radial_span),
        )
    return sorted(coalesced, key=lambda item: item.distance)


def full_scan_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    sensor_range_min: float,
    sensor_range_max: float,
    configured_range_min: float,
    configured_range_max: float,
) -> list[ScanPoint]:
    """Convert the complete 360-degree scan without changing wall perception."""

    if not math.isfinite(angle_min) or not math.isfinite(angle_increment):
        return []
    minimum = max(sensor_range_min, configured_range_min)
    maximum = min(sensor_range_max, configured_range_max)
    if angle_increment == 0.0 or minimum >= maximum:
        return []
    result = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not minimum <= distance <= maximum:
            continue
        angle = math.atan2(
            math.sin(angle_min + index * angle_increment),
            math.cos(angle_min + index * angle_increment),
        )
        point = Point2(distance * math.cos(angle), distance * math.sin(angle))
        result.append(ScanPoint(index, distance, point))
    return result


class OpponentDetector:
    """Remove fitted walls, then accept only compact in-corridor clusters."""

    def __init__(
        self,
        wall_exclusion_m: float = 0.24,
        cluster_gap_m: float = 0.24,
        maximum_cluster_beam_gap: int = 12,
        minimum_points: int = 1,
        minimum_width_m: float = 0.0,
        maximum_width_m: float = 0.75,
        fragment_merge_gap_m: float = 0.34,
        maximum_radial_span_m: float = 0.70,
        corridor_margin_m: float = 0.32,
        maximum_centerline_distance_m: float = 0.70,
        maximum_distance_m: float = 12.0,
        allow_rear_candidates: bool = True,
    ) -> None:
        self.wall_exclusion = wall_exclusion_m
        self.cluster_gap = cluster_gap_m
        self.maximum_cluster_beam_gap = maximum_cluster_beam_gap
        self.minimum_points = minimum_points
        self.minimum_width = minimum_width_m
        self.maximum_width = maximum_width_m
        self.fragment_merge_gap = fragment_merge_gap_m
        self.maximum_radial_span = maximum_radial_span_m
        self.corridor_margin = corridor_margin_m
        self.maximum_centerline_distance = maximum_centerline_distance_m
        self.maximum_distance = maximum_distance_m
        self.allow_rear_candidates = allow_rear_candidates

    def _merge_fragments(
        self, clusters: Sequence[Sequence[ScanPoint]]
    ) -> list[list[ScanPoint]]:
        """Join nearby pieces of one sparse vehicle without reaching a wall."""

        merged = [list(cluster) for cluster in clusters]
        changed = True
        while changed:
            changed = False
            for first_index in range(len(merged)):
                if changed:
                    break
                first = merged[first_index]
                for second_index in range(first_index + 1, len(merged)):
                    second = merged[second_index]
                    first_center = Point2(
                        median(item.point.x for item in first),
                        median(item.point.y for item in first),
                    )
                    second_center = Point2(
                        median(item.point.x for item in second),
                        median(item.point.y for item in second),
                    )
                    combined = [*first, *second]
                    xs = [item.point.x for item in combined]
                    ys = [item.point.y for item in combined]
                    extent = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
                    if (
                        first_center.distance(second_center) <= self.fragment_merge_gap
                        and abs(first_center.x - second_center.x) <= 0.45
                        and extent <= self.maximum_width
                    ):
                        merged[first_index] = sorted(
                            combined, key=lambda item: item.beam_index
                        )
                        del merged[second_index]
                        changed = True
                        break
        return merged

    @staticmethod
    def _wall_y(wall, x: float) -> float | None:
        if wall is None:
            return None
        # Extend the local fit forward for conservative wall rejection.  Curved
        # wall fragments often lie just beyond the fit's accepted x range.
        if x >= -0.5:
            return wall.y(x)
        return None

    def _inside_corridor(self, point: Point2, observation: WallObservation) -> bool:
        left = self._wall_y(observation.left, point.x)
        right = self._wall_y(observation.right, point.x)
        if left is not None and right is not None:
            return right + self.corridor_margin < point.y < left - self.corridor_margin
        if point.x < -0.5:
            half_width = 0.5 * observation.track_width_m - self.corridor_margin
            return abs(point.y) < half_width
        half_width = 0.5 * observation.track_width_m - self.corridor_margin
        if left is not None:
            return left - observation.track_width_m + self.corridor_margin < point.y < left - self.corridor_margin
        if right is not None:
            return right + self.corridor_margin < point.y < right + observation.track_width_m - self.corridor_margin
        # Forward detections are actionable only when wall perception proves
        # that they lie inside the drivable corridor.
        return False

    def _near_wall(self, point: Point2, observation: WallObservation) -> bool:
        for wall in (observation.left, observation.right):
            wall_y = self._wall_y(wall, point.x)
            if wall_y is not None and abs(point.y - wall_y) <= self.wall_exclusion:
                return True
        # Rear wall fits are unavailable because the solo wall tracker is
        # intentionally forward-only.  Remove the remembered corridor edges.
        if point.x < -0.10:
            half_width = 0.5 * observation.track_width_m
            if abs(abs(point.y) - half_width) <= self.wall_exclusion:
                return True
        return False

    def detect(
        self,
        points: Sequence[ScanPoint],
        observation: WallObservation,
        corridor_centerline: Sequence[Point2] = (),
        *,
        dedicated_rear_sensor: bool = False,
    ) -> list[OpponentCandidate]:
        if dedicated_rear_sensor:
            # Inspect complete measured chains BEFORE clipping to the remembered
            # corridor. A curved wall crossing that mask otherwise becomes a
            # compact, apparently vehicle-sized endpoint. This is not a wall fit
            # and uses the existing compact-object width and clustering limits.
            raw = [p for p in points if 0.30 <= p.range <= self.maximum_distance]
            chains: list[list[ScanPoint]] = []
            for item in raw:
                if (not chains
                        or item.beam_index - chains[-1][-1].beam_index > self.maximum_cluster_beam_gap
                        or item.point.distance(chains[-1][-1].point) > self.cluster_gap):
                    chains.append([])
                chains[-1].append(item)
            long_chain_beams = set()
            for chain in chains:
                xs = [p.point.x for p in chain]
                ys = [p.point.y for p in chain]
                if math.hypot(max(xs)-min(xs), max(ys)-min(ys)) > self.maximum_width:
                    long_chain_beams.update(p.beam_index for p in chain)
            points = [p for p in points if p.beam_index not in long_chain_beams]
        usable = [
            item for item in points
            if 0.30 <= item.range <= self.maximum_distance
            and self._inside_corridor(item.point, observation)
            and not self._near_wall(item.point, observation)
        ]
        if not usable:
            return []

        clusters: list[list[ScanPoint]] = [[usable[0]]]
        for item in usable[1:]:
            previous = clusters[-1][-1]
            if (
                item.beam_index - previous.beam_index <= self.maximum_cluster_beam_gap
                and item.point.distance(previous.point) <= self.cluster_gap
            ):
                clusters[-1].append(item)
            else:
                clusters.append([item])

        candidates = []
        for cluster in self._merge_fragments(clusters):
            if len(cluster) < self.minimum_points:
                continue
            xs = [item.point.x for item in cluster]
            ys = [item.point.y for item in cluster]
            x, y = median(xs), median(ys)
            extent = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            width = max(0.02, max(ys) - min(ys))
            radial_span = max(item.range for item in cluster) - min(item.range for item in cluster)
            if not self.minimum_width <= extent <= self.maximum_width:
                continue
            if radial_span > self.maximum_radial_span:
                continue
            if x > 0.30:
                # Compact ends of incomplete wall chains look exactly like a
                # sparse car. Require the primary wall-derived centerline to
                # prove the candidate's longitudinal and lateral corridor.
                if not corridor_centerline:
                    continue
                nearest = min(corridor_centerline, key=lambda point: abs(point.x - x))
                if (
                    abs(nearest.x - x) > 1.0
                    or nearest.distance(Point2(x, y)) > self.maximum_centerline_distance
                ):
                    continue
            elif x < -0.30 and (
                not self.allow_rear_candidates
                or (not dedicated_rear_sensor and abs(y) > 0.30)
            ):
                # The installed low scan plane has only a narrow, partially
                # occluded direct-rear view.  Rear-flank returns outside this
                # band are indistinguishable from the walls and are not safe
                # enough to publish as opponents. This blind-sector restriction
                # belongs to the original forward mounts, not the independently
                # validated, unobstructed rear sensor. Corridor and compactness
                # checks above remain identical for the dedicated sensor.
                continue
            distance = math.hypot(x, y)
            if x > 0.30:
                position = "FRONT"
            elif x < -0.30:
                position = "REAR"
            elif y >= 0.0:
                position = "LEFT_SIDE"
            else:
                position = "RIGHT_SIDE"
            candidates.append(OpponentCandidate(
                x=x,
                y=y,
                distance=distance,
                width=width,
                bearing=math.atan2(y, x),
                position=position,
                beam_indices=tuple(sorted(item.beam_index for item in cluster)),
                extent=extent,
                point_count=len(cluster),
                radial_span=radial_span,
            ))
        return candidates


@dataclass
class _Track:
    track_id: int
    x: float
    y: float
    width: float
    vx: float = 0.0
    vy: float = 0.0
    age: int = 1
    hits: int = 1
    missed: int = 0
    last_observed_x: float = 0.0
    observed_x: float = 0.0
    observed_y: float = 0.0
    observed_time: float = 0.0
    sources: tuple[str, ...] = ()
    plausible: bool = False
    foreground_hits: int = 0


def estimate_tracking_motion(previous, current, dt, speed=0.0, yaw_rate=0.0):
    """Register stationary LiDAR chains for tracker-only ego-frame motion.

    Returns the previous-base -> current-base rigid transform. No world pose,
    navigation state, wall fit or controller output is modified.
    """
    angle=-yaw_rate*dt
    tx,ty=-speed*dt*math.cos(angle*.5), -speed*dt*math.sin(angle*.5)
    if len(previous)<15 or len(current)<15:
        return tx,ty,angle
    old=list(previous)[::max(1,len(previous)//160)]
    new=list(current)[::max(1,len(current)//200)]
    grid={}
    for p in new:grid.setdefault((math.floor(p.x/.4),math.floor(p.y/.4)),[]).append(p)
    for _ in range(6):
        cosine,sine=math.cos(angle),math.sin(angle);pairs=[]
        for p in old:
            x,y=cosine*p.x-sine*p.y+tx,sine*p.x+cosine*p.y+ty
            cell=(math.floor(x/.4),math.floor(y/.4))
            neighbours=[q for i in (-1,0,1) for j in (-1,0,1)
                        for q in grid.get((cell[0]+i,cell[1]+j),())]
            q=min(neighbours,key=lambda q:(q.x-x)**2+(q.y-y)**2,default=None)
            if q is not None and math.hypot(q.x-x,q.y-y)<.5:pairs.append((p,q))
        if len(pairs)<15:break
        px=sum(p.x for p,q in pairs)/len(pairs);py=sum(p.y for p,q in pairs)/len(pairs)
        qx=sum(q.x for p,q in pairs)/len(pairs);qy=sum(q.y for p,q in pairs)/len(pairs)
        cross=sum((p.x-px)*(q.y-qy)-(p.y-py)*(q.x-qx) for p,q in pairs)
        dot=sum((p.x-px)*(q.x-qx)+(p.y-py)*(q.y-qy) for p,q in pairs)
        proposed=math.atan2(cross,dot)
        # A failed scan registration must not produce a vehicle teleport.
        if abs(math.atan2(math.sin(proposed-angle), math.cos(proposed-angle)))>.15:break
        angle=proposed;c,s=math.cos(angle),math.sin(angle)
        tx,ty=qx-c*px+s*py,qy-s*px-c*py
    return tx,ty,angle


def fractional_motion(motion, fraction):
    """Interpolate a rigid transform on SE(2), including wrapped yaw."""
    tx, ty, angle = motion
    angle = math.atan2(math.sin(angle), math.cos(angle))
    if abs(angle) < 1e-8:
        return tx*fraction, ty*fraction, angle*fraction
    ratio = (complex(math.cos(angle*fraction), math.sin(angle*fraction))-1) / (
        complex(math.cos(angle), math.sin(angle))-1)
    translation = complex(tx, ty)*ratio
    return translation.real, translation.imag, angle*fraction


class TrackingMotionTimeline:
    """Primary acquisition epochs only; secondary callbacks never mutate this.

    Registration can lose its correspondences when visible wall chains change.
    Reject impossible transforms and use the last valid LiDAR velocity briefly,
    then the existing command seed. Limits come from the configured car model.
    """

    def __init__(self, maximum_speed, maximum_steering, wheelbase):
        self.maximum_speed = maximum_speed
        self.maximum_yaw_rate = maximum_speed*math.tan(maximum_steering)/wheelbase
        self.reset()

    def reset(self):
        self.timestamp = None
        self.cloud = []
        self.last_valid = None
        self.history = deque(maxlen=32)
        self.diagnostics = {}

    def update(self, cloud, timestamp, speed, yaw_rate):
        dt = None if self.timestamp is None else timestamp-self.timestamp
        previous_valid = self.last_valid
        if not math.isfinite(timestamp) or (dt is not None and dt <= 0):
            self.diagnostics = dict(timestamp=timestamp, dt=dt,
                                    rejected_reason='non_positive_or_invalid_dt',
                                    previous_valid=previous_valid)
            return None
        if dt is None:
            self.timestamp, self.cloud = timestamp, list(cloud)
            self.diagnostics = dict(timestamp=timestamp, dt=None, raw_motion=[0.,0.,0.],
                                    motion=[0.,0.,0.], velocity=[0.,0.,0.],
                                    previous_valid=previous_valid, rejected_reason=None)
            return (0.,0.,0.)
        raw = estimate_tracking_motion(self.cloud, cloud, dt, speed, yaw_rate)
        tx, ty, angle = raw
        angle = math.atan2(math.sin(angle), math.cos(angle))
        reason = None
        # 1 cm accommodates scan quantization, never scales the ID distance gate.
        noise = .01
        if not all(math.isfinite(v) for v in raw):
            reason = 'non_finite_motion'
        elif math.hypot(tx,ty) > self.maximum_speed*dt+noise:
            reason = 'translation_speed_limit'
        elif abs(angle) > self.maximum_yaw_rate*dt+noise/self.maximum_speed:
            reason = 'yaw_rate_limit'
        elif speed >= 0 and tx > noise:
            # A forward-only run cannot move stationary walls forwards by 35 cm.
            reason = 'reverse_translation'
        elif abs(ty) > self.maximum_speed*dt*abs(math.sin(min(math.pi,self.maximum_yaw_rate*dt)*.5))+noise:
            reason = 'lateral_kinematic_limit'
        motion = (tx,ty,angle)
        if reason:
            if previous_valid and timestamp-previous_valid['timestamp'] <= .3:
                motion = tuple(v*dt for v in previous_valid['velocity'])
                fallback = 'last_valid_lidar'
            else:
                angle = -yaw_rate*dt
                motion = (-speed*dt*math.cos(angle*.5), -speed*dt*math.sin(angle*.5), angle)
                fallback = 'command_seed'
        else:
            self.last_valid = dict(timestamp=timestamp, motion=motion,
                                   velocity=tuple(v/dt for v in motion))
            fallback = None
        self.diagnostics = dict(timestamp=timestamp, dt=dt, raw_motion=raw,
                                motion=motion, velocity=tuple(v/dt for v in motion),
                                previous_valid=previous_valid, rejected_reason=reason,
                                fallback=fallback, previous_points=len(self.cloud),
                                current_points=len(cloud))
        mx,my,ma=motion
        mc,ms=math.cos(ma),math.sin(ma)
        ego_delta=(-mc*mx-ms*my,ms*mx-mc*my,-ma)
        self.diagnostics.update(ego_delta=ego_delta,
                                ego_velocity=tuple(v/dt for v in ego_delta))
        self.history.append((self.timestamp,timestamp,motion))
        # Pair with the immediately preceding scan even after a rejected delta;
        # otherwise the next registration spans two epochs but uses one dt.
        self.timestamp, self.cloud = timestamp, list(cloud)
        return motion

    def align_point(self, point, stamp, target):
        if stamp is None or stamp >= target:
            return point
        x,y = point.x,point.y
        for start,end,motion in self.history:
            overlap = max(0.,min(end,target)-max(start,stamp))
            if overlap <= 0:
                continue
            tx,ty,angle = fractional_motion(motion,overlap/(end-start))
            c,s = math.cos(angle),math.sin(angle)
            x,y = c*x-s*y+tx,s*x+c*y+ty
        return Point2(x,y)


class OpponentTracker:
    """Small nearest-neighbour tracker for relative position and closing rate."""

    def __init__(
        self,
        association_distance_m: float = 1.0,
        confirmation_hits: int = 3,
        nominal_width_m: float = 0.25,
    ) -> None:
        self.association_distance = association_distance_m
        self.confirmation_hits = max(2, confirmation_hits)
        self.nominal_width = max(0.10, nominal_width_m)
        self._tracks: dict[int, _Track] = {}
        self._next_id = 1
        self.timestamp = 0.0
        self._has_epoch = False
        self.diagnostics = {}
        self._output = []

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1
        self.timestamp = 0.0
        self._has_epoch = False
        self.diagnostics = {}
        self._output = []

    @staticmethod
    def _position(x: float, y: float) -> str:
        if x > 0.30:
            return "FRONT"
        if x < -0.30:
            return "REAR"
        return "LEFT_SIDE" if y >= 0.0 else "RIGHT_SIDE"

    def update(
        self, candidates: Sequence[OpponentCandidate], dt: float,
        *, timestamp: float | None = None, ego_motion=None,
        raw_observations=(), motion_timeline=None,
    ) -> list[TrackedOpponent]:
        dt=dt if timestamp is None or not self._has_epoch else timestamp-self.timestamp
        if not math.isfinite(dt) or dt <= 0 or (timestamp is not None and not math.isfinite(timestamp)):
            self.diagnostics=dict(timestamp=timestamp,dt=dt,rejected_reason='non_positive_or_invalid_dt',candidates=[],predictions=[])
            return self._output
        now=self.timestamp+dt if timestamp is None else timestamp
        self.timestamp=now
        self._has_epoch=True
        tx,ty,angle=ego_motion or (0.,0.,0.)
        c,s=math.cos(angle),math.sin(angle)
        debug=dict(timestamp=now,dt=dt,ego_motion=[tx,ty,angle],candidates=[],predictions=[])
        def align(point,stamp):
            if motion_timeline is not None:
                return motion_timeline.align_point(point,stamp,now)
            age=max(0.,now-stamp) if stamp is not None else 0.
            ax,ay,a=fractional_motion((tx,ty,angle),min(1.,age/dt))
            ca,sa=math.cos(a),math.sin(a)
            return Point2(ca*point.x-sa*point.y+ax,sa*point.x+ca*point.y+ay)
        # Align each acquisition BEFORE fusion and raw-support admission. The
        # former order compared old-frame raw y with current-frame predicted y,
        # discarding the only true rear evidence during turns.
        candidates=[replace(a,x=(p:=align(Point2(a.x,a.y),a.timestamp)).x,y=p.y,
                            distance=math.hypot(p.x,p.y)) for a in candidates
                    if a.timestamp is None or 0 <= now-a.timestamp <= .15]
        for track in self._tracks.values():
            px,py=track.x+track.vx*dt,track.y+track.vy*dt
            track.x,track.y=c*px-s*py+tx,s*px+c*py+ty
            track.vx,track.vy=c*track.vx-s*track.vy,s*track.vx+c*track.vy
            ox,oy=track.observed_x,track.observed_y
            track.observed_x,track.observed_y=c*ox-s*oy+tx,s*ox+c*oy+ty
            debug['predictions'].append(dict(id=track.track_id,x=track.x,y=track.y,missed=track.missed,hits=track.hits,vx=track.vx,vy=track.vy))

        # Reacquire an existing confirmed physical object directly from compact
        # raw evidence, even when corridor clipping rejected the observation.
        # These candidates are association-only; they cannot create an identity.
        support=[]
        for source,stamp,points in raw_observations:
            if stamp is not None and not 0 <= now-stamp <= .15:
                continue
            points=[replace(p,point=align(p.point,stamp)) for p in points]
            chains=[]
            for p in points:
                if p.range<.30 or p.range>12:continue
                if (not chains or p.beam_index-chains[-1][-1].beam_index>12
                        or p.point.distance(chains[-1][-1].point)>.24):chains.append([])
                chains[-1].append(p)
            compact=[];wall_points=[]
            for chain in chains:
                xs=[p.point.x for p in chain];ys=[p.point.y for p in chain]
                extent=math.hypot(max(xs)-min(xs),max(ys)-min(ys))
                if extent>.75:wall_points.extend(p.point for p in chain)
                elif chain:compact.append((chain,extent))
            for chain,extent in compact:
                x=median(p.point.x for p in chain);y=median(p.point.y for p in chain)
                # Raw support must respect the forward mounts' already validated
                # rear-flank blind restriction, just like their detector. SIDE
                # and FRONT handoffs remain source independent.
                if source in ('primary','secondary') and x<-.30 and abs(y)>.30:
                    continue
                if len(chain)==1:
                    # A single isolated return has no shape evidence. Admit it
                    # only at the measured edge of another confirmed vehicle
                    # occluding this track, not as generic wall/ground support.
                    beam=chain[0].beam_index
                    foreground=[p for p in points if 0<abs(p.beam_index-beam)<=2
                                and .30<=p.range<chain[0].range-.536]
                    if not any(t.hits>=self.confirmation_hits and t.plausible
                               and math.hypot(t.x-x,t.y-y)>.536
                               and any(math.hypot(t.x-p.point.x,t.y-p.point.y)<=.50
                                       for p in foreground)
                               for t in self._tracks.values()):
                        continue
                wall_distance=min((math.hypot(x-p.x,y-p.y) for p in wall_points),default=None)
                candidate=OpponentCandidate(x,y,math.hypot(x,y),max(.02,max(p.point.y for p in chain)-min(p.point.y for p in chain)),math.atan2(y,x),self._position(x,y),tuple(p.beam_index for p in chain),stamp,(source,),extent,len(chain),wall_distance,True)
                support.append(replace(candidate,radial_span=max(p.range for p in chain)-min(p.range for p in chain)))
            # Attach measured neighbouring-wall proximity to detector candidates.
            candidates=[replace(a,wall_distance=min((math.hypot(a.x-p.x,a.y-p.y) for p in wall_points),default=a.wall_distance))
                        if source in a.sources else a for a in candidates]
        admitted=list(candidates)
        for candidate in support:
            # A distant car can expose only one beam during a sensor handoff or
            # partial occlusion. That measured point may maintain a confirmed
            # physical footprint, but cannot confirm/birth a car or use the full
            # distance gate to steal a nearby wall endpoint.
            single_point=candidate.point_count==1
            if any(t.hits>=self.confirmation_hits and (not single_point or t.plausible)
                   and math.hypot(t.x-candidate.x,t.y-candidate.y)<=min(self.association_distance,.50 if single_point else self.association_distance)
                   and abs(t.y-candidate.y)<=.30 for t in self._tracks.values()):
                admitted.append(candidate)
        # Existing physical footprint coalescing, not a wider association gate.
        admitted=fuse_opponent_candidates(admitted,[])
        unmatched=set(self._tracks);unused=set(range(len(admitted)));edges=[]
        for index,candidate in enumerate(admitted):
            cx,cy=candidate.x,candidate.y
            wall_like=candidate.wall_distance is not None and candidate.wall_distance<.24
            # Radial wall slivers slide with viewpoint. New rear identities
            # require transverse support consistent with the validated Hanla
            # front facets; confirmed vehicles may turn/handoff freely.
            radial_sliver=candidate.position=='REAR' and candidate.extent>0 and candidate.radial_span>.90*candidate.extent
            record=dict(timestamp=candidate.timestamp,sources=candidate.sources,x=cx,y=cy,extent=candidate.extent,width=candidate.width,radial_span=candidate.radial_span,point_count=candidate.point_count or len(candidate.beam_indices),wall_distance=candidate.wall_distance,body_distance=math.hypot(max(-.260-cx,0.,cx-.276),max(abs(cy)-.116,0.)),tracking_only=candidate.tracking_only,edges=[],matched_id=None,reason=None)
            debug['candidates'].append(record)
            for ident,track in self._tracks.items():
                distance=math.hypot(cx-track.x,cy-track.y);lateral=abs(cy-track.y)
                elapsed=max(.01,now-track.observed_time)
                vx,vy=(cx-track.observed_x)/elapsed,(cy-track.observed_y)/elapsed
                reason='wall_proximity' if wall_like else 'radial_wall_sliver' if radial_sliver and not track.plausible else 'position_gate' if distance>self.association_distance else 'lateral_gate' if lateral>.30 else 'stale_timestamp' if candidate.timestamp is not None and candidate.timestamp<=track.observed_time else None
                record['edges'].append(dict(id=ident,predicted_x=track.x,predicted_y=track.y,longitudinal=cx-track.x,lateral_residual=cy-track.y,distance=distance,lateral=lateral,candidate_vx=vx,candidate_vy=vy,gate=reason is None,rejected_reason=reason,missed=track.missed))
                if reason is None:
                    cost=distance+.02*math.hypot(vx-track.vx,vy-track.vy)
                    # A consistent recently-missed confirmed identity outranks a
                    # tentative fragment. Source/classification never owns IDs.
                    if track.hits<self.confirmation_hits:cost+=.10
                    edges.append((cost,index,ident,vx,vy))
        for cost,index,ident,vx,vy in sorted(edges):
            if index not in unused or ident not in unmatched:continue
            candidate=admitted[index];track=self._tracks[ident]
            unused.remove(index);unmatched.remove(ident)
            track.x+=.55*(candidate.x-track.x);track.y+=.55*(candidate.y-track.y)
            track.vx+=.30*(vx-track.vx);track.vy+=.30*(vy-track.vy)
            track.width+=.30*(candidate.width-track.width)
            track.age+=1;track.hits+=1;track.missed=0
            track.last_observed_x=candidate.x
            track.observed_x,track.observed_y=candidate.x,candidate.y
            track.observed_time=now
            track.sources=tuple(sorted(set(track.sources+candidate.sources)))
            track.foreground_hits = track.foreground_hits+1 if math.hypot(track.vx,track.vy)>.25 else 0
            if candidate.position!='REAR' or ego_motion is None or track.foreground_hits>=self.confirmation_hits:track.plausible=True
            debug['candidates'][index].update(matched_id=ident,reason='matched_existing',candidate_velocity=[vx,vy],hits=track.hits,foreground_hits=track.foreground_hits,plausible=track.plausible)
        for index in sorted(unused):
            candidate=admitted[index];record=debug['candidates'][index]
            if candidate.wall_distance is not None and candidate.wall_distance<.24:
                record['reason']='wall_proximity_no_birth';continue
            if candidate.position=='REAR' and candidate.extent>0 and candidate.radial_span>.90*candidate.extent:
                record['reason']='radial_wall_sliver_no_birth';continue
            if candidate.tracking_only:
                record['reason']='raw_support_cannot_birth';continue
            duplicates=[t for t in self._tracks.values()
                        if math.hypot(candidate.x-t.x,candidate.y-t.y)<=.50
                        and abs(candidate.y-t.y)<=min(.30,.5*(candidate.width+t.width))]
            if duplicates:
                # A repeated/stale facet of a confirmed or missed car cannot
                # birth another ID. Fresh admissible edges above reacquire it.
                record.update(reason='existing_footprint_duplicate',matched_id=min(duplicates,key=lambda t:math.hypot(candidate.x-t.x,candidate.y-t.y)).track_id)
                continue
            track=_Track(self._next_id,candidate.x,candidate.y,candidate.width,last_observed_x=candidate.x,observed_x=candidate.x,observed_y=candidate.y,observed_time=now,sources=candidate.sources,plausible=candidate.position!='REAR' or ego_motion is None)
            self._tracks[track.track_id]=track;self._next_id+=1
            record.update(matched_id=track.track_id,reason='new_no_admissible_unconsumed_edge')
        for ident in list(unmatched):
            track=self._tracks[ident];track.age+=1;track.missed+=1
            if track.missed>15:del self._tracks[ident]
        self.diagnostics=debug

        output = []
        for track in self._tracks.values():
            # The live detector runs at 10 Hz; three hits retain 0.2 s of
            # temporal confirmation without postponing the pass trajectory.
            if track.hits < self.confirmation_hits or track.missed > 12 or not track.plausible:
                continue
            distance = math.hypot(track.x, track.y)
            relative_vx=track.vx+(tx/dt if ego_motion is not None else 0.)
            relative_vy=track.vy+(ty/dt if ego_motion is not None else 0.)
            range_rate = (
                (track.x * relative_vx + track.y * relative_vy) / distance
                if distance > 1e-6 else 0.0
            )
            output.append(TrackedOpponent(
                track_id=track.track_id,
                x=track.x,
                y=track.y,
                distance=distance,
                width=max(self.nominal_width, track.width),
                bearing=math.atan2(track.y, track.x),
                relative_vx=relative_vx,
                relative_vy=relative_vy,
                closing_rate=-range_rate if track.missed==0 else 0.,
                position=self._position(track.x, track.y),
                age=track.age,
                missed=track.missed,
                last_observed_x=track.last_observed_x,
            ))
        self._output=sorted(output, key=lambda item: item.distance)
        return self._output


def offset_path(
    points: Sequence[Point2], lateral_offset: float, transition_length_m: float = 4.0,
    initial_lateral_offset_m: float = 0.0,
) -> tuple[Point2, ...]:
    """Apply a smooth normal offset; offset zero is an exact solo no-op."""

    if abs(lateral_offset) < 1e-9 and abs(initial_lateral_offset_m) < 1e-9:
        return tuple(points)
    result = []
    travelled = 0.0
    previous = Point2(0.0, 0.0)
    if points and abs(initial_lateral_offset_m) > 1e-9:
        first = points[0]
        second = points[1] if len(points) > 1 else first
        span = second.x - first.x
        slope = (second.y - first.y) / span if abs(span) > 1e-8 else 0.0
        previous = Point2(0.0, first.y - slope * first.x)
    for index, point in enumerate(points):
        travelled += previous.distance(point)
        before = points[index - 1] if index else Point2(0.0, 0.0)
        after = points[index + 1] if index + 1 < len(points) else point
        if index == 0 and abs(initial_lateral_offset_m) > 1e-9:
            # A held pass line starts parallel to the sensed road, not on a
            # fresh ramp from its centre. Estimate its near-field tangent.
            span = after.x - point.x
            slope = (after.y - point.y) / span if abs(span) > 1e-8 else 0.0
            before = Point2(0.0, point.y - slope * point.x)
        dx, dy = after.x - before.x, after.y - before.y
        norm = math.hypot(dx, dy)
        if norm < 1e-8:
            normal_x, normal_y = 0.0, 1.0
        else:
            normal_x, normal_y = -dy / norm, dx / norm
        ratio = max(0.0, min(1.0, travelled / max(0.5, transition_length_m)))
        blend = ratio * ratio * (3.0 - 2.0 * ratio)
        offset = initial_lateral_offset_m + (lateral_offset - initial_lateral_offset_m) * blend
        result.append(Point2(
            point.x + normal_x * offset,
            point.y + normal_y * offset,
        ))
        previous = point
    return tuple(result)


class RaceStrategy:
    """Conservative FOLLOW/PASS/RETURN state machine with rear awareness."""

    def __init__(
        self,
        vehicle_half_width_m: float = 0.116,
        lateral_margin_m: float = 0.16,
        minimum_pass_gap_m: float = 0.55,
        minimum_pass_offset_m: float = 0.65,
        wall_clearance_m: float = 0.25,
        opponent_clearance_m: float = 0.25,
        lateral_rate_mps: float = 0.70,
        medium_speed_mps: float = 1.50,
        dynamic_clearance_target_m: float = 0.25,
        prepare_observation_s: float = 0.0,
        rear_awareness_enabled: bool = True,
        safety_recovery_enabled: bool = True,
        side_occupancy_enabled: bool = True,
        filter_return_offset: bool = True,
    ) -> None:
        self.vehicle_half_width = vehicle_half_width_m
        self.lateral_margin = lateral_margin_m
        self.minimum_pass_gap = minimum_pass_gap_m
        self.minimum_pass_offset = minimum_pass_offset_m
        self.wall_clearance = max(0.0, wall_clearance_m)
        self.opponent_clearance = max(0.0, opponent_clearance_m)
        self.lateral_rate = lateral_rate_mps
        self.medium_speed = medium_speed_mps
        self.dynamic_clearance_target = max(0.05, dynamic_clearance_target_m)
        self.warning_clearance = self.dynamic_clearance_target + 0.05
        self.abort_clearance = max(0.05, self.dynamic_clearance_target - 0.05)
        self.recovery_clearance = self.dynamic_clearance_target + 0.10
        self.prediction_horizon = 0.45
        self.prepare_observation_s = prepare_observation_s
        self.rear_awareness_enabled = rear_awareness_enabled
        self.safety_recovery_enabled = safety_recovery_enabled
        self.side_occupancy_enabled = side_occupancy_enabled
        self.filter_return_offset = filter_return_offset
        self.last_recovery_correction = 0.0
        self.last_target_offset = 0.0
        self.state = "CRUISE"
        self.selected_side = "NONE"
        self.offset = 0.0
        self.pass_track_id: int | None = None
        self.pass_missing_time = 0.0
        self.pass_safety_state = "CLEAR"
        self.pass_warning_time = 0.0
        self.pass_abort_time = 0.0
        self.pass_recovery_time = 0.0
        self.aborted_track_id: int | None = None
        self.prepare_time = 0.0
        self.pass_retry_time = 0.0
        self.hold_recovery_offset: float | None = None
        self.side_wait_track_id: int | None = None

    def reset(self) -> None:
        self.state = "CRUISE"
        self.selected_side = "NONE"
        self.offset = 0.0
        self.pass_track_id = None
        self.pass_missing_time = 0.0
        self.pass_safety_state = "CLEAR"
        self.pass_warning_time = 0.0
        self.pass_abort_time = 0.0
        self.pass_recovery_time = 0.0
        self.aborted_track_id = None
        self.prepare_time = 0.0
        self.pass_retry_time = 0.0
        self.hold_recovery_offset = None
        self.side_wait_track_id = None

    def _pass_lane_clear(
        self,
        tracks: Sequence[TrackedOpponent],
        front: TrackedOpponent,
        side: str,
    ) -> bool:
        """Reject a pass lane occupied beside or just beyond the lead car."""

        if not self.side_occupancy_enabled:
            return True

        for other in tracks:
            if other.track_id == front.track_id or other.missed > 2:
                continue
            if not -0.90 <= other.x <= front.x + 2.0:
                continue
            lateral_separation = other.y - front.y
            if abs(other.x - front.x) <= 1.0 and abs(lateral_separation) > 0.22:
                # Two cars abreast form one occupied road section.  Do not
                # reinterpret the outer car as a new lead and squeeze around
                # the wall side of the pair.
                return False
            if side == "LEFT" and lateral_separation > 0.22:
                return False
            if side == "RIGHT" and lateral_separation < -0.22:
                return False
        return True

    @staticmethod
    def _nearest(
        tracks: Sequence[TrackedOpponent], position: str, maximum_lateral_m: float,
        maximum_missed: int = 2,
    ):
        values = [
            item for item in tracks
            if (
                item.position == position
                and abs(item.y) <= maximum_lateral_m
                and item.missed <= maximum_missed
            )
        ]
        return min(values, key=lambda item: item.distance, default=None)

    def _pass_space(
        self, front: TrackedOpponent, corridor: CorridorMeasurement
    ) -> tuple[float, float, float, float]:
        if corridor.left_wall_y is None or corridor.right_wall_y is None:
            return -math.inf, -math.inf, 0.0, 0.0
        opponent_half = max(0.08, 0.5 * front.width)
        left_gap = corridor.left_wall_y - (front.y + opponent_half)
        right_gap = (front.y - opponent_half) - corridor.right_wall_y
        # Vehicle-center intervals which simultaneously preserve clearance to
        # the wall and to the opponent. The pass target is the interval center,
        # never a fixed lateral offset.
        left_lower = (
            front.y + opponent_half + self.opponent_clearance
            + self.vehicle_half_width
        )
        left_upper = (
            corridor.left_wall_y
            - max(self.wall_clearance, self.recovery_clearance)
            - self.vehicle_half_width
        )
        right_lower = (
            corridor.right_wall_y
            + max(self.wall_clearance, self.recovery_clearance)
            + self.vehicle_half_width
        )
        right_upper = (
            front.y - opponent_half - self.opponent_clearance
            - self.vehicle_half_width
        )
        left_target = 0.5 * (left_lower + left_upper)
        right_target = 0.5 * (right_lower + right_upper)
        return left_gap, right_gap, left_target, right_target

    def _dynamic_pass_clearance(
        self,
        opponent: TrackedOpponent | None,
        corridor: CorridorMeasurement,
        side: str,
    ) -> tuple[float, float, float, float, float]:
        """Return current and short-horizon clearance for the selected side."""

        if opponent is None:
            return math.inf, math.inf, math.inf, math.inf, 0.0
        opponent_half = max(0.08, 0.5 * opponent.width)
        width_sum = self.vehicle_half_width + opponent_half
        lateral_velocity = max(-1.0, min(1.0, opponent.relative_vy))
        predicted_y = opponent.y + lateral_velocity * self.prediction_horizon
        if side == "LEFT":
            wall_clearance = corridor.left_clearance
            opponent_clearance = -opponent.y - width_sum
            current_gap = (
                corridor.left_wall_y - (opponent.y + opponent_half)
                if corridor.left_wall_y is not None else -math.inf
            )
            predicted_gap = (
                corridor.left_wall_y - (predicted_y + opponent_half)
                if corridor.left_wall_y is not None else -math.inf
            )
        else:
            wall_clearance = corridor.right_clearance
            opponent_clearance = opponent.y - width_sum
            current_gap = (
                (opponent.y - opponent_half) - corridor.right_wall_y
                if corridor.right_wall_y is not None else -math.inf
            )
            predicted_gap = (
                (predicted_y - opponent_half) - corridor.right_wall_y
                if corridor.right_wall_y is not None else -math.inf
            )

        # Best possible balanced clearance inside the predicted wall-to-car
        # opening. This catches a gap that is disappearing before the cars are
        # longitudinally alongside.
        predicted_clearance = 0.5 * (
            predicted_gap - 2.0 * self.vehicle_half_width
        )
        predicted_x = opponent.x + opponent.relative_vx * self.prediction_horizon
        alongside_now = abs(opponent.x) <= 0.70
        alongside_soon = min(abs(opponent.x), abs(predicted_x)) <= 0.85
        if alongside_now:
            predicted_clearance = min(
                predicted_clearance, wall_clearance, opponent_clearance
            )
        elif alongside_soon:
            predicted_clearance = min(predicted_clearance, wall_clearance)
        required_gap = (
            2.0 * self.vehicle_half_width
            + 2.0 * self.dynamic_clearance_target
        )
        remaining_gap = min(current_gap, predicted_gap) - required_gap
        return (
            wall_clearance,
            opponent_clearance,
            predicted_clearance,
            remaining_gap,
            lateral_velocity,
        )

    def _update_pass_hysteresis(self, clearance: float, dt: float) -> bool:
        """Update warning/abort/recovery timers; return sustained abort risk."""

        bounded_dt = max(0.01, min(0.25, dt))
        if clearance < self.warning_clearance:
            self.pass_warning_time += bounded_dt
            self.pass_recovery_time = 0.0
            if clearance < self.abort_clearance:
                self.pass_abort_time += bounded_dt
            else:
                self.pass_abort_time = max(0.0, self.pass_abort_time - bounded_dt)
            if self.pass_safety_state == "CLEAR":
                self.pass_safety_state = "WARNING"
        elif clearance >= self.recovery_clearance:
            self.pass_recovery_time += bounded_dt
            self.pass_warning_time = max(0.0, self.pass_warning_time - bounded_dt)
            self.pass_abort_time = max(0.0, self.pass_abort_time - bounded_dt)
            if self.pass_recovery_time >= 0.75:
                self.pass_safety_state = "CLEAR"
                self.pass_warning_time = 0.0
                self.pass_abort_time = 0.0
        else:
            self.pass_recovery_time = 0.0

        return (
            self.pass_abort_time >= 0.25
            or (
                clearance < self.dynamic_clearance_target
                and self.pass_warning_time >= 0.50
            )
        )

    def update(
        self,
        tracks: Sequence[TrackedOpponent],
        corridor: CorridorMeasurement,
        solo_speed_mps: float,
        dt: float,
        pass_candidates: Sequence[OpponentCandidate] = (),
    ) -> RaceDecision:
        front = self._nearest(tracks, "FRONT", 0.70, maximum_missed=0)
        pass_opponent = next(
            (item for item in tracks if item.track_id == self.pass_track_id), None
        )
        rear = self._nearest(
            [item for item in tracks if item.track_id != self.pass_track_id],
            "REAR",
            0.75,
        )
        rear_approaching = (
            self.rear_awareness_enabled
            and rear is not None and rear.distance < 6.0 and rear.closing_rate > 0.25
        )
        self.pass_retry_time = max(
            0.0,
            self.pass_retry_time - max(0.01, min(0.25, dt)),
        )
        left_gap = right_gap = -math.inf
        left_target = right_target = 0.0
        space_opponent = (
            pass_opponent if self.state in {"PASS_LEFT", "PASS_RIGHT"} else front
        )
        if space_opponent is not None:
            left_gap, right_gap, left_target, right_target = self._pass_space(
                space_opponent, corridor
            )
        required_pass_gap = max(
            self.minimum_pass_gap,
            2.0 * self.vehicle_half_width
            + self.wall_clearance + self.opponent_clearance,
        )
        left_safe = left_gap >= required_pass_gap
        right_safe = right_gap >= required_pass_gap
        left_lane_clear = front is None or self._pass_lane_clear(tracks, front, "LEFT")
        right_lane_clear = front is None or self._pass_lane_clear(tracks, front, "RIGHT")
        side_blocked = front is not None and (
            (left_gap > right_gap + 0.05 and not left_lane_clear)
            or (right_gap > left_gap + 0.05 and not right_lane_clear)
        )
        if side_blocked and self.state in {"CRUISE", "FOLLOW", "PREPARE_PASS"}:
            self.side_wait_track_id = front.track_id
        if front is not None and self.side_occupancy_enabled:
            for candidate in pass_candidates:
                if (math.hypot(candidate.x - front.x, candidate.y - front.y) < 0.50
                        and abs(candidate.y - front.y) <= 0.30):
                    continue
                if not -0.90 <= candidate.x <= front.x + 2.0:
                    continue
                if candidate.y > front.y + 0.22:
                    left_safe = False
                if candidate.y < front.y - 0.22:
                    right_safe = False
        if front is not None and self.state == "FOLLOW" and self.aborted_track_id is None:
            # Retry dwell suppresses repeated blocked attempts, not a newly
            # observed free opening. Do not consume another approach interval
            # after a genuine blocker has cleared.
            if ((left_safe and left_gap > right_gap + 0.05
                 and self._pass_lane_clear(tracks, front, "LEFT"))
                    or (right_safe and right_gap > left_gap + 0.05
                        and self._pass_lane_clear(tracks, front, "RIGHT"))):
                self.pass_retry_time = 0.0
        current_wall_clearance = math.inf
        current_opponent_clearance = math.inf
        predicted_pass_clearance = math.inf
        remaining_pass_gap = math.inf
        opponent_lateral_velocity = 0.0
        additional_pass_threat = False

        if self.state == "CRUISE" and front is not None and front.distance < 8.0:
            self.state = "FOLLOW"
            self.prepare_time = 0.0
        elif self.state == "FOLLOW":
            aborted = next(
                (
                    item for item in tracks
                    if item.track_id == self.aborted_track_id
                ),
                None,
            )
            if self.aborted_track_id is not None:
                # Never reselect the opposite side around the same physical
                # opponent after an abort. Remain FOLLOW until that encounter
                # is definitively over, even if lateral gating temporarily
                # removes it from the normal `front` selection.
                if aborted is None:
                    self.pass_missing_time += max(0.01, min(0.25, dt))
                    if self.pass_missing_time >= 1.0:
                        self.state = "CRUISE"
                        self.aborted_track_id = None
                        self.pass_safety_state = "CLEAR"
                else:
                    self.pass_missing_time = 0.0
                    abort_left, abort_right, _, _ = self._pass_space(
                        aborted, corridor
                    )
                    recovered_clearance = 0.5 * (
                        max(abort_left, abort_right)
                        - 2.0 * self.vehicle_half_width
                    )
                    if recovered_clearance >= self.recovery_clearance:
                        self.pass_recovery_time += max(0.01, min(0.25, dt))
                        if self.pass_recovery_time >= 0.50:
                            self.pass_safety_state = "CLEAR"
                    else:
                        self.pass_recovery_time = 0.0
                    if aborted.x < -0.60:
                        self.state = "CRUISE"
                        self.aborted_track_id = None
                        self.pass_safety_state = "CLEAR"
            elif front is None:
                self.state = "CRUISE"
                self.pass_safety_state = "CLEAR"
                self.pass_recovery_time = 0.0
            elif (
                front.distance < 7.0
                and not rear_approaching
                and self.pass_retry_time <= 0.0
            ):
                self.state = "PREPARE_PASS"
        elif self.state == "PREPARE_PASS":
            if front is None:
                self.state = "CRUISE"
                self.prepare_time = 0.0
            elif rear_approaching:
                self.state = "FOLLOW"
                self.prepare_time = 0.0
            else:
                self.prepare_time += max(0.01, min(0.25, dt))
                if self.prepare_time < self.prepare_observation_s:
                    pass
                elif left_safe and left_lane_clear and left_gap > right_gap + 0.05:
                    self.state, self.selected_side = "PASS_LEFT", "LEFT"
                elif right_safe and right_lane_clear and right_gap > left_gap + 0.05:
                    self.state, self.selected_side = "PASS_RIGHT", "RIGHT"
                elif self.prepare_time >= self.prepare_observation_s:
                    self.state, self.selected_side = "FOLLOW", "NONE"
                    self.pass_retry_time = 0.25
                if self.state.startswith("PASS_"):
                    self.pass_track_id = front.track_id
                    self.pass_missing_time = 0.0
                    self.pass_safety_state = "CLEAR"
                    self.pass_warning_time = 0.0
                    self.pass_abort_time = 0.0
                    self.pass_recovery_time = 0.0
        elif self.state in {"PASS_LEFT", "PASS_RIGHT"}:
            passed = next(
                (item for item in tracks if item.track_id == self.pass_track_id), None
            )
            selected_side = "LEFT" if self.state == "PASS_LEFT" else "RIGHT"
            (
                current_wall_clearance,
                current_opponent_clearance,
                predicted_pass_clearance,
                remaining_pass_gap,
                opponent_lateral_velocity,
            ) = self._dynamic_pass_clearance(
                passed, corridor, selected_side
            )
            sustained_risk = self._update_pass_hysteresis(
                predicted_pass_clearance, dt
            )
            lane_occupied = (
                passed is not None
                and not self._pass_lane_clear(tracks, passed, selected_side)
            )
            additional_pass_threat = rear_approaching or lane_occupied
            if lane_occupied:
                sustained_risk = True
                self.pass_safety_state = "WARNING"
            if sustained_risk and passed is not None and passed.x > 0.75:
                self.state = "FOLLOW"
                self.selected_side = "NONE"
                self.pass_track_id = None
                self.aborted_track_id = passed.track_id
                self.pass_safety_state = "ABORT"
                self.pass_recovery_time = 0.0
            elif sustained_risk and passed is not None:
                # Side-by-side or nearly clear: never reverse sides or snap
                # back across the opponent. Hold the current lateral line and
                # reduce the pass speed until longitudinal separation exists.
                self.pass_safety_state = "HOLD"
            if passed is None:
                self.pass_missing_time += max(0.01, min(0.25, dt))
            else:
                self.pass_missing_time = 0.0
            # The installed sensor is body-occluded directly behind. Permit a
            # short prediction through that blind sector only after the same
            # track was genuinely observed beside the car. A front track lost
            # at close range must never be extrapolated into a completed pass.
            passed_with_observation = (
                passed is not None
                and passed.x < -0.60
                and (
                    passed.missed == 0
                    or passed.last_observed_x <= 0.30
                )
            )
            if (
                self.state in {"PASS_LEFT", "PASS_RIGHT"}
                and passed_with_observation
                and not rear_approaching
            ):
                self.state = "RETURN"
        elif self.state == "RETURN":
            if front is not None and front.distance < 5.0:
                self.state = "FOLLOW"
                self.selected_side = "NONE"
            elif abs(self.offset) < 0.03:
                self.state = "CRUISE"
                self.selected_side = "NONE"
                self.pass_track_id = None

        target_offset = 0.0
        if self.pass_safety_state != "HOLD":
            self.hold_recovery_offset = None
        elif self.hold_recovery_offset is None:
            self.hold_recovery_offset = self.offset
        if self.state == "PASS_LEFT":
            if space_opponent is not None and self.pass_safety_state == "CLEAR":
                target_offset = left_target - corridor.corridor_center_y
            else:
                target_offset = self.offset
                if (
                    self.safety_recovery_enabled and additional_pass_threat
                    and self.pass_safety_state == "HOLD"
                    and
                    current_opponent_clearance < self.warning_clearance
                    and current_wall_clearance > self.recovery_clearance
                ):
                    target_offset = max(self.offset, self.hold_recovery_offset + min(
                        0.12,
                        self.recovery_clearance - current_opponent_clearance,
                    ))
                elif (
                    current_wall_clearance < self.warning_clearance
                    and current_opponent_clearance > self.recovery_clearance
                ):
                    target_offset -= min(
                        0.20,
                        self.recovery_clearance - current_wall_clearance,
                    )
        elif self.state == "PASS_RIGHT":
            if space_opponent is not None and self.pass_safety_state == "CLEAR":
                target_offset = right_target - corridor.corridor_center_y
            else:
                target_offset = self.offset
                if (
                    self.safety_recovery_enabled and additional_pass_threat
                    and self.pass_safety_state == "HOLD"
                    and
                    current_opponent_clearance < self.warning_clearance
                    and current_wall_clearance > self.recovery_clearance
                ):
                    target_offset = min(self.offset, self.hold_recovery_offset - min(
                        0.12,
                        self.recovery_clearance - current_opponent_clearance,
                    ))
                elif (
                    current_wall_clearance < self.warning_clearance
                    and current_opponent_clearance > self.recovery_clearance
                ):
                    target_offset += min(
                        0.20,
                        self.recovery_clearance - current_wall_clearance,
                    )
        elif self.state == "PREPARE_PASS" and front is not None:
            if left_gap > right_gap + 0.05 and left_safe and left_lane_clear:
                target_offset = 0.35 * (left_target - corridor.corridor_center_y)
            elif right_gap > left_gap + 0.05 and right_safe and right_lane_clear:
                target_offset = 0.35 * (right_target - corridor.corridor_center_y)
        elif self.state == "RETURN" and rear_approaching:
            target_offset = self.offset

        self.last_recovery_correction = (
            target_offset - self.offset
            if self.state.startswith("PASS_") and self.pass_safety_state != "CLEAR"
            else 0.0
        )
        if rear_approaching and self.state not in {"PASS_LEFT", "PASS_RIGHT"}:
            target_offset = self.offset
        self.last_target_offset = target_offset
        maximum_step = self.lateral_rate * max(0.01, min(0.25, dt))
        if self.state == "RETURN" and not rear_approaching and not self.filter_return_offset:
            # The unchanged controller already smooths the physical recenter.
            # Do not prolong the outgoing PASS goal with a second time filter.
            maximum_step = abs(self.offset)
        self.offset = max(
            self.offset - maximum_step,
            min(self.offset + maximum_step, target_offset),
        )

        speed_cap = math.inf
        if front is not None and self.state in {"FOLLOW", "PREPARE_PASS"}:
            safe_distance = 1.20 + 0.55 * max(0.0, solo_speed_mps)
            estimated_lead_speed = max(0.0, solo_speed_mps - front.closing_rate)
            speed_cap = max(
                0.50,
                min(
                    solo_speed_mps,
                    estimated_lead_speed + 0.55 * (front.distance - safe_distance),
                ),
            )
            if side_blocked:
                # A distinct confirmed blocker requires waiting, not spending
                # the lead distance needed by the unchanged PASS transition.
                # This does not affect a single front or an already active PASS.
                speed_cap = min(speed_cap, 0.50)
        if self.state in {"PASS_LEFT", "PASS_RIGHT"}:
            # Keep the whole lateral transition at the established medium
            # speed.  Waiting until side-by-side is too late to complete the
            # smooth offset when the opponent first resolves at about 5 m.
            speed_cap = min(speed_cap, self.medium_speed)
            passed = next(
                (item for item in tracks if item.track_id == self.pass_track_id), front
            )
            if passed is not None and abs(passed.x) < 0.90:
                speed_cap = min(speed_cap, self.medium_speed)
            if self.pass_safety_state in {"WARNING", "HOLD"}:
                speed_cap = min(speed_cap, max(0.50, 0.70 * self.medium_speed))
            if passed is not None and self.side_wait_track_id == self.pass_track_id:
                lateral_clearance = (
                    abs(passed.y) - self.vehicle_half_width - 0.5 * passed.width
                )
                if passed.x > -0.60 and lateral_clearance < self.dynamic_clearance_target:
                    # After a real side-blocked wait, finish establishing
                    # clearance before accelerating alongside the same lead.
                    # Ordinary single-opponent PASS never enters this branch.
                    speed_cap = min(speed_cap, 0.50)
                else:
                    self.side_wait_track_id = None

        primary = front or rear or (tracks[0] if tracks else None)
        return RaceDecision(
            race_state=self.state,
            selected_pass_side=self.selected_side,
            desired_lateral_offset=self.offset,
            speed_cap=speed_cap,
            left_pass_clearance=left_gap,
            right_pass_clearance=right_gap,
            closest_front_distance=front.distance if front else math.inf,
            closest_rear_distance=rear.distance if rear else math.inf,
            front_closing_rate=front.closing_rate if front else 0.0,
            rear_closing_rate=rear.closing_rate if rear else 0.0,
            primary_opponent_x=primary.x if primary else math.nan,
            primary_opponent_y=primary.y if primary else math.nan,
            current_wall_clearance=current_wall_clearance,
            current_opponent_clearance=current_opponent_clearance,
            predicted_pass_clearance=predicted_pass_clearance,
            remaining_pass_gap=remaining_pass_gap,
            opponent_lateral_velocity=opponent_lateral_velocity,
            pass_safety_state=self.pass_safety_state,
        )
