"""Continuous wall selection, robust local fitting, and track-width memory."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .geometry import Point2, median
from .lidar_processing import ScanPoint


@dataclass(frozen=True)
class PolynomialWall:
    """Local wall curve y = c0 + c1*x + c2*x^2."""

    side: str
    coefficients: tuple[float, float, float]
    x_min: float
    x_max: float
    point_count: int
    residual_m: float

    def y(self, x: float) -> float:
        c0, c1, c2 = self.coefficients
        return c0 + c1 * x + c2 * x * x

    def slope(self, x: float) -> float:
        return self.coefficients[1] + 2.0 * self.coefficients[2] * x


@dataclass(frozen=True)
class WallObservation:
    left: PolynomialWall | None
    right: PolynomialWall | None
    track_width_m: float


def _solve(matrix: list[list[float]], vector: list[float]) -> list[float] | None:
    size = len(vector)
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-10:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                current - factor * reference
                for current, reference in zip(augmented[row], augmented[column])
            ]
    return [augmented[row][-1] for row in range(size)]


def _least_squares(points: Sequence[Point2], degree: int) -> tuple[float, float, float] | None:
    size = degree + 1
    matrix = [[0.0] * size for _ in range(size)]
    vector = [0.0] * size
    for point in points:
        powers = [1.0]
        for _ in range(2 * degree):
            powers.append(powers[-1] * point.x)
        for row in range(size):
            vector[row] += point.y * powers[row]
            for column in range(size):
                matrix[row][column] += powers[row + column]
    # Tiny ridge regularization only prevents a degenerate short segment.
    for index in range(size):
        matrix[index][index] += 1e-8
    result = _solve(matrix, vector)
    if result is None:
        return None
    while len(result) < 3:
        result.append(0.0)
    return result[0], result[1], result[2]


def _fragment_gap(first: Sequence[Point2], second: Sequence[Point2]) -> float:
    """Cheap gap between forward extents; avoids an O(N^2) point-pair search."""
    if not first or not second:
        return math.inf
    first_ends = (min(first, key=lambda point: point.x), max(first, key=lambda point: point.x))
    second_ends = (min(second, key=lambda point: point.x), max(second, key=lambda point: point.x))
    return min(a.distance(b) for a in first_ends for b in second_ends)


def _binned(points: Sequence[Point2], bin_size_m: float) -> list[Point2]:
    bins: dict[int, list[Point2]] = {}
    for point in points:
        index = math.floor(point.x / bin_size_m)
        bins.setdefault(index, []).append(point)
    return [
        Point2(median(p.x for p in values), median(p.y for p in values))
        for _, values in sorted(bins.items())
    ]


def fit_wall(
    side: str,
    points: Sequence[Point2],
    minimum_points: int,
    bin_size_m: float,
    maximum_residual_m: float,
    maximum_forward_m: float = 8.0,
) -> PolynomialWall | None:
    usable = [point for point in points if -0.10 <= point.x <= maximum_forward_m]
    samples = _binned(usable, max(0.05, bin_size_m))
    if len(samples) < minimum_points:
        return None

    filtered = samples
    coefficients = None
    for _ in range(3):
        degree = 2 if len(filtered) >= max(minimum_points, 10) else 1
        coefficients = _least_squares(filtered, degree)
        if coefficients is None:
            return None
        residuals = [
            abs(point.y - (coefficients[0] + coefficients[1] * point.x
                           + coefficients[2] * point.x * point.x))
            for point in filtered
        ]
        typical = median(residuals)
        threshold = max(0.055, min(maximum_residual_m, 3.0 * typical))
        retained = [
            point for point, residual in zip(filtered, residuals)
            if residual <= threshold
        ]
        if len(retained) < minimum_points or len(retained) == len(filtered):
            break
        filtered = retained

    if coefficients is None or len(filtered) < minimum_points:
        return None
    residuals = [
        abs(point.y - (coefficients[0] + coefficients[1] * point.x
                       + coefficients[2] * point.x * point.x))
        for point in filtered
    ]
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    if not math.isfinite(rms) or rms > maximum_residual_m:
        return None
    return PolynomialWall(
        side=side,
        coefficients=coefficients,
        x_min=min(point.x for point in filtered),
        x_max=max(point.x for point in filtered),
        point_count=len(filtered),
        residual_m=rms,
    )


class WallTracker:
    def __init__(
        self,
        minimum_points: int,
        bin_size_m: float,
        maximum_residual_m: float,
        anchor_x_m: float,
        initial_track_width_m: float,
        minimum_track_width_m: float,
        maximum_track_width_m: float,
        width_filter_alpha: float,
        maximum_forward_m: float = 8.0,
    ) -> None:
        self.minimum_points = minimum_points
        self.bin_size_m = bin_size_m
        self.maximum_residual_m = maximum_residual_m
        self.maximum_forward_m = max(1.0, maximum_forward_m)
        self.anchor_x_m = anchor_x_m
        self.minimum_width = minimum_track_width_m
        self.maximum_width = maximum_track_width_m
        self.width_alpha = max(0.0, min(1.0, width_filter_alpha))
        self.track_width_m = max(
            self.minimum_width, min(self.maximum_width, initial_track_width_m)
        )
        self.previous_observation: WallObservation | None = None

    @staticmethod
    def _longitudinal_span(segment: Sequence[ScanPoint]) -> float:
        if not segment:
            return 0.0
        values = [item.point.x for item in segment]
        return max(values) - min(values)

    @staticmethod
    def _median_lateral(segment: Sequence[ScanPoint]) -> float:
        return median(item.point.y for item in segment)

    def _rank_segments(
        self, segments: Sequence[Sequence[ScanPoint]], side_sign: float
    ) -> list[Sequence[ScanPoint]]:
        target_y = side_sign * 0.5 * self.track_width_m
        previous = None
        if self.previous_observation is not None:
            previous = (
                self.previous_observation.left
                if side_sign > 0.0 else self.previous_observation.right
            )
        ranked = []
        for segment in segments:
            seed = [
                item.point for item in segment
                if -0.10 <= item.point.x <= 1.60 and side_sign * item.point.y > 0.18
            ]
            if len(seed) < 2:
                continue
            score = min(
                math.hypot(point.x - self.anchor_x_m, point.y - target_y)
                for point in seed
            )
            span = self._longitudinal_span(segment)
            # A wall is normally a long chain. Compact in-corridor objects can
            # have many beams but very little longitudinal support, so keep
            # them behind genuine boundary chains in the provisional fit.
            if span < 0.80:
                score += 1.8 * (0.80 - span)
            lateral = self._median_lateral(segment)
            score += 0.35 * max(0.0, abs(lateral - target_y) - 0.20)
            if previous is not None:
                comparable = [
                    item.point for item in segment
                    if previous.x_min - 0.5 <= item.point.x <= previous.x_max + 1.5
                ]
                if comparable:
                    continuity = median(
                        abs(point.y - previous.y(point.x)) for point in comparable
                    )
                    score += min(3.0, 2.5 * continuity)
            ranked.append((score, -span, segment))
        return [
            segment for _, _, segment
            in sorted(ranked, key=lambda item: (item[0], item[1]))
        ]

    def _fit_ranked(
        self, side: str, ranked: Sequence[Sequence[ScanPoint]]
    ) -> tuple[PolynomialWall | None, Sequence[ScanPoint] | None]:
        for segment in ranked:
            combined = [item.point for item in segment]
            used_segments = {id(segment)}
            # First connect nearby fragments even when no individual fragment
            # is long enough to seed a polynomial (common at tight bends).
            grew = True
            while grew:
                grew = False
                for candidate in ranked:
                    if id(candidate) in used_segments:
                        continue
                    candidate_points = [item.point for item in candidate]
                    endpoint_gap = _fragment_gap(combined, candidate_points)
                    target_y = (1.0 if side == "left" else -1.0) * 0.5 * self.track_width_m
                    boundary_like = (
                        abs(self._median_lateral(candidate) - target_y) <= 0.45
                        or self._longitudinal_span(candidate) >= 0.80
                    )
                    if endpoint_gap <= 1.20 and boundary_like:
                        combined.extend(candidate_points)
                        used_segments.add(id(candidate))
                        grew = True
            wall = fit_wall(
                side, combined, self.minimum_points, self.bin_size_m,
                self.maximum_residual_m, self.maximum_forward_m,
            )
            if wall is None:
                continue
            anchor = max(wall.x_min, min(wall.x_max, self.anchor_x_m))
            value = wall.y(anchor)
            if not ((side == "left" and value > 0.15) or
                    (side == "right" and value < -0.15)):
                continue

            # Gantry posts and grazing angles can split one solid wall into
            # several scan segments. Grow the seeded wall across a modest
            # occlusion when a segment either follows its extrapolation or is
            # spatially close to its current endpoint, then refit robustly.
            for candidate in ranked:
                if id(candidate) in used_segments:
                    continue
                candidate_points = [item.point for item in candidate]
                residual = median(
                    abs(point.y - wall.y(point.x)) for point in candidate_points
                )
                endpoint_gap = _fragment_gap(combined, candidate_points)
                follows_wall = residual <= max(0.35, self.maximum_residual_m)
                long_chain = self._longitudinal_span(candidate) >= 0.80
                if not follows_wall and not (long_chain and endpoint_gap <= 1.20):
                    continue
                trial = fit_wall(
                    side, [*combined, *candidate_points], self.minimum_points,
                    self.bin_size_m, self.maximum_residual_m, self.maximum_forward_m,
                )
                if trial is not None:
                    combined.extend(candidate_points)
                    wall = trial
            return wall, segment
        return None, None

    def observe(self, segments: Sequence[Sequence[ScanPoint]]) -> WallObservation:
        """Fit current walls without changing width or temporal memory."""

        left, left_segment = self._fit_ranked("left", self._rank_segments(segments, 1.0))
        right_ranked = [
            segment for segment in self._rank_segments(segments, -1.0)
            if segment is not left_segment
        ]
        right, _ = self._fit_ranked("right", right_ranked)

        return WallObservation(left, right, self.track_width_m)

    def update(self, segments: Sequence[Sequence[ScanPoint]]) -> WallObservation:
        observation = self.observe(segments)
        left, right = observation.left, observation.right
        if left is not None and right is not None:
            start = max(0.20, left.x_min, right.x_min)
            stop = min(4.0, left.x_max, right.x_max)
            widths = []
            x = start
            while x <= stop:
                slope = 0.5 * (left.slope(x) + right.slope(x))
                width = (left.y(x) - right.y(x)) / math.sqrt(1.0 + slope * slope)
                if self.minimum_width <= width <= self.maximum_width:
                    widths.append(width)
                x += 0.25
            if widths:
                measured = median(widths)
                self.track_width_m += self.width_alpha * (
                    measured - self.track_width_m
                )
        result = WallObservation(left, right, self.track_width_m)
        self.previous_observation = result
        return result
