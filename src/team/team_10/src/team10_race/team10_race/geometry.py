"""Dependency-free local geometry in the LiDAR frame."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Point2:
    """A point in the vehicle/LiDAR frame: +X forward and +Y left."""

    x: float
    y: float

    def distance(self, other: "Point2") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    @property
    def norm(self) -> float:
        return math.hypot(self.x, self.y)


def finite_point(point: Point2) -> bool:
    return math.isfinite(point.x) and math.isfinite(point.y)


def median(values: Iterable[float]) -> float:
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        raise ValueError("cannot take median of an empty sequence")
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def cumulative_lengths(points: Sequence[Point2]) -> list[float]:
    lengths = [0.0]
    for first, second in zip(points, points[1:]):
        lengths.append(lengths[-1] + first.distance(second))
    return lengths
