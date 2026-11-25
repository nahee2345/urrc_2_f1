"""Road direction from LiDAR geometry, optional RGB-D, and a side buffer."""
import math
import numpy as np

from .config import G
from .core import Result, compact, footprint_clearance, rollout
from .recovery import escape_plan
from .right_wall import (
    extend_straight, follow_rollout, route_result, smooth_step,
    steer_candidates, sweep, trace_wall, wall_shape,
)


def side_clearance(points, side):
    """Fit the nearest long side surface beside the body in its current frame."""
    p = points[(abs(points[:, 0]) <= .55) & (side * points[:, 1] > .05)
               & (side * points[:, 1] < 2.)]
    if len(p) < 5:
        return None
    slope, intercept = np.polyfit(p[:, 0], p[:, 1], 1)
    residual = p[:, 1] - (slope*p[:, 0] + intercept)
    keep = abs(residual) <= max(.035, np.median(abs(residual))*2.)
    p = p[keep]
    if len(p) < 5 or np.ptp(p[:, 0]) < .25:
        return None
    slope, intercept = np.polyfit(p[:, 0], p[:, 1], 1)
    if abs(slope) > .8 or np.std(p[:, 1] - (slope*p[:, 0] + intercept)) > .045:
        return None
    # Normal distance from the oriented rectangle, including its front corner.
    support = G.half_width + abs(slope)*G.half_length
    return float((abs(intercept) - support) / math.hypot(1., slope))


def gap_response(right, left, cfg):
    """Relieve a close wall. Neither a safe band nor a far wall attracts the car."""
    low, high = cfg["band_min_m"], cfg["band_max_m"]
    right_near = right is not None and right < low
    left_near = left is not None and left < low
    if right_near or left_near:
        r = low - right if right_near else 0.
        l = low - left if left_near else 0.
        return float(np.clip((r-l)*cfg["relief_gain"], -cfg["relief_max_m"], cfg["relief_max_m"])), (
            "BOTH_NEAR" if right_near and left_near else "RIGHT_NEAR" if right_near else "LEFT_NEAR")
    if right is None:
        return 0., "RIGHT_UNKNOWN"
    return 0., "IN_BAND" if right <= high else "RIGHT_FAR"


def supported_shape(rows, x):
    """Suppress single-point curvature spikes without flattening a sustained bend."""
    y, slope, _ = wall_shape(rows, x)
    span = rows[-1, 0] - rows[0, 0]
    if span >= 2.4:
        line = np.polyfit(rows[:, 0], rows[:, 1], 1)
        if np.quantile(abs(rows[:, 1] - np.polyval(line, rows[:, 0])), .9) < .035:
            return line[0]*x + line[1], np.full_like(x, line[0])
    return y, slope


def camera_slopes(x, slope, vision, cfg):
    """Use camera shape when it agrees with LiDAR on the common interval."""
    if vision is None:
        return slope, False
    p = np.asarray(vision)
    p = p[np.isfinite(p[:, :2]).all(axis=1) & (p[:, 0] >= .6) & (p[:, 0] <= x[-1]), :2]
    if len(p) < 7 or np.ptp(p[:, 0]) < 2.4:
        return slope, False
    lo, hi = float(p[:, 0].min()), float(p[:, 0].max())
    a, b, c = np.polyfit(p[:, 0], p[:, 1], 2)
    if np.std(p[:, 1] - np.polyval((a,b,c), p[:, 0])) > .12:
        return slope, False
    use = (x >= lo) & (x <= hi)
    estimated = 2.*a*x[use] + b
    if np.count_nonzero(use) < 5 or np.max(abs(np.arctan(estimated)-np.arctan(slope[use]))) > cfg["vision_heading_agreement_rad"]:
        return slope, False
    weight = cfg["vision_weight"] * smooth_step((x[use]-lo)/.8) * smooth_step((hi-x[use])/.8)
    out = slope.copy()
    out[use] = (1. - weight)*slope[use] + weight*estimated
    return out, bool(np.any(weight > 0.))


def road_reference(right, left, vision, cfg):
    """Integrate wall tangents from the car's own position, discarding offset."""
    available = [wall for wall in (right, left) if wall is not None]
    if not available:
        return None
    primary = max(available, key=lambda w: w[-1, 0])
    end = float(primary[-1, 0])
    x = np.arange(0., end, .3)
    if len(x) < 6:
        return None
    _, slope = supported_shape(primary, x)
    if len(available) == 2:
        secondary = left if primary is right else right
        overlap = x <= secondary[-1, 0]
        _, other = supported_shape(secondary, x[overlap])
        tangent_difference = abs(np.arctan(other)-np.arctan(slope[overlap]))
        # Smoothly reduce the other wall's influence near the end of its view
        # or when it indicates a different direction (e.g. at a junction).
        w = .5 * smooth_step((secondary[-1, 0]-x[overlap])/1.1)
        w *= 1. - smooth_step((tangent_difference-.09)/.14)
        slope[overlap] = (1.-w)*slope[overlap] + w*other
    slope, fused = camera_slopes(x, slope, vision, cfg)
    heading = np.arctan(slope)
    # Average over adjacent samples before differentiating; no S-shaped offset.
    heading = np.convolve(np.pad(heading, (2, 2), mode="edge"), [.1,.2,.4,.2,.1], mode="valid")
    tan = np.tan(heading)
    y = np.r_[0., np.cumsum(np.diff(x) * (tan[1:] + tan[:-1]) * .5)]
    curvature = np.gradient(heading, x)*np.cos(heading)
    return x, y, curvature, fused


def checked_fallback(points, previous, speed, margin, horizon, clearance, contact):
    close = speed < .6 and (contact or clearance < margin)
    if close:
        escape = escape_plan(points, previous, speed)
        if escape is not None:
            return escape
    best = None
    for k in steer_candidates(previous, speed):
        s, poses = rollout(k, previous, speed, min(14., horizon))
        free, hit = sweep(points, s, poses, margin)
        if free < .85:
            continue
        score = min(free, 8.) - .28*abs(k-previous) - .20*abs(k)
        if best is None or score > best[0]:
            best = score, k, free, s, poses, hit
    if best is None:
        if speed < .6 and not close:
            escape = escape_plan(points, previous, speed)
            if escape is not None:
                return escape
        return Result(clearance=clearance)
    _, k, free, s, poses, hit = best
    return route_result("ROAD_FALLBACK", k, free, min(1., len(points)/90.), clearance, s, poses, hit)


def plan_road(points, previous, speed, settings, cfg, lateral, brake, latency, vision=None, contact=False):
    points = compact(points)
    if len(points) < 30:
        return Result()
    clearance = footprint_clearance(points)
    margin = settings["sweep_margin_m"]
    horizon = min(settings["horizon_m"], max(9., speed*.3 + speed*speed/(2.*brake) + 4.))
    right_gap = side_clearance(points, -1)
    left_gap = side_clearance(points, 1)
    relief, band = gap_response(right_gap, left_gap, cfg)
    reference = road_reference(trace_wall(points, -1, horizon), trace_wall(points, 1, horizon), vision, cfg)
    result = None
    if reference is not None:
        x, y, road_k, fused = reference
        entry_heading = math.atan2(y[1], x[1])
        # Extension requires real forward returns and a nearly straight reference.
        x, y, abs_curve = extend_straight(points, x, y, abs(road_k), horizon, settings)
        # Shift away only while too close, using a monotonic displacement.
        y += relief * smooth_step(x / cfg["relief_length_m"])
        k, s, poses = follow_rollout(x, y, previous, speed, settings)
        if len(s) >= 2:
            free, hit = sweep(points, s, poses, margin)
            if free >= 1.05:
                future = x <= free
                curve = abs_curve[future]
                room = np.maximum(0., x[future] - speed*latency)
                corner_v2 = lateral/np.maximum(curve, .001)
                preview = float(np.sqrt(np.min(corner_v2 + 2.*brake*.82*room)))
                corner = np.count_nonzero(curve >= settings["corner_curvature_1pm"]) >= 3
                result = route_result("ROAD_CURVE" if corner else "ROAD_FORWARD", k, free,
                                      min(1., len(points)/90.), clearance, s, poses, hit)
                result.road_guided = True
                result.road_curvature = float(road_k[np.argmax(abs(road_k[:len(curve)]))])
                result.preview_limit = preview
                result.heading = entry_heading
                result.vision_fused = fused
    if result is None:
        result = checked_fallback(points, previous, speed, margin, horizon, clearance, contact)
    result.right_gap_m = right_gap
    result.left_gap_m = left_gap
    result.wall_band = band
    result.wall_relief_m = relief
    return result
