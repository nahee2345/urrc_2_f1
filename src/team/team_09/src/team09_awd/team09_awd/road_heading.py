"""Road direction from LiDAR geometry, optional RGB-D, and a side buffer."""
import math
import numpy as np

from .config import G, SETTINGS
from .core import Result, compact, footprint_clearance, rollout
from .recovery import escape_plan
from .right_wall import (
    extend_straight, follow_rollout, route_result, smooth_step,
    pursue, steer_candidates, sweep, trace_wall, wall_shape,
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
    span = rows[-1, 0] - rows[0, 0]
    if span >= 2.4:
        line = np.polyfit(rows[:, 0], rows[:, 1], 1)
        if np.quantile(abs(rows[:, 1] - np.polyval(line, rows[:, 0])), .9) < .035:
            return line[0]*x + line[1], np.full_like(x, line[0])
    y, slope, _ = wall_shape(rows, x)
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
    paired_span = 0.
    if len(available) == 2:
        secondary = left if primary is right else right
        overlap = x <= secondary[-1, 0]
        _, other = supported_shape(secondary, x[overlap])
        tangent_difference = abs(np.arctan(other)-np.arctan(slope[overlap]))
        common_x = x[overlap]
        outside = np.flatnonzero(tangent_difference > SETTINGS["pace"]["wall_agreement_rad"])
        # Only count overlap continuously supported by both fitted wall traces.
        end_index = int(outside[0]) - 1 if len(outside) else len(common_x) - 1
        if end_index >= 0:
            paired_span = max(0., float(common_x[end_index] - max(primary[0, 0], secondary[0, 0])))
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
    return x, y, curvature, fused, paired_span


def projected_gap(points, distances, poses, horizon):
    """Lower bound on wall clearance at swept poses, including sample padding."""
    stop = int(np.searchsorted(distances, horizon, side="right"))
    seen = poses[:stop]
    delta = points[None, :, :] - seen[:, None, :2]
    c, sn = np.cos(seen[:, 2, None]), np.sin(seen[:, 2, None])
    dx = np.abs(delta[:, :, 0] * c + delta[:, :, 1] * sn) - G.half_length
    dy = np.abs(-delta[:, :, 0] * sn + delta[:, :, 1] * c) - G.half_width
    # Between samples the centre translates and the vehicle corners rotate.
    step = np.diff(distances[:stop], append=distances[stop-1])
    turn = abs(np.diff(seen[:, 2], append=seen[-1, 2]))
    pad = .5 * (step + math.hypot(G.half_length, G.half_width) * turn)
    return float(np.min(np.maximum(dx, dy) - pad[:, None]))


def gentle_weight(peak, paired_span, free, heading, right, left, cfg, yaw_residual):
    """Continuous permission to spend a little more of the lateral/drive budget."""
    pace = SETTINGS["pace"]
    if right is None or left is None or yaw_residual is None or not math.isfinite(yaw_residual):
        return 0.
    gap_floor = cfg["band_min_m"]
    k0, k1 = pace["gentle_full_curvature_1pm"], pace["gentle_end_curvature_1pm"]
    # Require measured side clearance, tangents on both sides, alignment and
    # a checked forward sweep. free also imposes the independent stopping limit.
    evidence = min(
        (k1 - peak) / (k1 - k0),
        (min(right, left) - gap_floor) / (pace["full_clearance_m"] - gap_floor),
        (paired_span - pace["paired_span_m"]) / 1.0,
        (free - 4.) / 2.,
        1. - abs(heading) / pace["heading_error_rad"],
        1. - abs(yaw_residual) / SETTINGS["control"]["yaw_residual_limit_radps"],
    )
    return float(smooth_step(evidence))


def tight_turn_candidate(x, y, road_k, speed, settings, base_request):
    """Try a nearer pursuit point only while already turning into a sustained bend."""
    pace = SETTINGS["pace"]
    if speed >= pace["turn_end_speed_mps"] or base_request == 0.:
        return None
    local = road_k[(x[:len(road_k)] >= .29) & (x[:len(road_k)] <= 1.21)]
    if len(local) < 4:
        return None
    signed_min = float(np.min(math.copysign(1., base_request) * local))
    turn = smooth_step((signed_min - pace["turn_curvature_1pm"]) / pace["turn_curvature_1pm"])
    slow = smooth_step((pace["turn_end_speed_mps"] - speed) /
                       (pace["turn_end_speed_mps"] - pace["turn_full_speed_mps"]))
    scale = 1. - pace["turn_lookahead_reduction"] * turn * slow
    if scale >= .999:
        return None
    shorter = dict(settings)
    for key in ("lookahead_base_m", "lookahead_time_s", "lookahead_max_m"):
        shorter[key] *= scale
    candidate = pursue(x, y, 0., 0., 0., speed, shorter)
    if candidate * base_request <= 0. or abs(candidate) <= abs(base_request) + .001:
        return None
    return shorter, float(scale)


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


def plan_road(points, previous, speed, settings, cfg, lateral, brake, latency,
              vision=None, contact=False, yawrate=None, profile="both", pass_state=None, stamp=0., pose=None):
    """baseline, pace and both share perception, braking and actuator bounds."""
    points = compact(points)
    if len(points) < 30:
        return Result()
    clearance = footprint_clearance(points)
    margin = settings["sweep_margin_m"]
    horizon = min(settings["horizon_m"], max(9., speed*.3 + speed*speed/(2.*brake) + 4.))
    right_gap = side_clearance(points, -1)
    left_gap = side_clearance(points, 1)
    relief, band = gap_response(right_gap, left_gap, cfg)
    walls = (trace_wall(points, -1, horizon), trace_wall(points, 1, horizon))
    reference = road_reference(*walls, vision, cfg)
    result = None
    if reference is not None:
        x, y, road_k, fused, paired_span = reference
        entry_heading = math.atan2(y[1], x[1])
        # Extension requires real forward returns and a nearly straight reference.
        x, y, abs_curve = extend_straight(points, x, y, abs(road_k), horizon, settings)
        # Shift away only while too close, using a monotonic displacement.
        y += relief * smooth_step(x / cfg["relief_length_m"])
        k, s, poses = follow_rollout(x, y, previous, speed, settings)
        if len(s) >= 2:
            free, hit = sweep(points, s, poses, margin)
            braking_free = free
            scale = 1.
            passing = None
            if not contact and pass_state is not None:
                passing = pass_state.plan(points, walls, x, y, previous, speed, settings, stamp, pose)
            if passing is not None:
                x, y, k, s, poses, free, hit = passing
                braking_free = free
                slope = np.gradient(y, x)
                abs_curve = np.abs(np.gradient(slope, x))/(1+slope*slope)**1.5
                road_k = np.gradient(slope, x)/(1+slope*slope)**1.5
            if passing is None and profile == "both" and not contact and free >= 1.05:
                candidate = tight_turn_candidate(x, y, road_k, speed, settings, k)
                if candidate is not None:
                    shorter, test_scale = candidate
                    new_k, new_s, new_poses = follow_rollout(x, y, previous, speed, shorter)
                    new_free, new_hit = sweep(points, new_s, new_poses, margin)
                    near = min(free, max(1.2, speed * .65))
                    if (new_free >= free
                            and projected_gap(points, new_s, new_poses, near)
                            >= projected_gap(points, s, poses, near)):
                        k, s, poses = new_k, new_s, new_poses
                        free, hit, scale = new_free, new_hit, test_scale
            if free >= 1.05:
                future = x <= free
                curve = abs_curve[future]
                room = np.maximum(0., x[future] - speed*latency)
                corner_v2 = lateral / np.maximum(curve, .001)
                braking_v2 = 2. * brake * .82 * room
                base_preview = float(np.sqrt(np.min(corner_v2 + braking_v2)))
                weight = 0.
                corridor_gap = None
                if passing is None and profile in ("pace", "both") and not contact:
                    residual = None if yawrate is None else yawrate - speed * previous
                    weight = gentle_weight(float(np.max(curve)), paired_span, free, entry_heading,
                                           right_gap, left_gap, cfg, residual)
                    if weight > 0.:
                        stop_distance = speed * latency + speed**2 / (2. * brake) + .45
                        corridor_gap = projected_gap(points, s, poses, min(free, max(2., stop_distance)))
                        floor = cfg["band_min_m"]
                        width = SETTINGS["pace"]["full_clearance_m"] - floor
                        weight = min(weight, float(smooth_step((corridor_gap - floor) / width)))
                # Keep the new budget below the existing reachable steering envelope.
                max_lateral = min(lateral * SETTINGS["pace"]["lateral_scale"],
                                  SETTINGS["control"]["curvature_lateral_bound_mps2"])
                scale_lat = 1. + weight * max(0., max_lateral / lateral - 1.)
                preview = float(np.sqrt(np.min(corner_v2 * scale_lat + braking_v2)))
                corner = np.count_nonzero(curve >= settings["corner_curvature_1pm"]) >= 3
                result = route_result("ROAD_CURVE" if corner else "ROAD_FORWARD", k, free,
                                      min(1., len(points)/90.), clearance, s, poses, hit)
                if passing is not None:
                    result.route_kind = "OVERTAKE_LEFT" if pass_state.side > 0 else "OVERTAKE_RIGHT"
                result.road_guided = True
                result.road_curvature = float(road_k[np.argmax(abs(road_k[:len(curve)]))])
                result.preview_limit = preview
                result.base_preview_limit = base_preview
                result.braking_free = braking_free
                result.projected_gap = corridor_gap
                result.pace_weight = weight
                result.turn_lookahead_scale = scale
                result.paired_wall_span = paired_span
                result.heading = entry_heading
                result.vision_fused = fused
    if result is None:
        result = checked_fallback(points, previous, speed, margin, horizon, clearance, contact)
    result.right_gap_m = right_gap
    result.left_gap_m = left_gap
    result.wall_band = band
    result.wall_relief_m = relief
    return result
