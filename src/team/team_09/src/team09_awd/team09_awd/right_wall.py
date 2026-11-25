"""Local right-wall line, corner placement, and swept-body clearance."""
import math

import numpy as np

from .config import G
from .core import Result, compact, footprint_clearance, reachable_curvature, rollout, steering_rate
from .recovery import escape_plan


def trace_wall(points, side, horizon):
    """Trace contiguous returns in forward slabs, starting from the measured side."""
    rows = []
    for x in np.arange(.30, horizon + .09, .40):
        # Angular scans become sparse along a parallel wall as range grows.
        # Avoid counting a return repeatedly: each slab owns its own points.
        slab = points[(points[:, 0] >= x - .20) & (points[:, 0] < x + .20)]
        if not rows:
            if x > 1.10:
                break
            y = side * slab[:, 1]
            slab = slab[(y > G.half_width * .75) & (y < 4.)]
            if len(slab):
                nearest = slab[np.argmin(side * slab[:, 1]), 1]
                slab = slab[np.abs(slab[:, 1] - nearest) < .14]
        else:
            x0, y0 = rows[-1]
            max_gap = min(6.0, 1.2 + .022 * x0*x0)
            if x - x0 > max_gap:
                break
            slope = 0.
            if len(rows) >= 2:
                x1, y1 = rows[-2]
                slope = np.clip((y0 - y1) / (x0 - x1), -1.2, 1.2)
            expected = y0 + slope * (x - x0)
            tolerance = .22 + .18 * (x - x0)
            slab = slab[np.abs(slab[:, 1] - expected) <= tolerance]
        if len(slab):
            rows.append((float(np.median(slab[:, 0])), float(np.median(slab[:, 1]))))
    if len(rows) < 5 or rows[-1][0] - rows[0][0] < 1.59:
        return None
    return np.asarray(rows)


def wall_shape(rows, x):
    """Fit a local quadratic in a shared forward coordinate at each sample."""
    y = np.empty_like(x)
    slope = np.empty_like(x)
    curvature = np.empty_like(x)
    for i, xi in enumerate(x):
        indices = np.argsort(abs(rows[:, 0] - xi))[:7]
        dx = rows[indices, 0] - xi
        a, b, c = np.polyfit(dx, rows[indices, 1], 2)
        y[i], slope[i] = c, b
        curvature[i] = 2. * a / (1. + b * b) ** 1.5
    return y, slope, curvature


def smooth_step(x):
    t = np.clip(x, 0., 1.)
    return t*t*t*(10. + t*(-15. + 6.*t))


def corner_bias(x, curvature, speed, settings):
    """Move toward the outside before turn-in, then toward the inside."""
    threshold = settings["corner_curvature_1pm"]
    moderate = np.abs(curvature) >= threshold
    persistent = np.flatnonzero(moderate[:-2] & moderate[1:-1] & moderate[2:]
                                & (curvature[:-2] * curvature[2:] > 0.))
    if not len(persistent):
        return np.zeros_like(x), False
    first = int(persistent[0])
    sign = math.copysign(1., curvature[first])
    end = first + 2
    while end + 1 < len(x) and sign * curvature[end + 1] >= .65 * threshold:
        end += 1
    if x[first] > max(settings["corner_setup_m"], speed * .85):
        return np.zeros_like(x), False
    entry = x[first]
    lead = max(settings["corner_setup_m"], speed * .85)
    split = min(x[end], entry + settings["corner_blend_m"])
    if split <= entry:
        return np.zeros_like(x), False
    outside = smooth_step((x - (entry - lead)) / max(.6, lead * .6))
    turn_in = smooth_step((x - (entry - min(.8, lead * .2))) / max(.6, split - entry + min(.8, lead * .2)))
    exit_return = smooth_step((x - x[end]) / settings["corner_blend_m"])
    # Positive means move away from the right wall. Right-hand corner: + then -.
    bias = (-sign * outside * (1. - turn_in) + sign * turn_in * (1. - exit_return))
    return np.clip(bias, -1., 1.), True


def make_line(right, left, speed, settings):
    right_end = end = right[-1, 0]
    inferred_width = None
    if left is not None and left[-1, 0] >= right_end + .8:
        overlap = np.arange(.7, right_end + .09, .4)
        if len(overlap) >= 5:
            ly, _, _ = wall_shape(left, overlap)
            ry, _, _ = wall_shape(right, overlap)
            width = ly - ry
            future = np.arange(max(.7, right_end - 1.2), left[-1, 0], .4)
            _, _, left_turn = wall_shape(left, future)
            clear_turn = len(left_turn) >= 3 and np.count_nonzero(
                abs(left_turn) >= settings["corner_curvature_1pm"]) >= 3
            if clear_turn and np.ptp(width) < .22 and np.min(width) > .55:
                inferred_width = float(width[-1])
                end = min(left[-1, 0], right_end + settings["corner_setup_m"])
    x = np.r_[np.arange(0., end, .40), end]
    wall, slope, curvature = wall_shape(right, x)
    if inferred_width is not None:
        ly, ls, lk = wall_shape(left, x)
        weight = smooth_step((x - (right_end - .8)) / .8)
        wall = (1. - weight)*wall + weight*(ly - inferred_width)
        slope = (1. - weight)*slope + weight*ls
        curvature = (1. - weight)*curvature + weight*lk
    normal = np.sqrt(1. + slope*slope)
    bias, corner = corner_bias(x, curvature, speed, settings)
    gap = settings["clearance_m"]
    offset = np.full_like(x, G.half_width + gap)
    # Require a measured left boundary at each sample before moving across the lane.
    if corner and left is not None:
        observed = x <= left[-1, 0]
        ly, _, _ = wall_shape(left, x[observed])
        width = (ly - wall[observed]) / normal[observed]
        floor = G.half_width + settings["min_corner_clearance_m"]
        ceiling = width - floor
        room = np.maximum(0., ceiling - offset[observed])
        amplitude = settings["corner_fraction"]
        local_bias = bias[observed]
        # Across an occluded inside (right) wall, keep the normal offset
        # rather than squeezing closer to the unmeasured inside boundary.
        close_bias = np.minimum(local_bias, 0.) * (x[observed] <= right_end)
        chosen = offset[observed] + amplitude * (
            np.maximum(local_bias, 0.) * room + close_bias * (offset[observed] - floor))
        offset[observed] = np.where(ceiling >= floor, np.minimum(chosen, ceiling), offset[observed])
        corner = bool(np.max(np.abs(offset - (G.half_width + gap))) > .035)
    else:
        corner = False
    y = wall + offset * normal
    # Slow the preview for both the measured road bend and the selected line.
    dy = np.gradient(y, x)
    line_k = np.gradient(dy, x) / np.power(1. + dy*dy, 1.5)
    path_k = np.maximum(abs(line_k), abs(curvature))
    return x, y, path_k, corner


def extend_straight(points, x, y, curvature, horizon, settings):
    """A straight wall fit can guide past its last side return within LiDAR view."""
    end = x[-1]
    if end < 4. or np.max(curvature) >= settings["corner_curvature_1pm"] * .50:
        return x, y, curvature
    slope, intercept = np.polyfit(x, y, 1)
    if np.max(np.abs(y - (slope * x + intercept))) > settings["min_corner_clearance_m"]:
        return x, y, curvature
    # Three forward returns in the projected driving strip substantiate extra
    # view. Their actual points remain in the sweep; they can also be blockers.
    offset = points[:, 1] - (slope * points[:, 0] + intercept)
    forward = points[(points[:, 0] > end + .4) & (abs(offset) <= G.half_width + settings["clearance_m"]), 0]
    if len(forward) < 3:
        return x, y, curvature
    limit = min(horizon, float(np.partition(forward, -3)[-3]))
    tail = np.arange(end + .4, limit, .4)
    if not len(tail):
        return x, y, curvature
    return (np.r_[x, tail], np.r_[y, y[-1] + slope * (tail - end)],
            np.r_[curvature, np.zeros(len(tail))])


def pursue(x, y, px, py, yaw, speed, settings):
    look = np.clip(settings["lookahead_base_m"] + settings["lookahead_time_s"] * speed,
                   settings["lookahead_base_m"], settings["lookahead_max_m"])
    tx = min(x[-1], px + look)
    ty = float(np.interp(tx, x, y))
    dx, dy = tx - px, ty - py
    if dx < .12:
        return 0.
    return 2. * (dy * math.cos(yaw) - dx * math.sin(yaw)) / max(.2, dx*dx + dy*dy)


def follow_rollout(x, y, previous, speed, settings):
    distance = np.arange(0., x[-1] + .001, .15)
    rear_x, rear_y, yaw = -G.wheelbase / 2., 0., 0.
    angle = math.atan(G.wheelbase * previous)
    rate_step = steering_rate(speed) * .15 / max(speed, .4)
    request = pursue(x, y, 0., 0., 0., speed, settings)
    poses = []
    for i, _ in enumerate(distance):
        cx = rear_x + G.wheelbase / 2. * math.cos(yaw)
        cy = rear_y + G.wheelbase / 2. * math.sin(yaw)
        if cx > x[-1] - .15 or abs(yaw) > 1.15:
            break
        poses.append((cx, cy, yaw))
        # Until the next part of the road enters view, treat the requested near
        # steering as one command at a time. Include the measured steering slew.
        desired = request if i == 0 else pursue(x, y, cx, cy, yaw, speed, settings)
        goal = math.atan(G.wheelbase * reachable_curvature(desired, speed))
        nxt = min(angle + rate_step, max(angle - rate_step, goal))
        turn = math.tan((angle + nxt) * .5) / G.wheelbase * .15
        rear_x += .15 * math.cos(yaw + turn * .5)
        rear_y += .15 * math.sin(yaw + turn * .5)
        yaw += turn
        angle = nxt
    return request, distance[:len(poses)], np.asarray(poses)


def sweep(points, s, poses, margin):
    """Keep forward obstruction checks even when a parallel wall is close."""
    rel = points[None, :, :] - poses[:, None, :2]
    c, sn = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
    x = rel[:, :, 0] * c + rel[:, :, 1] * sn
    y = -rel[:, :, 0] * sn + rel[:, :, 1] * c
    # Covers the intermediate corner sweep between discrete sampled positions.
    longitudinal_pad = margin + np.diff(s, prepend=s[0])[:, None] / 2.
    side_gap = abs(y) - G.half_width
    in_length = abs(x) < G.half_length + longitudinal_pad
    blocked = in_length & (side_gap < margin)
    # Only previously alongside points get this exemption. They're still
    # rejected if the body moves closer to them or if they enter the hull.
    alongside = np.zeros(len(points), dtype=bool)
    for side in (-1, 1):
        near = ((abs(x[0]) <= G.half_length + .06) & (side * y[0] > 0.)
                & (side_gap[0] >= 0.) & (side_gap[0] < margin))
        if np.count_nonzero(near) >= 3 and np.ptp(x[0, near]) >= .12:
            reference = float(np.median(y[0, near]))
            # Apply the allowance only to the same side band actually measured
            # next to the car. Front obstacles and the opposite side are tested.
            alongside |= (abs(y[0] - reference) <= .012) & (side_gap[0] >= 0.)
    no_closing = (side_gap >= np.maximum(0., side_gap[0] - .001)) & (np.sign(y) == np.sign(y[0]))
    blocked &= ~(alongside[None, :] & no_closing)
    hits = np.flatnonzero(blocked.any(axis=1) | (abs(poses[:, 2]) > 1.22))
    i = int(hits[0]) if len(hits) else len(s) - 1
    return float(s[i]), i


def steer_candidates(previous, speed):
    bound = abs(reachable_curvature(G.max_curvature, speed))
    # No centre-target cost: choose by measured free path and steering continuity.
    return np.unique(np.r_[np.linspace(-bound, bound, 11), np.clip(previous, -bound, bound), 0.])


def route_result(kind, k, free, confidence, clearance, s, poses, hit):
    result = Result(float(k), free, confidence, clearance, 0., "RUN")
    result.route_kind = kind
    result.route_dist = s[:hit+1]
    result.route_poses = poses[:hit+1]
    result.preview_limit = float("inf")
    result.proximity_limit = float("inf")
    return result


def plan_right_wall(points, previous, speed, settings, lateral, brake, contact=False):
    points = compact(points)
    if len(points) < 30:
        return Result()
    clearance = footprint_clearance(points)
    margin = settings["sweep_margin_m"]
    confidence = min(1., len(points) / 90.)
    horizon = min(settings["horizon_m"], max(9., speed * .3 + speed*speed / (2.*brake) + 4.))
    right = trace_wall(points, -1, horizon)
    rt_error = None
    if right is not None:
        left = trace_wall(points, 1, horizon)
        x, y, curvatures, corner = make_line(right, left, speed, settings)
        if not corner:
            x, y, curvatures = extend_straight(points, x, y, curvatures, horizon, settings)
        rt_error = float(y[0])
        k, s, poses = follow_rollout(x, y, previous, speed, settings)
        if len(s) >= 2:
            free, hit = sweep(points, s, poses, margin)
            if free >= 1.05:
                route = route_result("RIGHT_WALL_CORNER" if corner else "RIGHT_WALL",
                                     k, free, confidence, clearance, s, poses, hit)
                route.right_wall_error_m = rt_error
                route.heading = math.atan2(y[1] - y[0], x[1] - x[0])
                route.wall_quality = 1.
                # Translate curvature ahead into a speed bound now, using a
                # braking budget and latency reserve in simulation metres.
                ahead = x <= free
                corner_v2 = lateral / np.maximum(curvatures[ahead], .001)
                braking_space = np.maximum(0., x[ahead] - speed * settings["preview_latency_s"])
                route.preview_limit = float(np.sqrt(np.min(corner_v2 + 2. * brake * .82 * braking_space)))
                return route
    tried_escape = (contact or clearance < margin) and speed < .6
    if tried_escape:
        escape = escape_plan(points, previous, speed)
        if escape is not None:
            return escape
    fallback_horizon = min(14., horizon)
    candidates = []
    for k in steer_candidates(previous, speed):
        s, poses = rollout(k, previous, speed, fallback_horizon)
        free, hit = sweep(points, s, poses, margin)
        if free >= .85:
            score = min(free, 8.) - .28 * abs(k - previous) - .20 * abs(k)
            candidates.append((score, k, free, s, poses, hit))
    if not candidates:
        if speed < .6 and not tried_escape:
            escape = escape_plan(points, previous, speed)
            if escape is not None:
                return escape
        return Result(clearance=clearance)
    _, k, free, s, poses, hit = max(candidates, key=lambda item: item[0])
    route = route_result("REACTIVE", k, free, confidence, clearance, s, poses, hit)
    route.right_wall_error_m = rt_error
    return route
