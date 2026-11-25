"""Speed authority from measured body-frame geometry, independent of map identity."""
import math
import numpy as np
from .navigation import valid_path, path_geometry, supported_curvature


def preview_speed(path, confidence, age, stable=False, clear=False, horizon=25.,
                  max_speed=10., decel=2.5, margin=.30):
    if not all(math.isfinite(v) and v > 0 for v in (max_speed, decel)) or not math.isfinite(margin) or margin < 0:
        raise ValueError('invalid preview speed envelope')
    result = dict(supported_preview=0., preview_speed_limit=3.5,
                  preview_stable_straight=False)
    if (not math.isfinite(confidence) or confidence < .4 or
            not math.isfinite(age) or not 0 <= age <= .25 or
            not valid_path(path, horizon)):
        return result
    # Forward coverage is conservative for curves; never count extrapolation.
    supported = float(path[-1, 0])
    limit = next((speed for end, speed in ((5., 3.5), (8., 5.), (12., 7.),
                    (15., 8.), (20., 9.)) if supported < end), 9.)
    heading, _ = path_geometry(path)
    # Use finite spatial support to avoid derivative spikes from pixel resampling.
    robust_curvature=float(np.percentile(np.abs(supported_curvature(path)),90))
    straight = (confidence >= .85 and robust_curvature < .04 and
                np.max(np.abs(heading)) < math.radians(5) and
                np.ptp(np.unwrap(heading)) < math.radians(5))
    if supported >= 20 and straight and stable and clear:
        # The configured ceiling is reachable only inside measured stopping space.
        limit = min(max_speed, math.sqrt(2*decel*max(0.,supported-margin)))
    return dict(supported_preview=supported, preview_speed_limit=min(max_speed,limit),
                preview_stable_straight=bool(straight and stable))
