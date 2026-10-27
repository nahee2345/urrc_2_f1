"""Ackermann wheel/steering setpoints; +race steering means RIGHT.

Gazebo's +Z points up, +Y left, so right steering angles are negative.
Wheel joint axes are +Y; negative angular rates advance vehicle along +X.
This module computes targets only, not position-teleportation.
"""
import math
from .model import Vehicle


def setpoints(v: Vehicle, speed_mps: float, race_steer_rad: float):
    v.validate()
    if not math.isfinite(speed_mps) or not math.isfinite(race_steer_rad):
        raise ValueError("commands must be finite")
    # Limit the inner wheel, not merely the nominal center steering command.
    # The inner Ackermann angle exceeds the center angle in a turn.
    allowable_center = math.atan(v.wheelbase / (v.wheelbase / math.tan(math.radians(v.max_steer_deg)) + v.wheel_track / 2))
    delta = max(-allowable_center, min(allowable_center, race_steer_rad))
    if abs(delta) < 1e-10:
        left = right = 0.0
        wheel_velocities = {w:-speed_mps/v.wheel_radius for w in ("front_left","front_right","rear_left","rear_right")}
    else:
        radius = v.wheelbase/math.tan(abs(delta))
        inner = math.atan(v.wheelbase/(radius-v.wheel_track/2))
        outer = math.atan(v.wheelbase/(radius+v.wheel_track/2))
        # Right turn: the inside wheel is front_right; left turn: front_left.
        if delta>0:
            left,right=-outer,-inner
        else:
            left,right=inner,outer
        # rear-wheel path radii differ, and the front wheels trace wider arcs.
        turn_sign = 1 if delta>0 else -1
        wheel_velocities={}
        for w in ("front_left","front_right","rear_left","rear_right"):
            y = (v.wheel_track/2 if w.endswith("left") else -v.wheel_track/2)
            side_radius=radius+turn_sign*y
            actual=math.hypot(side_radius,v.wheelbase) if w.startswith("front") else side_radius
            wheel_velocities[w]=-speed_mps*actual/(radius*v.wheel_radius)
    return {"steer_left_rad":left,"steer_right_rad":right,"wheel_radps":wheel_velocities,
            "drive_wheel_radps":{k:val for k,val in wheel_velocities.items() if v.drive=="awd" or (v.drive=="fwd" and k.startswith("front")) or (v.drive=="rwd" and k.startswith("rear"))}}
