"""Helpers for conservative longitudinal clearance during passing."""
import math


def required_headway(ego_speed,front_speed,reaction_s=.5,min_gap=1.):
    """Gap for an approaching lead, with non-negative closing speed."""
    if not all(math.isfinite(float(v)) for v in
               (ego_speed,front_speed,reaction_s,min_gap)):
        return float('inf')
    closing=max(0.,float(ego_speed)-float(front_speed))
    return max(0.,float(min_gap))+closing*max(0.,float(reaction_s))


def should_abort_pass(lead_distance,ego_speed,lead_speed):
    if lead_distance is None:return False
    if not math.isfinite(float(lead_distance)):return True
    if lead_speed is None:return float(lead_distance)<1.
    return float(lead_distance)<required_headway(
        ego_speed,lead_speed,reaction_s=.8,min_gap=.65)
