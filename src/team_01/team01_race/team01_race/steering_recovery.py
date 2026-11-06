"""Physical limiter checks steering alternatives against current safe curb bounds.

All candidate actuator states are computed from the SAME pre-command state.
A failed candidate never changes the limiter or authorizes any displacement.
"""


def choose_safe_steer(limiter,desired,dt,speed,safe,max_angle,allow_alternative):
    state=(limiter.angle,limiter.rate,limiter.filtered_target)
    targets=[float(desired)]
    if allow_alternative:
        targets.extend(desired+offset for offset in
                       (-.05,.05,-.10,.10,-.18,.18))
        targets.extend([state[0],0.])
    used=set()
    for index,target in enumerate(targets):
        target=max(-max_angle,min(max_angle,target))
        rounded=round(target,6)
        if rounded in used:continue
        used.add(rounded)
        limiter.angle,limiter.rate,limiter.filtered_target=state
        output=limiter.apply(target,dt,speed)
        if safe(output):
            return output,('VERIFIED' if index==0 else 'BOUNDED_REPLAN')
    limiter.angle,limiter.rate,limiter.filtered_target=state
    return None,'NO_REACHABLE_SAFE_STEER'
