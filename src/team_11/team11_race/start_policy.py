"""Official race-gate-backed start policy shared by practice and race.

Camera lighting is primary, real side-car motion is secondary. A positive
official /race/start/team_11 gate is ALWAYS required; no wall-clock auto-start
before gate. Solo-grid fallback is needed in qualifying with no adjacent car.
"""
from __future__ import annotations


def choose_start(mode: str, gate: bool, camera_out: bool,
                 neighbor_moved: bool, neighbor_present: bool,
                 seconds_since_gate: float) -> str | None:
    if not gate:
        return None
    if mode=="signal":
        return "OFFICIAL_SIGNAL"
    if mode!="camera":
        return None
    if camera_out:
        return "CAMERA_LIGHTS_OUT"
    if neighbor_moved:
        return "NEIGHBOR_MOVED_AFTER_GATE"
    # In the single-car qualifier it is physically impossible to see a
    # neighboring car move. The operator's published TRUE is authoritative.
    if seconds_since_gate>=.75 and not neighbor_present:
        return "OFFICIAL_GATE_SOLO"
    # For races where LiDAR misses an adjacent grid car, do not become stuck:
    # the official gate remains authoritative and occurs at atomic lights-out.
    if seconds_since_gate>=2.0:
        return "OFFICIAL_GATE_TIMEOUT"
    return None
