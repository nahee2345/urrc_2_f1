"""Stage 6 pre-race configuration, not a localization or navigation source.

Official rules prohibit stored map coordinates, world pose and competitor
telemetry as driving inputs. Track and spawn slot here are operator-provided
session metadata, never deduced from prohibited sources.
"""
from dataclasses import dataclass

TRACKS=("monza","spa","monaco","cota","hungaroring","baku")

@dataclass(frozen=True)
class Session:
    track: str
    grid: int
    initial_lane: int
    left_is_lane_3: bool = True

def configure(track, grid):
    if track not in TRACKS:
        raise ValueError("unsupported official track")
    if not isinstance(grid,int) or isinstance(grid,bool) or not 1<=grid<=11:
        raise ValueError("grid must be integer 1..11")
    return Session(track=track,grid=grid,initial_lane=1 if grid%2 else 3)

def neutral_context():
    """Fail closed when race runner does not provide a trustworthy grid slot."""
    return None

def lane_relative_to_ego(initial_lane):
    """Return a lateral preference sign only; no hard-coded road coordinates."""
    if initial_lane not in (1,2,3):
        raise ValueError("lane 1/2/3 required")
    return {1:-1,2:0,3:1}[initial_lane]
