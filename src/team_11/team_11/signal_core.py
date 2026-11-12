"""Fail-closed camera signal state machine. No ROS or Gazebo dependencies.

A valid start requires at least four bright red lamps, stable over multiple
frames, followed by a stable all-off state. Darkness alone never starts.
"""
from dataclasses import dataclass

@dataclass
class StartGate:
    required_lamps: int = 4
    lit_frames: int = 2
    dark_frames: int = 3
    max_gap_s: float = 2.5

    def __post_init__(self):
        self.lit_run = 0
        self.dark_run = 0
        self.armed_at = None
        self.last_lit_at = None
        self.started = False

    def observe(self, count: int, stamp: float, valid: bool = True) -> bool:
        if self.started:
            return True
        if not valid or count < 0 or count > 5:
            self.lit_run = 0
            self.dark_run = 0
            self.armed_at = None
            self.last_lit_at = None
            return False
        if count >= self.required_lamps:
            self.lit_run += 1
            self.dark_run = 0
            if self.lit_run >= self.lit_frames:
                if self.armed_at is None:
                    self.armed_at = stamp
                self.last_lit_at = stamp
            return False
        if self.armed_at is None:
            self.lit_run = 0
            return False
        if (self.last_lit_at is None or stamp < self.last_lit_at
                or stamp - self.last_lit_at > self.max_gap_s):
            self.lit_run = 0
            self.dark_run = 0
            self.armed_at = None
            self.last_lit_at = None
            return False
        if count == 0:
            self.dark_run += 1
            if self.dark_run >= self.dark_frames:
                self.started = True
        else:
            self.dark_run = 0
        return self.started
