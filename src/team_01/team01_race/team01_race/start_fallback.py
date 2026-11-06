"""Allow start only after seven seconds of spawn-time without red evidence."""
import math


class StartFallback:
    def __init__(self, seconds=7.):
        self.seconds=float(seconds)
        self.spawn_wall=None

    def observe_spawn(self,now):
        if self.spawn_wall is None and math.isfinite(float(now)):
            self.spawn_wall=float(now)

    def elapsed(self,now):
        if self.spawn_wall is None:return None
        return max(0.,float(now)-self.spawn_wall)

    def active(self,now,gate):
        if gate.lights_out:return False
        # Real red evidence takes priority over fallback; never depart during
        # an observed countdown simply because the off sequence was missed.
        if gate.first_bright_time is not None or gate.saw_full_red:
            return False
        seconds=self.elapsed(now)
        return seconds is not None and seconds>=self.seconds
