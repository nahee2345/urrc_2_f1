"""Elapsed simulation time plus an independent clock-stream liveness check."""
class RuntimeClock:
    def __init__(self):
        self.source=None;self.elapsed=None;self.last_advance_wall=None
    def update(self,source,wall):
        if self.source is None:
            self.elapsed=source;self.last_advance_wall=wall
        elif source>self.source:
            self.elapsed+=max(0.,source-self.source)
            self.last_advance_wall=wall
        # Never rebase on an older sample: alternating streams would count the
        # same 140-second jump repeatedly and manufacture huge sensor ages.
        self.source=source if self.source is None else max(self.source,source)
        return self.elapsed
    def alive(self,wall,timeout):
        return self.last_advance_wall is not None and 0<=wall-self.last_advance_wall<=timeout
