"""Camera transport/decoding health, independent of paths and traffic lights."""

import math


def image_age_with_geometry(reported_age,now,geometry_time,source):
    """A decoded Camera frame is newer evidence than a delayed age heartbeat.

    Keep the original geometry timestamp: this never renews an old frame and
    never uses LiDAR as evidence that a Camera image arrived.
    """
    if source in ('CAMERA_PATH','FUSED_PATH') and math.isfinite(geometry_time):
        age=now-geometry_time
        if 0<=age<reported_age:return age
    return reported_age


class CameraHealth:
    def __init__(self, required_frames=3, timeout=.25):
        self.required_frames = required_frames
        self.timeout = timeout
        self.frames = 0
        self.last_valid = float('-inf')
        self.last_stamp = None

    def invalidate(self):
        self.frames = 0
        self.last_valid = float('-inf')
        self.last_stamp = None

    def accept(self, now, stamp):
        if now-self.last_valid > self.timeout:
            self.frames = 0
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        self.frames += 1
        self.last_valid = now
        self.last_stamp = stamp

    def ready(self, now):
        if now-self.last_valid > self.timeout:
            self.invalidate()
        return self.frames >= self.required_frames
