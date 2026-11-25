"""Fresh body twist for speed safety and frame transforms; pose is unused."""
import math

class SpeedFeedback:
    def __init__(self,timeout=.25):
        self.linear=(0.,0.,0.);self.yaw_rate=0.
        self.timeout=timeout;self.speed=0.;self.time=-math.inf;self.stamp=-math.inf
    def update(self,stamp,now,age,linear,yaw_rate=0.):
        if stamp<=self.stamp or not -.05<=age<=self.timeout or not all(math.isfinite(v) for v in (*linear,stamp,now,yaw_rate)):
            return False
        self.linear=tuple(linear);self.yaw_rate=yaw_rate
        self.speed=math.sqrt(sum(v*v for v in linear));self.time=now-max(0.,age);self.stamp=stamp
        return True
    def fresh(self,now):return 0<=now-self.time<=self.timeout
