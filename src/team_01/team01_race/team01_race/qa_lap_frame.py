"""Independent QA odometry-frame alignment, never imported by vehicle control.

Gazebo Ackermann odometry may start at (0,0,0) while official Monza
reference coordinates use the global Gazebo world. Fail closed on ambiguity.
"""
import json
import math
from pathlib import Path

class OdomWorld:
    def __init__(self, spawn_report):
        data=json.loads(Path(spawn_report).read_text(encoding='utf-8'))
        self.sx=float(data['x'])
        self.sy=float(data['y'])
        self.yaw=float(data['yaw'])
        self.mode=None

    def transform(self,x,y,yaw):
        x=float(x);y=float(y);yaw=float(yaw)
        if not all(math.isfinite(v) for v in (x,y,yaw)):
            raise ValueError('NONFINITE_ODOM')
        if self.mode is None:
            relative_distance=math.hypot(x,y)
            world_distance=math.hypot(x-self.sx,y-self.sy)
            if relative_distance<.60 and world_distance>1.0:
                self.mode='SPAWN_RELATIVE'
            elif world_distance<.60 and relative_distance>1.0:
                self.mode='WORLD'
            else:
                self.mode='UNVERIFIED'
        if self.mode=='WORLD':
            return x,y,yaw
        if self.mode=='SPAWN_RELATIVE':
            c=math.cos(self.yaw)
            s=math.sin(self.yaw)
            world_yaw=self.yaw+yaw
            return (self.sx+c*x-s*y, self.sy+s*x+c*y,
                    math.atan2(math.sin(world_yaw),math.cos(world_yaw)))
        raise ValueError('ODOM_FRAME_UNVERIFIED')
