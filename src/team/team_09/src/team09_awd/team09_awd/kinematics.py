"""Independent steering/speed actuator and wheel/IMU odometry mathematics."""
import math
import numpy as np
from .config import G, SETTINGS

WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


def clamp(x, low, high): return max(low, min(high, x))


def wheel_speeds(v, centre_angle):
    """Rear-axle longitudinal m/s -> four wheel rad/s (pure rolling assumption)."""
    k = math.tan(clamp(centre_angle, -G.max_centre_angle, G.max_centre_angle))/G.wheelbase
    left, right = 1 - G.track*k/2, 1 + G.track*k/2
    return (v*math.hypot(left, G.wheelbase*k)/G.radius,
            v*math.hypot(right, G.wheelbase*k)/G.radius,
            v*left/G.radius, v*right/G.radius)


def measured_curvature(left, right):
    values = []
    for angle, sign in ((left, 1), (right, -1)):
        t = math.tan(angle)
        values.append(t/(G.wheelbase + sign*G.track*t/2))
    return sum(values)/2


def euler(q):
    if len(q) != 4 or not all(math.isfinite(x) for x in q): return None
    norm = math.sqrt(sum(x*x for x in q))
    if not .5 < norm < 1.5: return None
    x,y,z,w = (x/norm for x in q)
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(clamp(2*(w*y-z*x), -1., 1.)),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def quaternion(roll, pitch, yaw):
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def rotation(rpy):
    r,p,y = rpy; cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                     [-sp,cp*sr,cp*cr]])


def transform_points(points, pose):
    return np.asarray(points) @ rotation(pose[3:]).T + pose[:3]


class IndependentActuator:
    def __init__(self):
        self.speed = 0.0
        self.steer = 0.0

    def step(self, v, steer, dt):
        if not all(math.isfinite(x) for x in (v,steer,dt)):
            raise ValueError("Nonfinite actuator command")
        a = SETTINGS["actuator"]
        v = clamp(v, -a["max_reverse_mps"], SETTINGS["control"]["max_speed_mps"])
        dt = clamp(dt, 0., .1)
        # Bring speed through zero before reversing torque target direction.
        target = 0. if self.speed*v < 0 else v
        limit = a["max_accel_mps2"] if abs(target)>abs(self.speed) else a["max_brake_mps2"]
        self.speed += clamp(target-self.speed, -limit*dt, limit*dt)
        # Steering is NEVER multiplied or divided by longitudinal target speed.
        self.steer = clamp(steer, -G.max_centre_angle, G.max_centre_angle)
        return self.speed, self.steer, wheel_speeds(self.speed, self.steer)


class WheelImuOdom:
    """Local dead reckoning only; not a world pose or an EKF.

    Signed speed comes from REAR wheels. IMU yaw supplies direction; centre
    lateral velocity L/2*yaw_rate corrects the rear-axle to base_link offset.
    Wheel slip remains an unobserved error and is labelled as such in logs.
    """
    def __init__(self):
        self.x=0.; self.y=0.; self.yaw=0.; self.yaw_origin=None; self.last=None

    def update(self, stamp, rear_left, rear_right, imu_rpy, yaw_rate):
        if not all(math.isfinite(x) for x in (stamp,rear_left,rear_right,*imu_rpy,yaw_rate)):
            raise ValueError("Nonfinite odometry input")
        if self.last is not None and stamp < self.last: self.__init__()
        if self.yaw_origin is None: self.yaw_origin=imu_rpy[2]
        yaw=math.atan2(math.sin(imu_rpy[2]-self.yaw_origin), math.cos(imu_rpy[2]-self.yaw_origin))
        v = G.radius*(rear_left+rear_right)/2
        vy = G.wheelbase*yaw_rate/2
        dt=0. if self.last is None else stamp-self.last
        if 0 < dt <= SETTINGS["actuator"]["state_timeout_s"]:
            mid=self.yaw+.5*math.atan2(math.sin(yaw-self.yaw),math.cos(yaw-self.yaw))
            self.x+=(v*math.cos(mid)-vy*math.sin(mid))*dt
            self.y+=(v*math.sin(mid)+vy*math.cos(mid))*dt
        self.last=stamp; self.yaw=yaw
        return self.x,self.y,yaw,v,vy
