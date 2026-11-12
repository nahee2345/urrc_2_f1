"""Pure Python safety conversion: URRC right-positive steering to Gazebo left-positive yaw."""
import math

def convert(speed, right_steering, gate, fresh_speed, fresh_steer):
    if not (gate and fresh_speed and fresh_steer):
        return 0.0, 0.0
    if not math.isfinite(speed) or not math.isfinite(right_steering):
        return 0.0, 0.0
    speed=max(-2.0,min(15.0,speed))
    steer=max(-math.pi/6,min(math.pi/6,right_steering))
    return speed, -speed*math.tan(steer)/.36
