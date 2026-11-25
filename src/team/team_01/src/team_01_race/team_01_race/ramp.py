"""IMU pitch classification and an independent speed cap; no path generation."""
import math


def quaternion_pitch_deg(x, y, z, w):
    values = (x, y, z, w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite_quaternion')
    norm = math.hypot(*values)
    if not .9 <= norm <= 1.1:
        raise ValueError('invalid_quaternion_norm')
    x, y, z, w = (v/norm for v in values)
    return math.degrees(math.asin(max(-1., min(1., 2.*(w*y-z*x)))))


class RampDetector:
    def __init__(self, *, alpha=.30, enter_deg=5.0, exit_deg=3.0,
                 confirm_frames=3, exit_confirm_frames=3, timeout=.25,
                 normal_speed=10.0, ramp_speed=5.0, stale_speed=5.0):
        if (not all(math.isfinite(v) for v in (alpha, enter_deg, exit_deg, timeout,
                                              normal_speed, ramp_speed, stale_speed)) or
                not 0 < alpha <= 1 or not 0 <= exit_deg < enter_deg <= 90 or
                min(confirm_frames, exit_confirm_frames) < 1 or timeout <= 0 or
                not 0 <= stale_speed <= ramp_speed <= normal_speed or normal_speed <= 0):
            raise ValueError('invalid ramp configuration')
        self.alpha, self.enter_deg, self.exit_deg = alpha, enter_deg, exit_deg
        self.confirm_frames, self.exit_confirm_frames = confirm_frames, exit_confirm_frames
        self.timeout = timeout
        self.normal_speed, self.ramp_speed, self.stale_speed = normal_speed, ramp_speed, stale_speed
        self.raw = self.filtered = None
        self.active = False
        self.enter_count = self.exit_count = 0
        self.last_stamp = self.last_valid_time = -math.inf
        self.valid = False
        self.error = 'no_imu'

    def invalidate(self, reason):
        self.valid = False
        self.error = reason
        self.enter_count = self.exit_count = 0

    def update(self, quaternion, stamp, now, *, age=0.0, orientation_available=True):
        if not all(math.isfinite(v) for v in (stamp, now, age)) or age < 0:
            self.invalidate('invalid_imu_timestamp')
            return False
        if stamp <= self.last_stamp:
            return False  # Replayed samples cannot renew freshness or confirmation.
        self.last_stamp = stamp
        try:
            if not orientation_available:
                raise ValueError('orientation_unavailable_or_wrong_frame')
            if age > self.timeout:
                raise ValueError('imu_message_too_old')
            pitch = quaternion_pitch_deg(*quaternion)
        except (ValueError, TypeError) as error:
            self.invalidate(str(error))
            return False
        fresh_before = self.fresh(now)
        if not fresh_before:
            self.enter_count = self.exit_count = 0
        self.raw = pitch
        self.filtered = (pitch if self.filtered is None or not fresh_before else
                         self.alpha*pitch+(1.-self.alpha)*self.filtered)
        self.last_valid_time = now-age
        self.valid, self.error = True, ''
        magnitude = abs(self.filtered)
        if self.active:
            self.enter_count = 0
            self.exit_count = self.exit_count+1 if magnitude <= self.exit_deg+1e-9 else 0
            if self.exit_count >= self.exit_confirm_frames:
                self.active = False
                self.exit_count = 0
        else:
            self.exit_count = 0
            self.enter_count = self.enter_count+1 if magnitude >= self.enter_deg-1e-9 else 0
            if self.enter_count >= self.confirm_frames:
                self.active = True
                self.enter_count = 0
        return True

    def fresh(self, now):
        return self.valid and 0 <= now-self.last_valid_time <= self.timeout

    def state(self, now):
        return ('RAMP' if self.active else 'NORMAL') if self.fresh(now) else 'IMU_STALE'

    def speed_limit(self, now):
        if not self.fresh(now):
            return self.stale_speed
        return self.ramp_speed if self.active else self.normal_speed

    def constrain(self, result, now):
        """Preserve every existing stop, pass choice, path and lower speed cap."""
        out = result.copy()
        cap = self.speed_limit(now)
        base = out['speed']
        out.update(terrain_state=self.state(now), ramp_active=self.active,
                   imu_fresh=self.fresh(now), ramp_speed_limit=cap,
                   speed_before_ramp=base, active_limiter='STOP')
        if out['enabled']:
            out['speed'] = min(base, cap)
            out['active_limiter'] = ('IMU_STALE' if not self.fresh(now) else 'RAMP') if (
                cap < self.normal_speed and cap <= base) else ('CRUISE' if base >= self.normal_speed else 'EXISTING')
            if self.active and out['mode'] == 'CENTER':
                out['mode'] = 'RAMP'
        return out
