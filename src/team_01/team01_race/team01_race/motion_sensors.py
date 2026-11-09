"""Read IMU yaw rate and real front steering joints independently of plans."""
import math
import time
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import qos_profile_sensor_data


class MotionSensors:
    def init_motion_sensors(self):
        self.imu_rate=None
        self.imu_at=0.
        self.joint_angle=None
        self.joint_at=0.
        self.create_subscription(Imu,'/team01/imu',self.on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState,'/team01/steering_joint_states',
                                 self.on_steering_joint,qos_profile_sensor_data)

    def on_imu(self,msg):
        rate=float(msg.angular_velocity.z)
        covariance=msg.angular_velocity_covariance
        if not math.isfinite(rate) or abs(rate)>8. or (covariance and covariance[0]<0):
            return
        self.imu_rate=rate
        self.imu_at=time.monotonic()

    def on_steering_joint(self,msg):
        names=list(msg.name);values=list(msg.position)
        needed=('front_left_steer_joint','front_right_steer_joint')
        if any(n not in names for n in needed):return
        try:
            angles=[float(values[names.index(n)]) for n in needed]
        except (ValueError,IndexError,TypeError):
            return
        if not all(math.isfinite(a) and abs(a)<.8 for a in angles):return
        self.joint_angle=sum(angles)/2.
        self.joint_at=time.monotonic()

    def feedback_yaw(self,now):
        if self.imu_rate is not None and now-self.imu_at<.85:
            return self.imu_rate
        if self.actual_yaw_rate is not None and now-self.odom_at<.6:
            return self.actual_yaw_rate
        return None

    def feedback_steer(self,now):
        if self.joint_angle is not None and now-self.joint_at<.85:
            return self.joint_angle
        return None

    def motion_sensor_diag(self,now):
        return {'yaw_rate_source':('IMU' if self.imu_rate is not None and
                                   now-self.imu_at<.85 else
                                   'ODOM' if self.actual_yaw_rate is not None and
                                   now-self.odom_at<.6 else 'NONE'),
                'imu_yaw_rad_s':self.imu_rate if now-self.imu_at<.85 else None,
                'joint_steer_rad':self.feedback_steer(now)}
