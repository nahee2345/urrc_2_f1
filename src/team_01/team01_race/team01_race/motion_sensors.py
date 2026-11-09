"""Read IMU yaw rate and real front steering joints independently of plans."""
import math
import time
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import qos_profile_sensor_data
from team01_race.sensor_freshness import feedback_fresh


class MotionSensors:
    def init_motion_sensors(self):
        self.imu_rate=None
        self.imu_at=0.
        self.imu_stamp=0.
        self.joint_angle=None
        self.joint_at=0.
        self.joint_stamp=0.
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
        self.imu_stamp=self.stamp(msg)

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
        self.joint_stamp=self.stamp(msg)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+1e-9*msg.header.stamp.nanosec

    def fresh(self,now,stamp,last_wall):
        sim_now=self.get_clock().now().nanoseconds*1e-9
        return feedback_fresh(sim_now,stamp,now,last_wall)

    def feedback_yaw(self,now):
        if self.imu_rate is not None and self.fresh(now,self.imu_stamp,self.imu_at):
            return self.imu_rate
        if self.actual_yaw_rate is not None and now-self.odom_at<.6:
            return self.actual_yaw_rate
        return None

    def feedback_steer(self,now):
        if self.joint_angle is not None and self.fresh(now,self.joint_stamp,self.joint_at):
            return self.joint_angle
        return None

    def motion_sensor_diag(self,now):
        imu_good=self.imu_rate is not None and self.fresh(now,self.imu_stamp,self.imu_at)
        return {'yaw_rate_source':('IMU' if imu_good else
                                   'ODOM' if self.actual_yaw_rate is not None and
                                   now-self.odom_at<.6 else 'NONE'),
                'imu_yaw_rad_s':self.imu_rate if imu_good else None,
                'joint_steer_rad':self.feedback_steer(now)}
