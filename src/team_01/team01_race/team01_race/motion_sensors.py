"""Read IMU yaw rate and real front steering joints independently of plans."""
import math
import time
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import qos_profile_sensor_data
from team01_race.sensor_freshness import feedback_fresh
from team01_race.feedback_guard import FeedbackGuard


class MotionSensors:
    def init_motion_sensors(self):
        self.feedback_guard=FeedbackGuard(self.p)
        self.feedback_approved=False
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
        if (not math.isfinite(rate) or abs(rate)>8. or
                (len(covariance)>0 and covariance[0]<0)):
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

    def update_feedback_trust(self,now):
        sim_now=self.get_clock().now().nanoseconds*1e-9
        imu=(self.imu_rate if self.imu_rate is not None and
             self.fresh(now,self.imu_stamp,self.imu_at) else None)
        joint=(self.joint_angle if self.joint_angle is not None and
               self.fresh(now,self.joint_stamp,self.joint_at) else None)
        self.feedback_approved=self.feedback_guard.observe(
            sim_now,joint,imu,self.actual_speed or 0.,
            self.joint_stamp,self.imu_stamp)
        return imu,joint

    def feedback_yaw(self,now):
        imu,joint=self.update_feedback_trust(now)
        if self.feedback_approved:return imu
        if self.actual_yaw_rate is not None and now-self.odom_at<.6:
            return self.actual_yaw_rate
        return None

    def feedback_steer(self,now):
        if (self.feedback_approved and self.joint_angle is not None and
                self.fresh(now,self.joint_stamp,self.joint_at)):
            return self.joint_angle
        return None

    def record_steering(self,steering):
        stamp=self.get_clock().now().nanoseconds*1e-9
        # Motion publishes opposite sign of the internal left-positive target.
        self.feedback_guard.command_sent(-float(steering),stamp)

    def motion_sensor_diag(self,now):
        imu_ok=self.imu_rate is not None and self.fresh(now,self.imu_stamp,self.imu_at)
        status=self.feedback_guard.diag()
        status.update({'yaw_rate_source':('IMU' if self.feedback_approved else
                                         'ODOM' if self.actual_yaw_rate is not None and
                                         now-self.odom_at<.6 else 'NONE'),
                       'imu_yaw_rad_s':self.imu_rate if imu_ok else None,
                       'raw_joint_steer_rad':(self.joint_angle if
                                              self.fresh(now,self.joint_stamp,self.joint_at)
                                              else None),
                       'active_joint_steer_rad':self.feedback_steer(now),
                       'imu_sample_age_sim_s':(self.get_clock().now().nanoseconds*1e-9-
                                                self.imu_stamp if imu_ok else None)})
        return status
