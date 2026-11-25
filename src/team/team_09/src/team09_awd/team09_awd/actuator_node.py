"""Gazebo adapter: camera start gate, command watchdog, independent actuators.

This node owns no path choice. It never subscribes to an operator world pose.
"""
import json
import math
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, Image
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Float64, String
from cv_bridge import CvBridge, CvBridgeError
from .config import G, SETTINGS, load, topic, frame
from .kinematics import WHEELS, IndependentActuator, WheelImuOdom, euler, quaternion, measured_curvature
from .start_signal import StartSignal

BUILD_ID = "v24.2 CAMERA_START_FIX"

def stamp_s(stamp): return stamp.sec + stamp.nanosec*1e-9


class Actuator(Node):
    def __init__(self):
        super().__init__("awd_actuator_v24_09")
        self.declare_parameter("config_file","")
        load(self.get_parameter("config_file").value)
        self.declare_parameter("race_start_topic","/race/start/"+SETTINGS["entry_id"])
        self.declare_parameter("race_start_mode","camera")
        self.gate=False; self.local_enable=True; self.signal=StartSignal(); self.bridge=CvBridge()
        self.gate_topic = self.get_parameter("race_start_topic").value
        self.gate_rx_count = 0; self.gate_at = None; self.gate_stamp = None
        self.camera_rx_count = 0; self.camera_at = None; self.camera_stamp = None
        self.camera_health = "NO_FRAMES"; self.last_debug = -math.inf
        self.last_go = None; self.last_reported = None
        self.command=None; self.command_at=-math.inf; self.joints=None; self.joint_at=-math.inf
        self.imu=None; self.imu_at=-math.inf; self.last_sim=None; self.last_status=-math.inf
        self.drive=IndependentActuator(); self.odometry=WheelImuOdom()
        self.actual_angle=0.; self.actual_speed=0.; self.reason="WAIT_SENSORS"
        self.drive_interruptions = 0
        self.last_drive_interruption = None
        self.last_drive_interruption_sim_s = None
        self.wheel_pub=[self.create_publisher(Float64,topic("drive/"+n),1) for n in WHEELS]
        self.steer_pub = self.create_publisher(Float64, topic("drive/steer_angle"), 1)
        self.steering_report_pub = self.create_publisher(Float32, topic("steering"), 1)
        self.speed_pub = self.create_publisher(Float32, topic("speed"), 1)
        self.odom_pub=self.create_publisher(Odometry,topic("odom"),10)
        self.feedback=self.create_publisher(String,topic("actuator_status"),10)
        self.debug_pub = self.create_publisher(Image, topic("start_camera/debug_image"), qos_profile_sensor_data)
        durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.go_pub=self.create_publisher(Bool,topic("race_start"),durable)
        # StartGate in urrc_2 uses a reliable, volatile publisher. It is already
        # discovered before spawn and sends the live start/stop events.
        host_gate_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Bool, self.gate_topic, self.on_gate, host_gate_qos
        )
        self.create_subscription(Bool,topic("enable"),self.on_enable,10)
        self.create_subscription(AckermannDriveStamped,topic("candidate_drive"),self.on_command,1)
        self.create_subscription(Imu,topic("imu"),self.on_imu,qos_profile_sensor_data)
        self.create_subscription(JointState,topic("joint_states"),self.on_joints,qos_profile_sensor_data)
        self.create_subscription(Image,topic("start_camera/image"),self.on_camera,qos_profile_sensor_data)
        # A steady timer continues to withdraw drive on a frozen/missing /clock.
        self.timer=self.create_timer(1/SETTINGS["actuator"]["hz"],self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(BUILD_ID + "; host=" + self.gate_topic +
                               " qos=RELIABLE/VOLATILE; require observed five-red then all-five-off")
        self.publish_start()

    def publish_start(self):
        go = bool(self.gate and self.local_enable and self.signal.state == "GO")
        if self.last_go != go:
            self.go_pub.publish(Bool(data=go))
            self.last_go = go
        return go

    def on_gate(self,msg):
        self.gate_rx_count += 1
        self.gate_at = time.monotonic()
        self.gate_stamp = self.get_clock().now().nanoseconds*1e-9
        if self.gate and not msg.data:
            self.signal=StartSignal(); self.command=None
        self.gate=bool(msg.data)
        self.get_logger().info(f"[START HOST] received={self.gate} rx={self.gate_rx_count} camera={self.signal.state}")
        self.publish_start()

    def on_enable(self,msg):
        self.local_enable=bool(msg.data)
        if not self.local_enable:
            self.command=None; self.signal=StartSignal()
        self.publish_start()

    def on_command(self,msg):
        v,a=msg.drive.speed,msg.drive.steering_angle
        if not all(math.isfinite(x) for x in (v,a)) or not -SETTINGS["actuator"]["max_reverse_mps"]-.001<=v<=SETTINGS["control"]["max_speed_mps"]+.001 or abs(a)>G.max_centre_angle+.001:
            self.command=None;return
        self.command=(stamp_s(msg.header.stamp),v,a);self.command_at=time.monotonic()

    def on_camera(self,msg):
        now = time.monotonic()
        self.camera_rx_count += 1; self.camera_at = now
        self.camera_stamp = stamp_s(msg.header.stamp)
        if self.signal.state == "GO":
            self.camera_health = "LATCHED_GO"
            return
        sim = self.get_clock().now().nanoseconds*1e-9
        settings = SETTINGS["start_signal"]
        age = sim-self.camera_stamp
        if not -settings["camera_future_tolerance_s"] <= age <= settings["camera_max_age_s"]:
            self.camera_health = "CAMERA_STAMP_OUT_OF_RANGE"
            self.signal.off_frames = 0
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            before = self.signal.state
            self.signal.update(image, self.camera_stamp)
            if self.signal.reason == "INVALID_IMAGE":
                self.camera_health = "INVALID_IMAGE"
                self.publish_start()
                return
            self.camera_health = "FRAMES_OK"
            if before != self.signal.state:
                self.get_logger().info(f"[START CAMERA] {before} -> {self.signal.state} sim={self.camera_stamp:.3f} "
                                       f"reason={self.signal.reason} host={self.gate}")
            self.publish_start()
            if self.debug_pub.get_subscription_count() and (before != self.signal.state or now-self.last_debug >= 1/settings["debug_hz"]):
                debug = self.bridge.cv2_to_imgmsg(self.signal.annotate(image), "bgr8")
                debug.header = msg.header
                self.debug_pub.publish(debug)
                self.last_debug = now
        except (ValueError,RuntimeError,cv2.error,CvBridgeError) as exc:
            self.signal.reset("FRAME_DECODE_ERROR")
            self.camera_health = "FRAME_DECODE_ERROR"
            self.publish_start()
            self.get_logger().error("[START CAMERA] decode failed: " + str(exc))

    def on_imu(self,msg):
        q=msg.orientation; rpy=euler((q.x,q.y,q.z,q.w))
        if msg.orientation_covariance[0]<0 or rpy is None or not math.isfinite(msg.angular_velocity.z):
            self.imu=None;return
        self.imu=(stamp_s(msg.header.stamp),rpy,float(msg.angular_velocity.z));self.imu_at=time.monotonic()

    def on_joints(self,msg):
        names=[n.split("::")[-1].split("/")[-1] for n in msg.name]
        if len(msg.position)!=len(names) or len(msg.velocity)!=len(names):return
        data={n:(p,v) for n,p,v in zip(names,msg.position,msg.velocity)}
        needed=[n+"_drive" for n in WHEELS]+["front_left_steer","front_right_steer"]
        if any(n not in data for n in needed) or not all(math.isfinite(x) for n in needed for x in data[n]):return
        stamp=stamp_s(msg.header.stamp);self.joints=(stamp,data);self.joint_at=time.monotonic()
        k=measured_curvature(data["front_left_steer"][0],data["front_right_steer"][0])
        self.actual_angle=math.atan(G.wheelbase*k)
        self.actual_speed=G.radius*(data["rear_left_drive"][1]+data["rear_right_drive"][1])/2
        if abs(self.actual_speed)>SETTINGS["actuator"]["joint_speed_limit_mps"] or self.imu is None:return
        if abs(stamp-self.imu[0])>SETTINGS["actuator"]["state_pair_skew_s"]:return
        x,y,yaw,v,vy=self.odometry.update(stamp,data["rear_left_drive"][1],data["rear_right_drive"][1],self.imu[1],self.imu[2])
        odom=Odometry();odom.header=msg.header;odom.header.frame_id=frame("odom");odom.child_frame_id=frame("base_link")
        odom.pose.pose.position.x=x;odom.pose.pose.position.y=y
        q=quaternion(self.imu[1][0],self.imu[1][1],yaw)
        odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=q
        odom.twist.twist.linear.x=v;odom.twist.twist.linear.y=vy;odom.twist.twist.angular.z=self.imu[2]
        # Conservative declared uncertainty, not a filter-estimated covariance.
        for i in (0,7,14): odom.pose.covariance[i]=1.;odom.twist.covariance[i]=.25
        for i in (21,28,35): odom.pose.covariance[i]=.04;odom.twist.covariance[i]=.04
        self.odom_pub.publish(odom)

    def tick(self):
        now=time.monotonic();sim=self.get_clock().now().nanoseconds*1e-9
        dt=0. if self.last_sim is None else sim-self.last_sim
        if dt<0:
            self.gate=False;self.signal=StartSignal();self.command=None;self.joints=None;self.imu=None
            self.drive=IndependentActuator();self.odometry=WheelImuOdom()
        self.last_sim=sim; a=SETTINGS["actuator"];c=SETTINGS["control"]
        go=self.publish_start()
        reason="PASS"
        if not self.local_enable:reason="DISABLED"
        elif not self.gate:reason="WAIT_HOST_GATE"
        elif self.signal.state!="GO":reason="WAIT_"+self.signal.state
        elif self.joints is None or now-self.joint_at>a["state_timeout_s"] or not 0<=sim-self.joints[0]<=a["state_timeout_s"]:reason="STALE_JOINTS"
        elif self.imu is None or now-self.imu_at>a["state_timeout_s"] or not 0<=sim-self.imu[0]<=a["state_timeout_s"]:reason="STALE_IMU"
        elif abs(self.imu[1][0])>math.radians(c["max_roll_deg"]) or abs(self.imu[1][1])>math.radians(c["max_pitch_deg"]):reason="ATTITUDE_STOP"
        elif abs(self.actual_speed)>a["joint_speed_limit_mps"]:reason="INVALID_WHEEL_SPEED"
        elif self.command is None or now-self.command_at>a["command_timeout_s"] or not 0<=sim-self.command[0]<=a["command_timeout_s"]:reason="COMMAND_TIMEOUT"
        if reason=="PASS":v,steer=self.command[1:]
        else:
            v=0.
            steer=self.drive.steer if reason=="COMMAND_TIMEOUT" else self.actual_angle
        # When paused, a zero wheel setpoint must be waiting before resume.
        if reason!="PASS" and dt<=0:self.drive.speed=0.
        speed,angle,wheels=self.drive.step(v,steer,max(0.,dt))
        for pub,w in zip(self.wheel_pub,wheels):pub.publish(Float64(data=float(w)))
        self.steer_pub.publish(Float64(data=float(angle)))
        self.steering_report_pub.publish(Float32(data=float(angle)))
        self.speed_pub.publish(Float32(data=float(speed)))
        if self.reason == "PASS" and reason != "PASS" and go:
            self.drive_interruptions += 1
            self.last_drive_interruption = reason
            self.last_drive_interruption_sim_s = sim
        self.reason=reason
        if now-self.last_status>=1/SETTINGS["logging"]["status_hz"]:
            self.last_status=now;self.go_pub.publish(Bool(data=bool(go)))
            joints={} if self.joints is None else self.joints[1]
            camera_health = self.camera_health
            if self.signal.state != "GO" and self.camera_at is not None and now-self.camera_at > SETTINGS["start_signal"]["camera_timeout_wall_s"]:
                camera_health = "CAMERA_TIMEOUT"
            summary = (reason, self.signal.state, camera_health)
            if summary != self.last_reported:
                self.get_logger().info(f"[START/DRIVE] state={reason} host={self.gate} rx={self.gate_rx_count} "
                                       f"camera={self.signal.state}/{self.signal.reason}/{camera_health}")
                self.last_reported = summary
            detail=dict(build=BUILD_ID,sim_s=sim,state=reason,host_gate=self.gate,signal=self.signal.state,
                        start_authorized=go,host_gate_topic=self.gate_topic,
                        host_gate_qos="RELIABLE/VOLATILE",host_gate_rx_count=self.gate_rx_count,
                        host_gate_publishers=self.count_publishers(self.gate_topic),host_gate_stamp_s=self.gate_stamp,
                        host_gate_age_wall_s=None if self.gate_at is None else now-self.gate_at,
                        camera_health=camera_health,camera_rx_count=self.camera_rx_count,
                        camera_stamp_s=self.camera_stamp,
                        camera_age_wall_s=None if self.camera_at is None else now-self.camera_at,
                        target_mps=v,actuator_mps=speed,wheel_speed_mps=self.actual_speed,
                        steer_target_rad=angle,steer_measured_rad=self.actual_angle,
                        front_left_rad=joints.get("front_left_steer",(None,))[0],front_right_rad=joints.get("front_right_steer",(None,))[0],
                        joint_stamp_s=None if self.joints is None else self.joints[0],imu_stamp_s=None if self.imu is None else self.imu[0],
                        speed_source="REAR_WHEEL_ESTIMATE",attitude_source="IMU_ORIENTATION")
            detail.update(
                drive_interruptions=self.drive_interruptions,
                last_drive_interruption=self.last_drive_interruption,
                last_drive_interruption_sim_s=self.last_drive_interruption_sim_s,
                command_age_wall_s=None if self.command is None else now-self.command_at,
                joint_age_wall_s=None if self.joints is None else now-self.joint_at,
                imu_age_wall_s=None if self.imu is None else now-self.imu_at,
            )
            detail.update(self.signal.diagnostics())
            self.feedback.publish(String(data=json.dumps(detail,allow_nan=False)))

    def stop(self):
        for pub in self.wheel_pub:pub.publish(Float64(data=0.))
        self.speed_pub.publish(Float32(data=0.))
        self.go_pub.publish(Bool(data=False))


def main():
    cv2.setNumThreads(1)
    rclpy.init();node=Actuator()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.stop()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
