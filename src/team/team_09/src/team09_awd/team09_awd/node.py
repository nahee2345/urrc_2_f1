"""Sensor-only v24 racer. The v12/v14 planner remains the baseline.

All path and speed decisions are made here. The actuator node only enforces
start/validity/timeout and converts independent speed/angle targets to joints.
"""
import csv
import json
import math
import os
from pathlib import Path
import threading
import time
from collections import deque
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2, Image, Imu, CameraInfo, JointState
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry, Path as PathMessage
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool, String
from ros_gz_interfaces.msg import Contacts
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
from .config import G, SETTINGS, load, topic, frame
from .core import Result, ResumeGate, WallTracker, external_points, compact, corridor_state, compensate, interpolate_pose, rollout
from .racing import plan_racing, CurveTracker
from .bypass import BypassState
from .recovery import escape_plan
from .overtake import Overtake
from .road_heading import plan_road
from .control import steer_step, govern_speed
from .model_builder import fingerprint
from .continuity import ScanBuffer, plan_is_current
from .depth import depth_meters, rgbd_preview, fit_depth_route
from .kinematics import euler, measured_curvature, transform_points
from .synchronization import RGBDPairer, AccelerationGuard
from .safe_log import BestEffortLog


def stamp_s(stamp):return stamp.sec+stamp.nanosec*1e-9


class Racer(Node):
    def __init__(self):
        super().__init__("awd_racer_v24_09")
        self.declare_parameter("config_file","");load(self.get_parameter("config_file").value)
        self.c=SETTINGS["control"];self.s=SETTINGS["sensors"]
        self.wall_settings = SETTINGS["right_wall"]
        self.road_heading_enabled = self.wall_settings["enabled"]
        self.road_settings = SETTINGS["road_heading"]
        self.trial_id = os.environ.get("URRC_V27_TRIAL_ID", "")
        self.track_name = os.environ.get("URRC_V27_TRACK", "")
        self.config_hash = fingerprint()
        self.pace_mode = os.environ.get("URRC_V27_PROFILE", "both")
        if self.pace_mode not in ("baseline", "pace", "both"):
            raise ValueError("URRC_V27_PROFILE must be baseline, pace or both")
        self.profile = f"v27 ROAD_PACE {self.pace_mode}" if self.road_heading_enabled else "v27 BASE_V14"
        self.lock=threading.RLock();self.state_group=MutuallyExclusiveCallbackGroup()
        self.history=deque(maxlen=400);self.pose=(0.,0.,0.);self.speed=0.;self.rate=0.;self.roll=0.;self.pitch=0.
        self.odom_at=-math.inf;self.imu_at=-math.inf;self.imu_stamp=-1.;self.imu_rate=0.;self.imu_accel=0.;self.imu_valid=False
        self.joint_stamp=-1.;self.joint_at=-math.inf;self.joint_k=0.;self.front_left=0.;self.front_right=0.
        self.epoch=0;self.reset_pending=False;self.distance=0.;self.last_pose=None
        self.pass_state=Overtake();self.scans=ScanBuffer();self.points=np.empty((0,2));self.cloud_at=-math.inf;self.cloud_stamp=-1.
        self.start=False;self.command=0.;self.k=0.;self.last_sim=None;self.result=Result();self.last_route="WAIT"
        self.curve_tracker=CurveTracker();self.wall_tracker=WallTracker();self.bypass_state=BypassState();self.gate=ResumeGate();self.guard=AccelerationGuard()
        self.bridge=CvBridge();self.camera_k=None;self.pairer=RGBDPairer(self.s["rgbd_pair_tolerance_s"],self.s["rgbd_queue_size"])
        self.depth_route=None;self.depth_stamp=-1.;self.depth_rgb_stamp=-1.;self.depth_quality=0.;self.depth_rows=0
        self.depth_status="NO_DEPTH";self.depth_slope=0.;self.depth_obstacles=np.empty((0,2));self.last_depth_attempt=-math.inf
        self.depth_rx=-math.inf;self.rgb_rx=-math.inf;self.depth_callback_status="WAIT_FRAMES"
        self.contact_count=0;self.contact_active=set();self.contact_seen={};self.feedback={}
        self.pub=self.create_publisher(AckermannDriveStamped,topic("candidate_drive"),1)
        self.status=self.create_publisher(String,topic("status"),10)
        self.plan_pub=self.create_publisher(PathMessage,topic("planned_path"),1)
        self.tf=TransformBroadcaster(self)
        latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2,topic("lidar/points"),self.cloud,latest)
        if self.c["use_depth"]:
            self.create_subscription(Image,topic("camera/image"),self.image,latest)
            self.create_subscription(Image,topic("camera/depth_image"),self.depth_image,latest)
            self.create_subscription(CameraInfo,topic("camera/camera_info"),self.camera_info,latest)
        self.create_subscription(Imu,topic("imu"),self.imu,latest,callback_group=self.state_group)
        self.create_subscription(Odometry,topic("odom"),self.odom,latest,callback_group=self.state_group)
        self.create_subscription(JointState,topic("joint_states"),self.joints,latest,callback_group=self.state_group)
        self.create_subscription(String,topic("actuator_status"),self.actuator_status,10)
        durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool,topic("race_start"),self.race_start,durable)
        self.contact_subs=[]
        for name in ("base_link","front_left_wheel","front_right_wheel","rear_left_wheel","rear_right_wheel"):
            self.contact_subs.append(self.create_subscription(Contacts,topic("contacts/"+name),lambda msg,n=name:self.contacts(msg,n),qos_profile_sensor_data))
        folder=SETTINGS["logging"]["log_dir"]
        folder=Path(folder).expanduser() if folder else Path(os.environ.get("XDG_STATE_HOME",str(Path.home()/".local/state")))/"urrc_awd"/SETTINGS["entry_id"]
        try:folder.mkdir(parents=True,exist_ok=True)
        except OSError:pass
        self.logfile=BestEffortLog(folder / (time.strftime("%Y%m%d-%H%M%S-") + self.pace_mode + "-v27.csv"))
        self.log=csv.writer(self.logfile)
        self.log.writerow(["sim_s","enabled","odom_x","odom_y","odom_yaw_rad","distance_m","speed_mps","yawrate_radps",
            "command_mps","curvature_1pm","free_m","clearance_m","centre_error_m","confidence","planning_ms","scan_age_sim_s",
            "contact_count","contact_monitor","reason","wall_heading_rad","wall_quality","imu_used","imu_yawrate","imu_accel_x",
            "route_kind","limiter","preview_limit_mps","road_curvature_1pm","vision_fused","depth_slope","depth_obstacle_points",
            "end_scan_age_s","odom_age_s","requested_curvature","roll_rad","pitch_rad","yaw_residual_radps","lateral_accel_est",
            "depth_state","depth_rows","depth_quality","front_left_rad","front_right_rad","measured_curvature_1pm","joint_stamp_s",
            "imu_stamp_s","odom_stamp_s","scan_stamp_s","rgb_stamp_s","depth_stamp_s","rgb_depth_skew_s","depth_callback_state",
            "route_transition","requested_curvature_delta","reaccel_guard","clearance_closing_mps","actuator_state","actuator_mps","speed_source","right_wall_error_m","profile",
            "right_gap_m","left_gap_m","wall_band","wall_relief_m","actuator_sim_s","drive_interruptions",
            "last_drive_interruption","last_drive_interruption_sim_s","command_age_wall_s","joint_age_wall_s",
            "imu_age_wall_s","pace_weight","base_preview_limit_mps","turn_lookahead_scale","paired_wall_span_m","pace_sweep_gap_m","trial_id","track","config_sha256"])
        self.last_log=-math.inf;self.last_status=-math.inf
        self.create_timer(1/self.c["planner_hz"],self.tick)
        self.get_logger().info(self.profile+" INDEPENDENT_ACTUATORS; CSV: "+self.logfile.name)

    def race_start(self,msg):
        self.start=bool(msg.data)
        if not self.start:self.command=0.

    def actuator_status(self,msg):
        try:self.feedback=json.loads(msg.data)
        except (ValueError,TypeError):pass

    def imu(self,msg):
        q=msg.orientation;rpy=euler((q.x,q.y,q.z,q.w));rate=msg.angular_velocity.z;accel=msg.linear_acceleration.x
        with self.lock:
            self.imu_valid=rpy is not None and msg.orientation_covariance[0]>=0 and all(math.isfinite(v) for v in (rate,accel))
            if not self.imu_valid:return
            self.roll,self.pitch=rpy[:2];self.imu_rate=rate;self.imu_accel=accel
            self.imu_at=time.monotonic();self.imu_stamp=stamp_s(msg.header.stamp)

    def joints(self,msg):
        if len(msg.position)!=len(msg.name):return
        data={n.split("::")[-1].split("/")[-1]:p for n,p in zip(msg.name,msg.position)}
        left=data.get("front_left_steer");right=data.get("front_right_steer")
        if left is None or right is None or not all(math.isfinite(v) for v in (left,right)):return
        with self.lock:
            self.front_left=left;self.front_right=right;self.joint_k=measured_curvature(left,right)
            self.joint_stamp=stamp_s(msg.header.stamp);self.joint_at=time.monotonic()

    def odom(self,msg):
        q=msg.pose.pose.orientation;rpy=euler((q.x,q.y,q.z,q.w));p=msg.pose.pose.position
        speed=msg.twist.twist.linear.x;rate=msg.twist.twist.angular.z;stamp=stamp_s(msg.header.stamp)
        if rpy is None or not all(math.isfinite(v) for v in (p.x,p.y,speed,rate)):return
        with self.lock:
            if self.history and stamp==self.history[-1][0]:return
            if self.history and stamp<self.history[-1][0]:
                self.history.clear();self.start=False;self.reset_pending=True;self.epoch+=1;self.last_pose=None
            if self.last_pose is not None:self.distance+=math.hypot(p.x-self.last_pose[0],p.y-self.last_pose[1])
            self.pose=(p.x,p.y,rpy[2]);self.last_pose=self.pose;self.speed=speed;self.rate=rate
            self.history.append((stamp,*self.pose));self.odom_at=time.monotonic()
        t=TransformStamped();t.header=msg.header;t.header.frame_id=frame("odom");t.child_frame_id=frame("base_link")
        t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.rotation=q;self.tf.sendTransform(t)

    def cloud(self,msg):
        try:
            p=point_cloud2.read_points_numpy(msg,field_names=("x","y","z"),skip_nans=True).reshape(-1,3)
            p=transform_points(p,self.s["lidar_pose"])
            height=p[:,2]-G.ground_z
            p=p[np.isfinite(p).all(axis=1)&(height>self.s["cloud_min_ground_height_m"])&(height<self.s["cloud_max_ground_height_m"])]
            self.points=external_points(p[:,:2]);self.cloud_at=time.monotonic();self.cloud_stamp=stamp_s(msg.header.stamp)
            self.scans.add(self.cloud_stamp,self.points,self.cloud_at)
        except (ValueError,TypeError,AssertionError) as exc:
            self.cloud_at=-math.inf;self.get_logger().error("Cloud decode: "+str(exc))

    def camera_info(self,msg):
        if msg.k[0]>0 and msg.k[4]>0:self.camera_k=(msg.k[0],msg.k[4],msg.k[2],msg.k[5])

    def image(self,msg):self.add_image("rgb",msg)
    def depth_image(self,msg):self.add_image("depth",msg)

    def add_image(self,kind,msg):
        if not self.c["use_depth"]:return
        if kind=="rgb":self.rgb_rx=time.monotonic()
        else:self.depth_rx=time.monotonic()
        pair=self.pairer.add(kind,stamp_s(msg.header.stamp),msg)
        if pair is None:self.depth_callback_status="WAIT_FRAME_PAIR";return
        rgb_stamp,rgb,depth_stamp,depth=pair
        if self.camera_k is None:self.depth_callback_status="WAIT_INTRINSICS";return
        if 0<=depth_stamp-self.last_depth_attempt<self.s["rgbd_process_interval_s"]:
            self.depth_callback_status="PROCESS_INTERVAL";return
        self.last_depth_attempt=depth_stamp
        try:
            color=self.bridge.imgmsg_to_cv2(rgb,"bgr8")
            meters=depth_meters(self.bridge.imgmsg_to_cv2(depth,"passthrough"),depth.encoding)
            route,obstacles,slope=rgbd_preview(color,meters,self.camera_k)
            fit=fit_depth_route(route)
            self.depth_route=route;self.depth_obstacles=obstacles;self.depth_slope=slope
            self.depth_stamp=depth_stamp;self.depth_rgb_stamp=rgb_stamp
            self.depth_rows=0 if route is None else len(route);self.depth_quality=0. if fit is None else fit[3]
            self.depth_status="ROUTE_VALID" if fit is not None else "NO_VALID_ROAD"
            self.depth_callback_status="PAIRED"
        except (ValueError,RuntimeError,cv2.error):
            self.depth_route=None;self.depth_status="DECODE_ERROR";self.depth_callback_status="DECODE_ERROR"

    def contacts(self,msg,name):
        self.contact_seen[name]=time.monotonic();hit=False
        for c in msg.contacts:
            pair=(c.collision1.name,c.collision2.name)
            other=[x for x in pair if SETTINGS["entry_id"]+"::" not in x]
            if other and not all("road_collision" in x or x.startswith("ground::") or "::ground::" in x for x in other):hit=True
        if hit and name not in self.contact_active:self.contact_count+=1
        if hit:self.contact_active.add(name)
        else:self.contact_active.discard(name)

    def reset_planning(self):
        self.pass_state=Overtake();self.scans=ScanBuffer();self.bypass_state=BypassState();self.curve_tracker=CurveTracker()
        self.wall_tracker=WallTracker();self.gate=ResumeGate();self.guard=AccelerationGuard()
        self.command=0.;self.k=0.;self.result=Result();self.last_route="WAIT"
        self.pairer=RGBDPairer(self.s["rgbd_pair_tolerance_s"],self.s["rgbd_queue_size"])
        self.depth_route=None;self.depth_stamp=-1.;self.last_depth_attempt=-math.inf

    def tick(self):
        # Every logged yaw value belongs to this single immutable state snapshot.
        with self.lock:
            history=tuple(self.history);pose=self.pose;speed=self.speed;odom_rate=self.rate;odom_at=self.odom_at
            imu_rate=self.imu_rate;imu_accel=self.imu_accel;imu_at=self.imu_at;imu_stamp=self.imu_stamp;imu_valid=self.imu_valid
            roll,pitch=self.roll,self.pitch;joint_k=self.joint_k;joint_stamp=self.joint_stamp;joint_at=self.joint_at
            left,right=self.front_left,self.front_right;epoch=self.epoch;reset=self.reset_pending;self.reset_pending=False;distance=self.distance
        if reset:self.reset_planning()
        stamp=self.get_clock().now().nanoseconds*1e-9;now=time.monotonic()
        dt=0. if self.last_sim is None else max(0.,min(.15,stamp-self.last_sim));self.last_sim=stamp
        c=self.c;duration=0.;reason="RUN";depth_pose=None
        imu_used=imu_valid and now-imu_at<c["sensor_wall_timeout_s"] and -.03<=stamp-imu_stamp<c["max_pose_age_s"]
        rate=imu_rate if imu_used else odom_rate
        joint_valid=now-joint_at<c["sensor_wall_timeout_s"] and -.03<=stamp-joint_stamp<c["max_pose_age_s"]
        selected=self.scans.select(history,stamp,max_age=c["max_pose_age_s"])
        source=None;scan_stamp=self.cloud_stamp
        if selected is not None:scan_stamp,points,arrival,source=selected
        scan_age=stamp-scan_stamp
        tilted=abs(roll)>math.radians(c["max_roll_deg"]) or abs(pitch)>math.radians(c["max_pitch_deg"])
        previous=joint_k if joint_valid else self.k
        prior_request=self.result.curvature
        if source is not None and not tilted and imu_used and joint_valid:
            begin=time.perf_counter();points=compensate(points,source,pose)
            depth_pose=interpolate_pose(history,self.depth_stamp)
            vision=None
            if (self.depth_route is not None and depth_pose is not None and
                    0<=stamp-self.depth_stamp<self.s["rgbd_valid_s"] and
                    (not self.road_heading_enabled or self.depth_status == "ROUTE_VALID")):
                vision=compensate(self.depth_route[:,:2],depth_pose,pose)
            if self.road_heading_enabled:
                self.result = plan_road(
                    points, previous, abs(speed), self.wall_settings, self.road_settings,
                    c["lateral_accel_mps2"], c["brake_mps2"],
                    max(c["minimum_latency_s"], scan_age+c["sensor_latency_reserve_s"]),
                    vision=vision, contact=bool(self.contact_active), yawrate=rate, profile=self.pace_mode, pass_state=self.pass_state, stamp=scan_stamp, pose=pose,
                )
            else:
                walls=self.wall_tracker.update(corridor_state(compact(points)),pose,stamp)
                self.result=plan_racing(points,previous,abs(speed),None,c["collision_margin_m"],yawrate=rate,wall_state=walls,vision=vision,
                    maximum=c["max_speed_mps"],lateral=c["lateral_accel_mps2"],brake=c["brake_mps2"],
                    latency=max(c["minimum_latency_s"],scan_age+c["sensor_latency_reserve_s"]),tracker=self.curve_tracker,
                    pose=pose,stamp=stamp,bypass_state=self.bypass_state)
                if self.result.mode=="BLOCKED" and abs(speed)<.6:
                    escape=escape_plan(points,previous,abs(speed))
                    if escape is not None:self.result=escape
            duration=(time.perf_counter()-begin)*1000
        r=self.result;route=getattr(r,"route_kind","WAIT")
        transition="" if route==self.last_route else self.last_route+"->"+route
        self.last_route=route
        if not self.start:reason="WAIT_START_LIGHTS"
        elif tilted:reason="ATTITUDE_STOP"
        elif not imu_used:reason="STALE_IMU"
        elif not joint_valid:reason="STALE_JOINTS"
        elif now-self.cloud_at>c["sensor_wall_timeout_s"] or not 0<=scan_age<=c["max_scan_age_s"]:reason="STALE_LIDAR"
        elif not history or now-odom_at>c["sensor_wall_timeout_s"] or not -.03<=stamp-history[-1][0]<=c["max_pose_age_s"]:reason="STALE_ODOM"
        elif source is None:reason="WAIT_ODOM_SYNC"
        elif r.confidence<.35 or r.mode=="BLOCKED":reason="NO_SAFE_PATH"
        safe=reason=="RUN"
        if not self.gate.update(safe,scan_stamp):
            if safe:reason="VERIFY_CLEAR"
        elif safe:reason=r.mode
        old_k=self.k
        if safe:
            # Start predictions from measured joints; command slew remains v12.
            self.k=steer_step(self.k,r.curvature,dt,abs(speed))
        previous_command=self.command
        self.command,limiter=govern_speed(r,self.command,self.k,abs(speed),rate,dt,c["max_speed_mps"],c["lateral_accel_mps2"],
            c["brake_mps2"],c["accel_mps2"],max(c["minimum_latency_s"],scan_age+c["sensor_latency_reserve_s"]),reason,imu_used)
        guard_reason="INACTIVE"
        if reason=="RUN" and not self.road_heading_enabled:
            self.command,guard_reason=self.guard.apply(self.command,previous_command,speed,r,stamp,c)
            if self.command<previous_command+ c["accel_mps2"]*dt-1e-7 and guard_reason not in ("CLEAR","OFF"):
                limiter="REACCEL_"+guard_reason
        finish=self.get_clock().now().nanoseconds*1e-9
        if not plan_is_current(stamp,finish,scan_stamp,epoch,self.epoch,max_age=c["max_scan_age_s"]):
            self.command=0.;reason="PLAN_DEADLINE";limiter=reason
        msg=AckermannDriveStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id=frame("base_link")
        msg.drive.speed=float(self.command);msg.drive.steering_angle=math.atan(G.wheelbase*self.k);self.pub.publish(msg)
        if safe and r.mode!="BLOCKED" and self.plan_pub.get_subscription_count():
            path=PathMessage();path.header=msg.header
            distances,poses=rollout(self.k,previous,abs(speed))
            if hasattr(r,"route_poses"):distances,poses=r.route_dist,r.route_poses
            for d,xyyaw in zip(distances,poses):
                if d>min(r.free,8.):break
                pp=PoseStamped();pp.header=path.header;pp.pose.position.x=float(xyyaw[0]);pp.pose.position.y=float(xyyaw[1])
                pp.pose.orientation.z=math.sin(float(xyyaw[2])/2);pp.pose.orientation.w=math.cos(float(xyyaw[2])/2);path.poses.append(pp)
            self.plan_pub.publish(path)
        fused=bool(getattr(r,"vision_fused",False)) and depth_pose is not None and 0<=stamp-self.depth_stamp<self.s["rgbd_valid_s"]
        depth_state="FUSED" if fused else "STALE" if stamp-self.depth_stamp>=self.s["rgbd_valid_s"] else self.depth_status
        monitor="CONTACT" if self.contact_active else "REPORTING" if len([x for x in self.contact_seen.values() if now-x<.7])>=2 else "UNKNOWN"
        if now-self.last_status>=1/SETTINGS["logging"]["status_hz"]:
            self.last_status=now
            feedback = self.feedback
            gap = getattr(r, "right_gap_m", None)
            gap_text = "unknown" if gap is None else f"{gap:.3f}"
            self.status.publish(String(data=(
                f"{self.profile} {reason} sim={stamp:.3f} route={route} limit={limiter} "
                f"wheel_speed={speed:.2f} cmd={self.command:.2f} drive={feedback.get('actuator_mps','?')} "
                f"actuator={feedback.get('state','UNKNOWN')} cuts={feedback.get('drive_interruptions','?')} "
                f"last_cut={feedback.get('last_drive_interruption')} right_m={gap_text} "
                f"band={getattr(r,'wall_band','-')} clear={r.clearance:.3f} free={r.free:.2f} "
                f"pace={getattr(r,'pace_weight',0.):.2f} look_scale={getattr(r,'turn_lookahead_scale',1.):.2f} "
                f"k={self.k:.3f} depth={depth_state} contact={monitor} plan_ms={duration:.1f}"
            )))
        if now-self.last_log>=1/SETTINGS["logging"]["csv_hz"]:
            self.last_log=now
            self.log.writerow([stamp,self.start,*pose,distance,speed,odom_rate,self.command,self.k,r.free,r.clearance,r.centre_error,r.confidence,
                duration,scan_age,self.contact_count,monitor,reason,r.heading,r.wall_quality,imu_used,imu_rate,imu_accel,route,limiter,
                getattr(r,"preview_limit",0.),getattr(r,"road_curvature",0.),fused,self.depth_slope,len(self.depth_obstacles),finish-scan_stamp,
                stamp-history[-1][0] if history else -1.,r.curvature,roll,pitch,rate-speed*joint_k,rate*speed,depth_state,self.depth_rows,self.depth_quality,
                left,right,joint_k,joint_stamp,imu_stamp,history[-1][0] if history else -1.,scan_stamp,self.depth_rgb_stamp,self.depth_stamp,
                self.depth_rgb_stamp-self.depth_stamp,self.depth_callback_status,transition,r.curvature-prior_request,guard_reason,self.guard.closing,
                self.feedback.get("state","UNKNOWN"),self.feedback.get("actuator_mps",""),"REAR_WHEEL_ESTIMATE",getattr(r,"right_wall_error_m",None),self.profile,
                getattr(r,"right_gap_m",None),getattr(r,"left_gap_m",None),getattr(r,"wall_band",""),
                getattr(r,"wall_relief_m",0.),self.feedback.get("sim_s"),self.feedback.get("drive_interruptions"),
                self.feedback.get("last_drive_interruption"),self.feedback.get("last_drive_interruption_sim_s"),
                self.feedback.get("command_age_wall_s"),self.feedback.get("joint_age_wall_s"),
                self.feedback.get("imu_age_wall_s"),getattr(r,"pace_weight",0.),getattr(r,"base_preview_limit",None),
                getattr(r,"turn_lookahead_scale",1.),getattr(r,"paired_wall_span",None),getattr(r,"projected_gap",None),self.trial_id,self.track_name,self.config_hash])
            self.logfile.flush()

    def close(self):
        if rclpy.ok():
            msg=AckermannDriveStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.drive.steering_angle=math.atan(G.wheelbase*self.k);self.pub.publish(msg)
        self.logfile.close();self.destroy_node()


def main():
    cv2.setNumThreads(1)
    rclpy.init();node=Racer();executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        executor.shutdown();node.close()
        if rclpy.ok():rclpy.shutdown()
