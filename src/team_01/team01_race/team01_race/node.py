"""ROS2 local racer: camera-based start, depth road, lidar safety, rear defence."""
import math
import time
import os
import json
from pathlib import Path
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from team01_race.perception import (decode_depth, decode_rgb, depth_geometry,
                                    laser_to_xy)
from team01_race.planning import TrafficGate, SafetyPlanner
from team01_race.road_surface import (dark_asphalt_bounds, transverse_start_marking,
                                       aligned_depth_rgb)
from team01_race.signal_vision import SignalVision
from team01_race.node_diagnostics import DiagnosticsMixin
from team01_race.track_memory import TrackMemory
from team01_race.mode_recovery import RecoveryGuard
from team01_race.road_node import RoadPerception
from team01_race.bev_node import BEVPublisher
from team01_race.road_roi import road_roi_metrics
from team01_race.start_diagnostics import StartDiagnosticsMixin
from team01_race.defaults import DEFAULTS

class Racer(Node, DiagnosticsMixin, StartDiagnosticsMixin, BEVPublisher):
    def __init__(self):
        super().__init__('team01_control')
        for k,v in DEFAULTS.items(): self.declare_parameter(k,v)
        for k,v in [('race_start_topic','/race/start/team01'),('speed_topic','/team01/speed'),
                    ('steering_topic','/team01/steering'),('lidar_topic','/team01/front_scan'),
                    ('race_start_mode','camera'),('use_sim_time',True)]:
            if not self.has_parameter(k):self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in DEFAULTS}
        self.start_mode=str(self.get_parameter('race_start_mode').value)
        self.permission=False
        self.gate=TrafficGate(self.p['traffic_light_min_red_blobs'],
                              self.p['traffic_light_ready_frames'],
                              self.p['traffic_light_off_frames'])
        self.signal_vision=SignalVision()
        self.planner=SafetyPlanner(self.p)
        self.motion_guard=RecoveryGuard(self.p)
        self.track=TrackMemory(self.p)
        self._setup_map_tf()
        self.depth_obstacles=np.empty((0,2))
        self.depth_valid=False
        self.road_image=None
        self.road_at=0.
        self.ground_xy=np.empty((0,2))
        self.ground_uv=np.empty((0,2),dtype=int)
        self.ground_at=0.
        self.depth_shape=None
        self.obstacle_uv=np.empty((0,2),dtype=int)
        self.obstacle_height=np.empty(0,dtype=float)
        self.colored_wall_xy=np.empty((0,2),dtype=float)
        self.wall_color_matches=0
        self.road_perception=RoadPerception()
        self.ground_count=0
        self.road_bounds=None
        self.asphalt_coverage=0.
        self.start_line_seen=False
        self._lap_record_saved=False
        self._last_partial_record_distance=-10.
        self.lidar_obstacles=np.empty((0,2))
        self.rear_gap=None
        self.depth_at=self.lidar_at=self.rgb_at=self.rear_at=0.
        self.last_tick=time.monotonic()
        self.last_log=0.
        self.actual_speed=None
        self.actual_yaw_rate=None
        self.odom_at=0.
        self.rgb_blobs=0
        self.rgb_strength=0.
        self._snapshots=set()
        self._last_rgb_save=0.
        self.camera_debug_dir=(Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state')))
                               /'urrc_team01'/'camera')
        self.road_debug_dir=self.camera_debug_dir.parent/'road'
        self._road_debug_saved=set()
        self._road_last_save=0.
        self.slope=0.
        self.pub_speed=self.create_publisher(Float32,self.get_parameter('speed_topic').value,10)
        self.pub_steer=self.create_publisher(Float32,self.get_parameter('steering_topic').value,10)
        self.pub_path=self.create_publisher(String,'/team01/debug/path',10)
        self.init_start_diagnostics()
        self.init_bev()
        self.sub_start=self.create_subscription(Bool,self.get_parameter('race_start_topic').value,
                                                self.on_start,10)
        self.create_subscription(Image,'/team01/front_depth',self.on_depth,qos_profile_sensor_data)
        self.create_subscription(Image,'/team01/front_rgb',self.on_rgb,qos_profile_sensor_data)
        self.create_subscription(Image,'/team01/road_rgb',self.on_road_rgb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/team01/gz_odom',self.on_odometry,qos_profile_sensor_data)
        self.create_subscription(LaserScan,self.get_parameter('lidar_topic').value,
                                 self.on_front_scan,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/team01/rear_scan',self.on_rear_scan,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)
        self.get_logger().info(f'Fail-closed racer online. start_mode={self.start_mode}')

    def on_start(self,msg):
        if not msg.data and self.permission:
            # Reset only after a previous authorization; false during lighting
            # must not erase the full-red observations.
            self.gate=TrafficGate(self.p['traffic_light_min_red_blobs'],
                                  self.p['traffic_light_ready_frames'],
                                  self.p['traffic_light_off_frames'])
            # Do not reuse an earlier course trace on a new race start.
            self.planner=SafetyPlanner(self.p)
            self.motion_guard=RecoveryGuard(self.p)
            self.track=TrackMemory(self.p)
            self._lap_record_saved=False
            self._last_partial_record_distance=-10.
        self.permission=bool(msg.data)
        self.get_logger().info(f'RACE_PERMISSION={self.permission}')
        if not self.permission:
            self.planner.speed=0.
            self.pub_speed.publish(Float32(data=0.))
            self.pub_steer.publish(Float32(data=0.))

    def on_depth(self,msg):
        try:
            d=decode_depth(msg)
            scene=depth_geometry(d, self.p['camera_hfov_deg'],
                                 self.p['sensor_pitch_down_deg'],
                                 mount_z=self.p['sensor_mount_height_m'],
                                 min_road_points=self.p['min_road_points'])
            self.depth_obstacles=scene.obstacles
            self.depth_shape=d.shape
            self.obstacle_uv=scene.obstacle_uv
            self.obstacle_height=scene.obstacle_height
            self.depth_valid=scene.valid
            self.slope=scene.slope_deg
            self.ground_xy=scene.ground_xy
            self.ground_uv=scene.ground_uv
            self.ground_count=len(scene.ground_xy)
            self.ground_at=time.monotonic()
            self._refresh_road()
        except Exception as exc:
            self.depth_valid=False
            self.ground_xy=np.empty((0,2))
            self.ground_uv=np.empty((0,2),dtype=int)
            self.ground_count=0
            self.ground_at=0.
            self.depth_shape=None
            self.road_bounds=None
            self.start_line_seen=False
            self.get_logger().warning(f'depth/road rejected: {exc}')
        self.depth_at=time.monotonic()

    def on_road_rgb(self,msg):
        try:
            self.road_image=decode_rgb(msg)
            self.road_at=time.monotonic()
            # Cameras arrive independently on software-rendered Gazebo.
            self._refresh_road()
        except Exception as exc:
            self.road_image=None
            self.road_at=0.
            self.get_logger().warning(f'Road RGB rejected: {exc}')

    def _refresh_road(self):
        self.road_perception.refresh(self)

    def on_rgb(self,msg):
        try:
            rgb=decode_rgb(msg)
            # Spatial temporal delta: stationary red guardrails and dark lenses
            # must NEVER be counted as illuminated start lights.
            self.rgb_blobs,self.rgb_strength=self.signal_vision.observe(rgb)
            self.gate.observe(self.rgb_blobs,self.rgb_strength)
            # Camera evidence makes subsequent on-machine diagnosis possible.
            # Includes the FULL image, not just the algorithm crop.
            now=time.monotonic()
            if self.signal_vision.ready:
                if 'dark' not in self._snapshots and self.signal_vision.peak_blobs==0:
                    self._save_rgb(rgb,'dark')
                if self.rgb_blobs>=4 and 'armed' not in self._snapshots:
                    self._save_rgb(rgb,'armed')
                if self.gate.lights_out and 'lights_out' not in self._snapshots:
                    self._save_rgb(rgb,'lights_out')
                if now-self._last_rgb_save>3.:
                    self._save_rgb(rgb,'latest')
                    self._last_rgb_save=now
            if self.gate.lights_out and self.gate.status=='LIGHTS_OUT' and not getattr(self,'start_logged',False):
                self.get_logger().info('CAMERA LIGHTS OUT verified: motion permission now possible')
                self.start_logged=True
            self.rgb_at=time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f'RGB rejected: {exc}')

    def _map_pose(self):
        """Use only an independently estimated map->base_link TF."""
        if self.map_tf is None:return None
        try:
            from rclpy.time import Time
            t=self.map_tf.lookup_transform('team01_map','base_link',Time())
            stamp=t.header.stamp
            now=self.get_clock().now().nanoseconds*1e-9
            age=now-(stamp.sec+stamp.nanosec*1e-9)
            if age>0.8 or age<-.5:return None
            q=t.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            return (float(t.transform.translation.x),float(t.transform.translation.y),yaw)
        except Exception:
            return None

    def _setup_map_tf(self):
        self.map_tf=None
        try:
            from tf2_ros import Buffer, TransformListener
            self.map_tf=Buffer()
            self.map_listener=TransformListener(self.map_tf,self)
        except ImportError:
            self.get_logger().warning('TF2 unavailable; mapped lap disabled')

    def on_odometry(self,msg):
        # Velocity only; NO world pose, no simulator track coordinates.
        try:
            value=float(msg.twist.twist.linear.x)
            if math.isfinite(value):
                self.actual_speed=abs(value)
                self.start_fallback.observe_spawn(time.monotonic())
                yaw=float(msg.twist.twist.angular.z)
                self.actual_yaw_rate=yaw if math.isfinite(yaw) else None
                self.odom_at=time.monotonic()
        except (AttributeError,ValueError,TypeError):
            pass

    def on_front_scan(self,msg):
        self.lidar_obstacles=laser_to_xy(msg.ranges,msg.angle_min,
                                         msg.angle_increment,sensor_x=.187)
        self.lidar_at=time.monotonic()

    def on_rear_scan(self,msg):
        cloud=laser_to_xy(msg.ranges,msg.angle_min,msg.angle_increment,
                          sensor_x=-.208,heading=math.pi)
        mask=(cloud[:,0]<-.30)&(np.abs(cloud[:,1])<.50)
        self.rear_gap=float(np.min(-cloud[mask,0])) if np.any(mask) else None
        self.rear_at=time.monotonic()

    def tick(self):
        now=time.monotonic()
        dt=now-self.last_tick
        self.last_tick=now
        enabled=self.authorized(now)
        d_ok=self.depth_valid and now-self.depth_at<self.p['depth_timeout_s']
        l_ok=now-self.lidar_at<self.p['lidar_timeout_s']
        # Stale rear sensor forbids lane-change overtakes, not ordinary driving.
        rear_gap=self.rear_gap if now-self.rear_at<self.p['lidar_timeout_s'] else -1.0
        visual_road=(self.road_bounds
                     if self.road_bounds is not None and
                     now-self.road_at<self.p['road_rgb_timeout_s'] and
                     now-self.ground_at<self.p['depth_timeout_s'] and
                     abs(self.road_at-self.ground_at)<=self.p['road_sync_max_age_s']
                     else None)
        pose=self._map_pose()
        mapped=self.track.target(pose,max(2.5,min(6.,2.+(self.actual_speed or 0.)*.35)))
        sim_dt=self.simulated_elapsed_s()
        motion=self.planner.step(self.depth_obstacles,self.lidar_obstacles,
                                 rear_gap,dt,enabled,d_ok,l_ok,visual_road,
                                 actual_speed=(self.actual_speed if now-self.odom_at<.6 else None),
                                 actual_yaw_rate=(self.actual_yaw_rate if now-self.odom_at<.6 else None),
                                 map_target=mapped,map_ready=(self.track.complete and mapped is not None and pose is not None),
                                 distance_dt=sim_dt)
        motion=self.motion_guard.update(motion,dt)
        if motion.mode=='RECOVER':
            self.planner.speed=0.
        if self.road_image is not None and now-self.road_at<self.p['road_rgb_timeout_s']:
            if now-self._road_last_save>5.:
                self._save_road('latest')
                self._road_last_save=now
            if self.planner.edge_alert=='CAUTION' and 'edge_caution' not in self._road_debug_saved:
                self._save_road('edge_caution')
            if (self.planner.edge_alert=='CRITICAL' and
                    'edge_critical' not in self._road_debug_saved):
                self._save_road('edge_critical')
            if motion.mode=='BRAKE' and 'first_brake' not in self._road_debug_saved:
                self._save_road('first_brake')
        # Provisional lap learning needs independent sensor evidence of the
        # starting stripe, distance/heading closure and curb signature.
        learning=self.planner.lap_learner
        if enabled and d_ok and motion.speed>0:
            self.track.observe(pose,self.planner.road)
        if enabled and d_ok and learning.observe_line(self.start_line_seen,self.planner.road):
            if self.track.try_close(pose):
                self.track.save()
                self.get_logger().info('SLAM track map closed; lap-2 mapped driving enabled')
            else:
                learning.lap=1
                self.get_logger().warning('Lap marker reached but SLAM map unverified; remain in exploration')
            self.get_logger().info('LAP 1 SENSOR-CANDIDATE COMPLETE; cautious learned lap 2 enabled')
            self._write_lap_record()
        # Save newly observed features during the first lap, even if the car
        # never completes. This file is NOT read at subsequent starts.
        if (enabled and learning.lap==1 and
                learning.distance_m-self._last_partial_record_distance>=10.):
            self._write_learning_progress()
            self._last_partial_record_distance=learning.distance_m
        # No early movement: speed repeatedly 0 while gate is unfulfilled.
        self.pub_speed.publish(Float32(data=float(motion.speed)))
        self.pub_steer.publish(Float32(data=float(motion.steering)))
        self.publish_start_diagnostics(now,motion,d_ok,l_ok)
        # High-rate evidence: whether a trajectory EXISTS and is selected.
        path=self.planner.reference_geometry
        payload={'mode':motion.mode,'reason':motion.reason,
                 'reference_valid':path is not None,
                 'reference_count':0 if path is None else len(path),
                 'trajectory_generation':self.planner.reference_scheduler.count,
                 'trajectory_source':self.planner.reference_scheduler.source,
                 'planner':self.planner.planner_type,
                 'painted_wall_matches':self.wall_color_matches,
                 'road_roi':road_roi_metrics(self.planner.road),
                 'road_disagreement_m':self.planner.road_belief.disagreement_m,
                 'road_innovation_m':self.planner.road_belief.innovation_m,
                 'road_conflict_reason':self.planner.road_belief.reason,
                 'candidate_rejections':getattr(self.planner,'candidate_rejections',[]),
                 'candidate_shift_m':self.planner.adaptive_shift_m,
                 'odom_sim_dt_s':round(sim_dt,4),
                 'candidate_xy':([] if self.planner.adaptive_points is None else np.round(self.planner.adaptive_points[::max(1,len(self.planner.adaptive_points)//20)],3).tolist()),
                 'road_bounds_xy':([] if self.planner.road.bounds is None else np.round(self.planner.road.bounds[::max(1,len(self.planner.road.bounds)//16)],3).tolist()),
                 'fault_transitions':self.motion_guard.transitions,
                 'recovery_hold':self.motion_guard.active,
                 'last_fault':self.motion_guard.last_fault,
                 'cmd_speed':round(float(motion.speed),3),
                 'cmd_steer':round(float(motion.steering),5),
                 'odom_speed':self.actual_speed if now-self.odom_at<.6 else None,
                 'odom_yaw_rate':self.actual_yaw_rate if now-self.odom_at<.6 else None,
                 'reference_xy':([] if path is None else
                                 np.round(path[::max(1,len(path)//24)],3).tolist())}
        self.pub_path.publish(String(data=json.dumps(payload,separators=(',',':'))))
        self.publish_bev(now)
        if now-self.last_log>1.:
            self.get_logger().info(f'{motion.mode} v={motion.speed:.2f} '
                f'steer={math.degrees(motion.steering):+.1f}deg '
                f'grade={self.slope:+.1f}deg ready={self.gate.saw_full_red} '
                f'out={self.gate.lights_out} sensor={d_ok and l_ok} '
                f'RGB={self.rgb_blobs} intensity={self.rgb_strength:.0f} '
                f'gate={self.gate.status} calibrated={self.signal_vision.ready} '
                f'peak={self.signal_vision.peak_blobs} permission={self.permission} '
                f'dynamic_area={self.signal_vision.last_coverage:.4f} '
                f'first_red_at={self.gate.first_bright_time} '
                f'road={self.planner.road.valid} reason={motion.reason} '
                f'depth_age={now-self.depth_at:.2f} lidar_age={now-self.lidar_at:.2f} '
                f'road_age={now-self.road_at:.2f} rgb_age={now-self.rgb_at:.2f} '
                f'ground_count={self.ground_count} lidar_points={len(self.lidar_obstacles)} '
                f'vision_bounds={0 if self.road_bounds is None else len(self.road_bounds)} '
                f'road_heading={self.planner.road.heading_deg:+.1f} '
                f'source={self.planner.road_source} black={self.asphalt_coverage:.2f} '
                f'lead={self.planner.lead_distance} lead_v={self.planner.lead_speed} '
                f'pass_at={self.planner.overtake_start_distance_m} '
                f'pass_length={self.planner.planned_pass_distance_m} '
                f'line={self.start_line_seen} lap={learning.lap} '
                f'lap_distance_est={learning.distance_m:.1f} '
                f'lap_yaw_est={learning.yaw_rad:+.2f} '
                f'line_phase={self.planner.racing_line.phase} '
                f'startup_s={self.planner.racing_line.startup_elapsed_s:.1f} '
                f'right_ref={self.planner.racing_line.right_reference_m} '
                f'right_error={self.planner.startup_right_error_m} '
                f'passing={self.planner.rejoin.phase} map={self.track.complete} '
                f'track_points={len(learning.first_lap_features)} '
                f'odom_v={self.actual_speed if now-self.odom_at<.6 else None} '
                f'odom_yaw_rate={self.actual_yaw_rate if now-self.odom_at<.6 else None} '
                f'planner={self.planner.planner_type} curve={self.planner.preview_curve:.3f} '
                f'recenter={self.planner.recenter_reason} '
                f'hold_offset={self.planner.racing_line.held_lane_offset_m} '
                f'front_wall_m={self.planner.static_wall_distance_m} '
                f'front_wall_world_v={self.planner.front_wall_speed_mps} '
                f'curb_ahead={self.planner.racing_line.last_distances.ahead_m:.2f} '
                f'curb_confirm={self.planner.racing_line.confirmed_frames} '
                f'curb_left={self.planner.racing_line.last_distances.left_m:.2f} '
                f'curb_right={self.planner.racing_line.last_distances.right_m:.2f} '
                f'edge={self.planner.edge_alert} '
                f'edge_m={self.planner.near_edge_m} '
                f'edge_vcap={self.planner.edge_speed_cap_mps} '
                f'line_offset={self.planner.target_lane_offset:+.2f} '
                f'join_length={self.planner.reference_path.last_capture_m:.2f} '
                f'join_error={self.planner.reference_path.last_initial_error_m:+.2f} '
                f'lap_check={learning.last_confirmation}')
            self.last_log=now



def main():
    rclpy.init()
    n=Racer()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.pub_speed.publish(Float32(data=0.))
        n.pub_steer.publish(Float32(data=0.))
        n.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
