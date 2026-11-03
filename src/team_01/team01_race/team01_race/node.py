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
from std_msgs.msg import Bool, Float32
from team01_race.perception import (decode_depth, decode_rgb, depth_geometry,
                                    laser_to_xy)
from team01_race.planning import TrafficGate, SafetyPlanner
from team01_race.road_surface import dark_asphalt_bounds, transverse_start_marking
from team01_race.signal_vision import SignalVision


DEFAULTS = {
    'straight_speed_mps': 8.0, 'corner_speed_factor': .30,
    'target_straight_speed_mps': 20.0, 'preview_curvature_threshold': .08,
    'corner_threshold_deg': 10., 'max_center_steer_deg': 29.8,
    'wheel_base_m': .34, 'max_lateral_accel_mps2': 5.0,
    'max_braking_mps2': 5.0, 'accel_mps2': 2.0,
    'reaction_delay_s': .3, 'emergency_margin_m': .55,
    'path_half_width_m': .18, 'overtake_offset_m': .48,
    'overtake_advantage_mps': .8, 'depth_timeout_s': .45,
    'lidar_timeout_s': .45, 'rgb_timeout_s': .7,
    'sensor_mount_height_m': .155, 'sensor_pitch_down_deg': 11.,
    'camera_hfov_deg': 92., 'min_road_points': 45,
    'front_max_range_m': 12., 'rear_defend_min_gap_m': 1.5,
    'traffic_light_min_red_blobs': 4, 'traffic_light_ready_frames': 2,
    'traffic_light_off_frames': 3,
    'unsafe_allow_without_front_lidar': False, 'unmapped_speed_mps': .75,
    'require_visual_road': True, 'road_rgb_timeout_s': .65,
    'permit_lidar_verified_fallback': True,
    'grid_straight_m': .65, 'grid_discovery_speed_mps': 1.25,
    'startup_right_hold_s': 3.0, 'startup_max_steer_deg': 3.0,
    'center_transition_s': 2.0,
    'disable_first_lap_racing_line': True, 'enable_overtake': False,
    'second_lap_straight_speed_mps': 10.0,
    'grid_min_ahead_m': 1.65, 'grid_min_width_m': .68,
    'road_confirm_frames': 6, 'corner_prelook_heading_deg': 7.5,
    'corner_end_heading_deg': 3.5, 'corner_entry_length_m': 1.2,
    'corner_apex_length_m': 2.9, 'corner_lateral_offset_m': .23,
    'max_line_offset_change_mps': .55,
    'lap_min_distance_m': 65., 'lap_min_time_s': 25.,
    'lap_min_abs_yaw_rad': 5.2, 'lap_max_abs_yaw_rad': 8.5,
    'learned_lap_speed_factor': 1.00,
    'edge_caution_extra_m': .12, 'edge_caution_min_speed_mps': .45,
    'edge_caution_extra_speed_mps': 1.15,
}


class Racer(Node):
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
        self.depth_obstacles=np.empty((0,2))
        self.depth_valid=False
        self.road_image=None
        self.road_at=0.
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
            self._lap_record_saved=False
            self._last_partial_record_distance=-10.
        self.permission=bool(msg.data)
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
            self.depth_valid=scene.valid
            self.slope=scene.slope_deg
            if (scene.valid and self.road_image is not None and
                    time.monotonic()-self.road_at<self.p['road_rgb_timeout_s']):
                asphalt=dark_asphalt_bounds(scene.ground_xy,scene.ground_uv,self.road_image)
                self.road_bounds=asphalt.bounds if asphalt.valid else None
                self.asphalt_coverage=asphalt.coverage
                self.start_line_seen=(transverse_start_marking(
                    scene.ground_xy,scene.ground_uv,self.road_image,asphalt.bounds)
                    if asphalt.valid else False)
            else:
                self.road_bounds=None
                self.asphalt_coverage=0.
                self.start_line_seen=False
        except Exception as exc:
            self.depth_valid=False
            self.road_bounds=None
            self.start_line_seen=False
            self.get_logger().warning(f'depth/road rejected: {exc}')
        self.depth_at=time.monotonic()

    def on_road_rgb(self,msg):
        try:
            self.road_image=decode_rgb(msg)
            self.road_at=time.monotonic()
        except Exception as exc:
            self.road_image=None
            self.road_at=0.
            self.get_logger().warning(f'Road RGB rejected: {exc}')

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

    def _save_rgb(self,rgb,stage):
        try:
            import cv2
            self.camera_debug_dir.mkdir(parents=True,exist_ok=True)
            out=self.camera_debug_dir/f'{stage}.png'
            cv2.imwrite(str(out),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            self._snapshots.add(stage)
        except Exception as exc:
            self.get_logger().warning(f'Camera diagnostic PNG save failed: {exc}')

    def _save_road(self,stage):
        # Frame capture is diagnostic only; it must never authorize movement.
        if self.road_image is None:return
        try:
            import cv2
            self.road_debug_dir.mkdir(parents=True,exist_ok=True)
            cv2.imwrite(str(self.road_debug_dir/f'{stage}.png'),
                        cv2.cvtColor(self.road_image,cv2.COLOR_RGB2BGR))
            self._road_debug_saved.add(stage)
        except Exception as exc:
            self.get_logger().warning(f'Road diagnostic PNG failed: {exc}')

    def on_odometry(self,msg):
        # Velocity only; NO world pose, no simulator track coordinates.
        try:
            value=float(msg.twist.twist.linear.x)
            if math.isfinite(value):
                self.actual_speed=abs(value)
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
        rgb_fresh=now-self.rgb_at<self.p['rgb_timeout_s']
        enabled=(self.permission and (self.start_mode=='signal' or
                 (self.gate.lights_out and rgb_fresh)))
        d_ok=self.depth_valid and now-self.depth_at<self.p['depth_timeout_s']
        l_ok=now-self.lidar_at<self.p['lidar_timeout_s']
        # Stale rear sensor forbids lane-change overtakes, not ordinary driving.
        rear_gap=self.rear_gap if now-self.rear_at<self.p['lidar_timeout_s'] else -1.0
        visual_road=(self.road_bounds if now-self.road_at<self.p['road_rgb_timeout_s']
                     else None)
        motion=self.planner.step(self.depth_obstacles,self.lidar_obstacles,
                                 rear_gap,dt,enabled,d_ok,l_ok,visual_road,
                                 actual_speed=(self.actual_speed if now-self.odom_at<.6 else None))
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
        if enabled and d_ok and learning.observe_line(self.start_line_seen,self.planner.road):
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
                f'passing=OFF '
                f'track_points={len(learning.first_lap_features)} '
                f'odom_v={self.actual_speed if now-self.odom_at<.6 else None} '
                f'planner={self.planner.planner_type} curve={self.planner.preview_curve:.3f} '
                f'curb_ahead={self.planner.racing_line.last_distances.ahead_m:.2f} '
                f'curb_confirm={self.planner.racing_line.confirmed_frames} '
                f'curb_left={self.planner.racing_line.last_distances.left_m:.2f} '
                f'curb_right={self.planner.racing_line.last_distances.right_m:.2f} '
                f'edge={self.planner.edge_alert} '
                f'edge_m={self.planner.near_edge_m} '
                f'edge_vcap={self.planner.edge_speed_cap_mps} '
                f'line_offset={self.planner.target_lane_offset:+.2f} '
                f'lap_check={learning.last_confirmation}')
            self.last_log=now

    def _write_learning_progress(self):
        try:
            learning=self.planner.lap_learner
            folder=Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state'))) / 'urrc_team01' / 'learning'
            folder.mkdir(parents=True,exist_ok=True)
            dest=folder/'active_lap1.json'
            tmp=folder/'active_lap1.json.tmp'
            tmp.write_text(json.dumps({
                'status':'IN_PROGRESS_NOT_A_COMPLETED_LAP',
                'source':'on-board RGB+Depth, LiDAR, odometry velocity and commanded steering',
                'preloaded_track_map':False,
                'distance_m_est':learning.distance_m,
                'features':learning.first_lap_features,
                'note':'Odometry velocity + steering integration drifts; independent official lap proof is required.',
            },ensure_ascii=False,indent=2),encoding='utf-8')
            tmp.replace(dest)
        except Exception as exc:
            self.get_logger().warning(f'Live track learning snapshot failed: {exc}')

    def _write_lap_record(self):
        if self._lap_record_saved:return
        try:
            learning=self.planner.lap_learner
            folder=Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state'))) / 'urrc_team01' / 'learning'
            folder.mkdir(parents=True,exist_ok=True)
            filename=folder/f'lap1_{int(time.time())}.json'
            filename.write_text(json.dumps({
                'status':learning.last_confirmation,
                'note':'Approximate commanded-distance trace, NOT verified Gazebo lap timing',
                'first_lap_distance_m_est':learning.first_lap_distance,
                'first_lap_brake_ticks':learning.first_lap_brakes,
                'records_distance_m_heading_deg_cmd_speed_mode':learning.first_lap_records,
                'observed_track_features':learning.first_lap_features,
                'preloaded_track_map':False,
            },ensure_ascii=False,indent=2),encoding='utf-8')
            self._lap_record_saved=True
            self.get_logger().info(f'Estimated first-lap record saved: {filename}')
        except Exception as exc:
            self.get_logger().warning(f'First-lap record saving failed: {exc}')


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
