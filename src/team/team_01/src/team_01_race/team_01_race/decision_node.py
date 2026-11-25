import math
import json
from dataclasses import fields
from .camera_bundle import decode_bundle,decode_lidar_observation
import time
import numpy as np
from std_msgs.msg import Bool, Float32, Int32, String
from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu
from vision_msgs.msg import Detection3DArray
from .ros_support import SENSOR_QOS, ConfigNode, path_array, path_message, stamp_seconds
from .navigation import DecisionEngine, Obstacle
from .ramp import RampDetector
from .corner import CornerConfig, CornerDetector, preview_geometry, SEVERITIES
from .corner_recovery import RecoveryConfig,CornerRecovery,wire_context,transformed_paths
from .navigation import valid_path,valid_boundary
from .speed_feedback import SpeedFeedback
from .obstacle_pass import ObstaclePassConfig
from .braking_preview import BrakingPreview,BrakingPreviewConfig
from .preview_speed import preview_speed
from .depth_geometry import DepthConfig
from .depth_corner import DepthCornerTrack,CornerTravel
from .speed_authority import SpeedIncreaseConfirmation,PrimaryLidarAdmission,ObservedPreviewHold


class DecisionNode(ConfigNode):
    def __init__(self):
        super().__init__('team_01_decision')
        primary=self.p('primary_steering_source','lidar')
        if primary not in ('lidar','auto'):raise ValueError('primary_steering_source must be lidar or auto')
        self.primary_lidar=primary=='lidar'
        self.validation_ceiling=float(self.p('validation_speed_ceiling_mps',12.))
        if not math.isfinite(self.validation_ceiling) or not 0<self.validation_ceiling<=12.:raise ValueError('invalid validation speed ceiling')
        g = self.geometry()
        self.engine = DecisionEngine(
            vehicle_width=g['vehicle_width_m'], wheelbase=g['wheelbase_m'],
            horizon=self.p('planner.planning_horizon_m', 5.0),
            cruise=self.p('planner.cruise_speed_mps', 7.0),
            temporal_alpha=self.p('planner.trajectory_temporal_alpha', .30),
            history_sec=self.p('planner.temporal_history_sec', .5),
            confirm_frames=self.p('safety.path_ready_confirm_frames', 3),
            lateral_margin=self.p('obstacle.vehicle_lateral_margin_m', .15),
            longitudinal_margin=self.p('obstacle.vehicle_longitudinal_margin_m', .30),
            max_lateral_accel=self.p('planner.max_lateral_accel_mps2', 3.0),
            min_confidence=self.p('planner.min_path_confidence', .40),
            emergency_ttc=self.p('safety.emergency_ttc_sec', .6),
            caution_ttc=self.p('safety.caution_ttc_sec', 1.5),
            max_decel=self.p('safety.assumed_max_decel_mps2', 2.5),
            camera_timeout=self.p('safety.camera_timeout_sec', .25),
            lidar_timeout=self.p('safety.sensor_timeout_sec', .25),
            max_sensor_skew=self.p('safety.max_sensor_skew_sec', .15),
            require_official=self.p('require_official_start', True),
            yellow_behavior=self.p('traffic.yellow_behavior', 'stop'),
            start_mode=self.p('traffic.start_mode', 'red_lights_out'), rear_overhang=g['rear_overhang_m'],
            front_obstacle_stop_distance=self.p('safety.front_obstacle_stop_distance_m',.5))
        pc=ObstaclePassConfig()
        for field in fields(pc):setattr(pc,field.name,self.p('obstacle_pass.'+field.name,getattr(pc,field.name)))
        self.engine.pass_planner.c=pc
        self.pass_debug_pub=self.create_publisher(String,'/team_01/debug/obstacle_pass',10)
        self.sync_debug_pub=self.create_publisher(String,'/team_01/debug/sensor_sync',1)
        self.last_sync_debug=-math.inf
        self.last_pass_log=-math.inf;self.last_pass_debug_key=None
        self.ramp = RampDetector(
            alpha=self.p('imu.pitch_alpha', .30),
            enter_deg=self.p('ramp.enter_deg', 5.0), exit_deg=self.p('ramp.exit_deg', 3.0),
            confirm_frames=self.p('ramp.confirm_frames', 3),
            exit_confirm_frames=self.p('ramp.exit_confirm_frames', 3),
            timeout=self.p('safety.imu_timeout_sec', .25), normal_speed=self.engine.cruise,
            ramp_speed=self.p('ramp.max_speed_mps', 5.0),
            stale_speed=self.p('safety.imu_stale_speed_mps', 5.0))
        corner_config = CornerConfig()
        for field in fields(corner_config):
            setattr(corner_config, field.name, self.p('corner_control.'+field.name, getattr(corner_config,field.name)))
        self.corner = CornerDetector(corner_config, normal_speed=self.engine.cruise)
        self.rgb_corner = CornerDetector(corner_config, normal_speed=self.engine.cruise)
        self.depth_enabled=self.p('depth_corner.enabled',True)
        dc=DepthConfig()
        for field in fields(dc):setattr(dc,field.name,self.p('depth_corner.'+field.name,getattr(dc,field.name)))
        self.depth_track=DepthCornerTrack(dc,corner_config);self.corner_travel=CornerTravel(dc.odom_timeout_sec)
        self.speed_increase=SpeedIncreaseConfirmation(self.engine.confirm_frames)
        self.primary_admission=PrimaryLidarAdmission(self.engine.confirm_frames)
        self.depth_debug_pub=self.create_publisher(String,'/team_01/debug/depth_corner',10)
        self.depth_last_log=-math.inf
        self.create_subscription(String,'/team_01/depth/corner_geometry',self.on_depth_geometry,1)
        recovery_config=RecoveryConfig()
        for field in fields(recovery_config):
            setattr(recovery_config,field.name,self.p('corner_path_recovery.'+field.name,getattr(recovery_config,field.name)))
        recovery_config.corner_path_hold_speed_mps=min(recovery_config.corner_path_hold_speed_mps,self.p('path_manager.previous_path_max_speed_mps',1.2))
        if self.primary_lidar:recovery_config.allow_general_path_hold=True
        self.preview_hold=ObservedPreviewHold(recovery_config.corner_path_hold_sec);self.current_preview_hold=None
        self.recovery=CornerRecovery(recovery_config,g['wheelbase_m'],g['vehicle_width_m'],self.engine.camera_timeout)
        preview_config=BrakingPreviewConfig()
        for field in fields(preview_config):setattr(preview_config,field.name,self.p('corner_braking_preview.'+field.name,getattr(preview_config,field.name)))
        self.braking_preview=BrakingPreview(preview_config,corner_config,self.engine.cruise)
        self.lidar_preview=BrakingPreview(preview_config,corner_config,self.engine.cruise)
        self.fused_preview=BrakingPreview(preview_config,corner_config,self.engine.cruise)
        self.active_preview=self.braking_preview
        self.create_subscription(String,'/team_01/lidar/braking_preview',self.on_lidar_preview,1)
        if self.primary_lidar or self.p('camera_geometry.bundle_topic','/team_01/camera/geometry_bundle')=='/team_01/path_manager/geometry_bundle':
            self.create_subscription(String,'/team_01/path_manager/traffic_light',self.on_visual_light,1)
        self.create_subscription(String,'/team_01/camera/braking_preview',self.on_braking_preview,1)
        if self.primary_lidar:
            self.create_subscription(String,'/team_01/camera/geometry_bundle',self.on_camera_preview_geometry,1)
        self.pp_feedback={};self.pp_feedback_time=-math.inf
        self.create_subscription(String,'/team_01/debug/pp',self.on_pp_feedback,1)
        self.final_internal_steering=0.
        self.command_speed=0.
        self.last_preview_log=-math.inf
        self.raw_preview_time=-math.inf;self.raw_preview_limit=None
        self.velocity=SpeedFeedback(self.p('safety.velocity_timeout_sec',.25))
        self.create_subscription(Odometry,'/team_01/odometry',self.on_velocity,SENSOR_QOS)
        self.source_status={};self.effective_source='NO_PATH';self.source_since=0.;self.last_source_speed=0.;self.last_source_target=0.
        self.source_totals={s:0. for s in ('CAMERA_PATH','LIDAR_PATH','FUSED_PATH','PREVIOUS_PATH','PREVIOUS_LIDAR_PATH','NO_PATH','STOP')}
        self.source_event_pub=self.create_publisher(String,'/team_01/debug/path_source_switch',10)
        self.create_subscription(String,'/team_01/internal/source_status',self.on_source_status,10)
        self.path_feedback_pub=self.create_publisher(String,'/team_01/internal/path_acceptance',10)
        self.recovery_debug_pub=self.create_publisher(String,'/team_01/debug/corner_recovery',10)
        self.last_recovery_log=-math.inf;self.last_recovery_key=None
        self.create_subscription(Float32,'/team_01/steering',lambda m:setattr(self,'final_internal_steering',-m.data if math.isfinite(m.data) else 0.),1)
        self.create_subscription(Bool,'/team_01/debug/lidar_scan_ready',lambda m:self.recovery.lidar_health(m.data,self.now_seconds()),1)
        self.create_subscription(Bool,'/team_01/debug/camera_ready',lambda m:self.recovery.health(m.data,self.now_seconds()),1)
        self.create_subscription(Float32,'/team_01/debug/camera_image_age',self.on_image_age,1)
        self.recovery_pub=self.create_publisher(String,'/team_01/internal/corner_path_context',10)
        self.command_pub=self.create_publisher(String,'/team_01/internal/decision_command',10)
        self.lidar_steering_pub=self.create_publisher(String,'/team_01/debug/lidar_steering_path',10)
        self.camera_preview_pub=self.create_publisher(String,'/team_01/debug/camera_preview',10)
        self.debug_pub=self.create_publisher(String,'/team_01/debug/decision_debug',10)
        self.last_debug_state=None;self.last_debug_time=-math.inf;self.input_info={}
        self.image_age_value=None;self.image_age_time=-math.inf
        self.steering_demand = 0.0
        self.steering_demand_time = -math.inf
        self.corner_braking_pub = self.create_publisher(Bool, '/team_01/internal/corner_braking', 10)
        self.corner_state_pub = self.create_publisher(String, '/team_01/debug/corner_state', 10)
        self.corner_limit_pub = self.create_publisher(Float32, '/team_01/debug/corner_speed_limit', 10)
        self.corner_policy_pub = self.create_publisher(String, '/team_01/internal/corner_policy', 10)
        self.corner_severity_pub = self.create_publisher(String, '/team_01/debug/corner_severity', 10)
        self.corner_status_pub = self.create_publisher(String, '/team_01/debug/corner_status', 10)
        self.last_corner_log = -math.inf
        self.last_corner_state = None
        self.create_subscription(Float32, '/team_01/debug/desired_steering', self.on_steering_demand, 1)
        self.imu_summary_pub = self.create_publisher(String, '/team_01/debug/imu_summary', 10)
        self.imu_status_pub = self.create_publisher(DiagnosticArray, '/team_01/debug/imu_status', 10)
        self.ramp_pub = self.create_publisher(Bool, '/team_01/debug/ramp_active', 10)
        self.ramp_state_pub = self.create_publisher(String, '/team_01/debug/ramp_state', 10)
        self.ramp_limit_pub = self.create_publisher(Float32, '/team_01/debug/ramp_speed_limit', 10)
        self.raw_pitch_pub = self.create_publisher(Float32, '/team_01/debug/imu_pitch_raw_deg', 10)
        self.filtered_pitch_pub = self.create_publisher(Float32, '/team_01/debug/imu_pitch_filtered_deg', 10)
        self.last_ramp_log = -math.inf
        self.create_subscription(Imu, self.p('imu_topic', '/team_01/imu'), self.on_imu, SENSOR_QOS)
        self.cache = {}
        self.last_header = Header(frame_id='team_01/base_link')
        self.last_camera_stamp = -math.inf
        self.previous_obstacles = {}
        self.last_mode = None
        self.last_log = -math.inf
        self.path_pub = self.create_publisher(Path, '/team_01/selected_path', 10)
        self.speed_pub = self.create_publisher(Float32, '/team_01/internal/requested_speed', 10)
        self.enabled_pub = self.create_publisher(Bool, '/team_01/internal/drive_enabled', 10)
        self.mode_pub = self.create_publisher(String, '/team_01/debug/mode', 10)
        self.state_pub = self.create_publisher(String, '/team_01/debug/state', 10)
        self.reason_pub = self.create_publisher(String, '/team_01/debug/stop_reason', 10)
        self.ready_count_pub = self.create_publisher(Int32, '/team_01/debug/path_ready_count', 10)
        self.ready_pub = self.create_publisher(Bool, '/team_01/debug/path_ready', 10)
        self.summary_pub = self.create_publisher(String, '/team_01/debug/decision_status', 10)
        self.diag_pub = self.create_publisher(String, '/team_01/debug/path_diagnostics', 10)
        if self.primary_lidar:
            self.last_sync_stamp=-math.inf
            self.create_subscription(String,'/team_01/lidar/synchronized_observation',self.on_synchronized_lidar,1)
        elif self.p('camera_geometry.prefer_bundle',False):
            self.create_subscription(String,self.p('camera_geometry.bundle_topic','/team_01/camera/geometry_bundle'),self.on_camera_bundle,1)
        else:
            for name in ('center_path', 'left_boundary', 'right_boundary'):
                self.create_subscription(Path, '/team_01/camera/'+name,
                                         lambda m, name=name: self.part(name, m), 1)
            self.create_subscription(DiagnosticArray, '/team_01/camera/path_status',
                                     lambda m: self.part('status', m), 1)
        if not self.primary_lidar:
            self.create_subscription(Detection3DArray, '/team_01/perception/obstacles', self.on_obstacles, 1)
        self.create_subscription(Bool, self.p('race_start_topic', '/race/start/team_01'), self.on_start, 10)
        self.create_subscription(Bool, '/team_01/internal/controller_ready', self.on_ready, 10)
        self.create_subscription(Bool, '/team_01/emergency_stop', self.on_emergency, 10)
        self.create_subscription(Float32, '/team_01/debug/final_speed', self.on_speed, 10)
        self.create_timer(.02, self.tick)
        self.get_logger().info(f"Steering primary={'LiDAR' if self.primary_lidar else 'legacy selector'}; Camera preview/start gate; vehicle envelope width={g['vehicle_width_m']:.4f} m; LiDAR obstacles only")

    def on_pp_feedback(self,msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):self.pp_feedback=data;self.pp_feedback_time=self.now_seconds()
        except (ValueError,TypeError):pass

    def on_visual_light(self,msg):
        if msg.data in ('RED','GREEN','YELLOW','UNKNOWN','LIGHTS_OUT'):
            self.engine.set_light(msg.data)
            if msg.data in ('RED','YELLOW'):self.tick()

    def on_source_status(self,msg):
        try:self.source_status=json.loads(msg.data)
        except (ValueError,TypeError):pass

    def trace_source(self,result,context,now):
        source=result.get('effective_path_source','NO_PATH')
        if source!=self.effective_source:
            old=self.effective_source
            self.source_totals[old]=self.source_totals.get(old,0.)+max(0.,now-self.source_since)
            event=dict(self.source_status,previous_source=old,new_source=source,
                stamp=now,current_source_duration=max(0.,now-self.source_since),
                speed_before_switch=self.last_source_speed,speed_after_switch=self.engine.actual_speed,
                target_speed_before=self.last_source_target,target_speed_after=result['speed'],
                previous_path_available=self.recovery.memory is not None,
                source_state='PREVIOUS_ACTIVE' if source in ('PREVIOUS_PATH','PREVIOUS_LIDAR_PATH') else self.source_status.get('state','SOURCE_RECOVERY'))
            def source_age(name):
                if name=='CAMERA_PATH':return self.source_status.get('camera_age')
                if name=='LIDAR_PATH':return self.source_status.get('lidar_age')
                if name=='FUSED_PATH':
                    ages=[self.source_status.get(k) for k in ('camera_age','lidar_age')]
                    return max(ages) if all(a is not None for a in ages) else None
                if name in ('PREVIOUS_PATH','PREVIOUS_LIDAR_PATH') and self.recovery.memory is not None:return max(0.,now-self.recovery.memory['time'])
                return None
            event.update(current_source_age=source_age(old),candidate_source_age=source_age(source),speed_measurement_source=result.get('current_speed_source'))
            if source in ('PREVIOUS_PATH','PREVIOUS_LIDAR_PATH'):
                issue=self.input_info.get('trajectory_issue','')
                if issue in ('trajectory_jump','invalid_target'):reason='trajectory_jump' if issue=='trajectory_jump' else 'no_forward_target'
                elif issue=='low_path_confidence':reason='confidence_drop'
                elif (self.source_status.get('camera_age') or 0.)>.25 or (self.source_status.get('lidar_age') or 0.)>.25:reason='sensor_stale'
                elif not self.source_status.get('camera_valid'):reason='camera_invalid'
                elif not self.source_status.get('lidar_valid'):reason='lidar_invalid'
                elif not self.source_status.get('geometry_equivalent'):reason='geometry_disagreement'
                else:reason='no_forward_target'
                event['switch_reason']=reason
            self.source_event_pub.publish(String(data=json.dumps(event,allow_nan=False)))
            self.get_logger().info('[PATH_SOURCE_SWITCH] '+json.dumps(event,allow_nan=False))
            self.effective_source=source;self.source_since=now
        self.last_source_speed=self.engine.actual_speed;self.last_source_target=result['speed']

    def on_lidar_preview(self,msg):
        try:self.lidar_preview.update(json.loads(msg.data),self.get_clock().now().nanoseconds*1e-9,self.now_seconds())
        except (ValueError,TypeError):pass

    def on_camera_preview_geometry(self,msg):
        try:
            d=json.loads(msg.data);now=self.now_seconds();ros_now=self.get_clock().now().nanoseconds*1e-9
            parts=decode_bundle(msg.data,ros_now,self.engine.camera_timeout)
            stamp=stamp_seconds(parts['center_path'].header.stamp)
            (self.rgb_corner if self.depth_enabled else self.corner).update(parts['center_path'].points,stamp,now,speed=self.engine.actual_speed,
                               confidence=float(d['confidence']),path_age=max(0.,ros_now-stamp))
        except (ValueError,TypeError,KeyError):return

    def lidar_straight(self):
        if not valid_path(self.engine.path) or not self.velocity.fresh(self.now_seconds()):return False
        m=preview_geometry(self.engine.path,self.corner.c)
        return m['robust_future_curvature']<self.corner.c.mild_curvature_exit and m['future_heading_change_deg']<self.corner.c.mild_heading_exit_deg

    def on_depth_geometry(self,msg):
        try:
            d=json.loads(msg.data)
            now=self.now_seconds();ros_now=self.get_clock().now().nanoseconds*1e-9
            updated=self.depth_track.update(d,ros_now,now,
                self.corner_travel.distance,self.corner_travel.fresh(now),self.lidar_straight())
            if updated and d.get('depth_corner_candidate'):self.preview_hold.clear()
            elif (updated and d.get('straight_geometry') and self.depth_track.straight and
                  self.corner_travel.fresh(now)):
                stamp=float(d['stamp']);travel=self.corner_travel.distance_at(stamp)
                self.preview_hold.record(stamp,now-max(0.,ros_now-stamp),travel,self.corner_travel.epoch,
                    float(d.get('depth_curvature_supported_distance_m',0.)))
        except (ValueError,TypeError,KeyError):return

    def on_braking_preview(self,msg):
        try:self.braking_preview.update(json.loads(msg.data),self.get_clock().now().nanoseconds*1e-9,self.now_seconds())
        except (ValueError,TypeError):pass

    def on_camera_bundle(self,msg):
        try:
            if self.primary_lidar and json.loads(msg.data).get('path_source') not in ('LIDAR_PATH','NO_PATH'):return
            parts=decode_bundle(msg.data,self.get_clock().now().nanoseconds*1e-9,self.engine.camera_timeout)
            for name,part in parts.items():self.part(name,part)
        except (ValueError,TypeError,KeyError,IndexError,OverflowError):return

    def on_synchronized_lidar(self,msg):
        try:
            parts,detections,valid=decode_lidar_observation(msg.data,self.get_clock().now().nanoseconds*1e-9,
                min(self.engine.camera_timeout,self.engine.lidar_timeout))
            stamp=stamp_seconds(parts['center_path'].header.stamp)
            if stamp<=self.last_sync_stamp:return
            # Both updates execute in one callback; the timer cannot observe a
            # newly captured path paired with the preceding scan's obstacles.
            self.on_camera_bundle(msg)
            if valid:
                now=self.now_seconds();age=self.age(parts['center_path'].header.stamp)
                self.engine.lidar([Obstacle(o['id'],o['x'],o['y'],o['width'],o['depth'],
                    o['confidence'],o['closing_speed']) for o in detections],now-age,stamp=stamp)
            self.last_sync_stamp=stamp
            now=self.now_seconds()
            if now-self.last_sync_debug>=.2:
                self.sync_debug_pub.publish(String(data=json.dumps(dict(source_stamp=stamp,
                    path_stamp=self.engine.camera_stamp,obstacle_stamp=self.engine.lidar_stamp,
                    observation_valid=valid,receive_age=self.age(parts['center_path'].header.stamp),
                    obstacle_count=len(detections)))))
                self.last_sync_debug=now
        except (ValueError,TypeError,KeyError,IndexError,OverflowError):return

    def on_image_age(self,msg):
        if math.isfinite(msg.data) and msg.data>=0:
            from .camera_health import image_age_with_geometry
            now=self.now_seconds()
            age=image_age_with_geometry(msg.data,now,self.engine.camera_time,self.engine.path_source)
            self.image_age_value=age;self.image_age_time=now
            if age>self.engine.camera_timeout:self.recovery.health(False,now)

    def on_steering_demand(self, msg):
        if math.isfinite(msg.data):
            self.steering_demand = msg.data
            self.steering_demand_time = self.now_seconds()

    def publish_corner(self, result, now):
        policy={key:result[key] for key in ('corner_severity','corner_speed_limit','corner_steering_limit_deg',
            'camera_source_stamp','camera_geometry_confidence','camera_geometry_age','robust_future_curvature',
            'max_future_curvature','future_heading_change_deg','corner_preview_speed_limit','distance_to_corner','required_braking_distance',
            'braking_margin','braking_feasible','entry_overspeed','preview_braking_insufficient','ramp_speed_limit','active_limiter')}
        policy.update({k:result.get(k) for k in ('supported_preview','preview_speed_limit','preview_stable_straight','preview_source')})
        policy.update(stamp=now,current_source_duration=max(0.,now-self.source_since),
            continuous_high_speed_target_duration=result.get('continuous_high_speed_target_duration',0.),
            continuous_straight_duration=result.get('continuous_straight_duration',0.),
            source_durations={k:self.source_totals[k]+(max(0.,now-self.source_since) if k==self.effective_source else 0.) for k in self.source_totals})
        policy.update(frame_id='team_01/base_link',final_target_speed=result['speed'],obstacle_limit=result['obstacle_limit'],existing_path_ttc_confidence_limit=result['existing_path_ttc_confidence_limit'])
        for key in ('raw_path_curvature','supported_path_curvature','lateral_acceleration_speed_limit',
                    'visible_path_speed_limit','confidence_speed_factor','source_speed_limit','base_speed_limiter'):
            policy[key]=result.get(key)
        policy={k:(None if isinstance(v,float) and not math.isfinite(v) else v) for k,v in policy.items()}
        policy.update({k:v for k,v in result.items() if k.startswith('depth_') or k=='corner_latched'})
        self.corner_policy_pub.publish(String(data=json.dumps(policy,allow_nan=False)))
        self.corner_severity_pub.publish(String(data=result['corner_severity']))
        self.corner_braking_pub.publish(Bool(data=result['corner_braking']))
        self.corner_state_pub.publish(String(data=result['corner_state']))
        self.corner_limit_pub.publish(Float32(data=float(result['corner_speed_limit'])))
        if (result['corner_state'],result['corner_severity']) != self.last_corner_state or now-self.last_corner_log >= 1.0:
            message = (f"state={result['corner_state']} corner_severity={result['corner_severity']} corner_steering_limit_deg={result['corner_steering_limit_deg']} current_speed={result['current_speed']:.3f} "
                       f"speed_source={result['current_speed_source']} distance_to_corner={result['distance_to_corner']:.3f} "
                       f"current_curvature={result['current_curvature']:.4f} max_future_curvature={result['max_future_curvature']:.4f} "
                       f"robust_future_curvature={result['robust_future_curvature']:.4f} future_heading_change_deg={result['future_heading_change_deg']:.2f} preview_braking_insufficient={result['preview_braking_insufficient']} future_heading_change={result['future_heading_change']:.4f} lateral_path_change={result['lateral_path_change']:.3f} "
                       f"required_braking_distance={result['required_braking_distance']:.3f} braking_margin={result['braking_margin']:.3f} "
                       f"braking_feasible={result['braking_feasible']} entry_overspeed={result['entry_overspeed']} "
                       f"enter={result['corner_enter_count']}/{self.corner.c.corner_enter_confirm_frames} "
                       f"exit={result['corner_exit_count']}/{self.corner.c.corner_exit_confirm_frames} "
                       f"corner_speed_limit={result['corner_speed_limit']:.1f} approach_braking={result['corner_braking']} active_speed_limiter={result['active_limiter']} "
                       f"final_target_speed={result['speed']:.3f}")
            self.corner_status_pub.publish(String(data=message))
            self.get_logger().info('[CORNER_SEVERITY] '+message)
            self.last_corner_log, self.last_corner_state = now, (result['corner_state'],result['corner_severity'])

    def on_imu(self, msg):
        now = self.now_seconds()
        stamp = stamp_seconds(msg.header.stamp)
        ros_now = self.get_clock().now().nanoseconds*1e-9
        if stamp > ros_now+.05:
            self.ramp.invalidate('imu_timestamp_in_future')
            return
        q = msg.orientation
        self.ramp.update((q.x,q.y,q.z,q.w), stamp, now, age=self.age(msg.header.stamp),
                         orientation_available=(msg.header.frame_id == 'team_01/imu_link' and
                                                msg.orientation_covariance[0] != -1.0))

    def publish_ramp(self, result, now):
        state = result['terrain_state']
        error = self.ramp.error or ('imu_timeout' if not result['imu_fresh'] else '')
        self.ramp_pub.publish(Bool(data=self.ramp.active))
        self.ramp_state_pub.publish(String(data=state))
        self.ramp_limit_pub.publish(Float32(data=float(result['ramp_speed_limit'])))
        if self.ramp.raw is not None:
            self.raw_pitch_pub.publish(Float32(data=float(self.ramp.raw)))
            self.filtered_pitch_pub.publish(Float32(data=float(self.ramp.filtered)))
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'team_01/imu_link'
        status = DiagnosticStatus(name='imu_ramp', hardware_id='team_01_imu',
                                  level=DiagnosticStatus.OK if result['imu_fresh'] else DiagnosticStatus.WARN,
                                  message=state)
        values = dict(pitch_raw_deg=self.ramp.raw, pitch_filtered_deg=self.ramp.filtered,
                      ramp_active=self.ramp.active, enter_count=self.ramp.enter_count,
                      enter_confirm_frames=self.ramp.confirm_frames, exit_count=self.ramp.exit_count,
                      exit_confirm_frames=self.ramp.exit_confirm_frames, ramp_speed_limit=result['ramp_speed_limit'],
                      imu_age_sec=now-self.ramp.last_valid_time, error=error,
                      active_limiter=result['active_limiter'], speed_before_ramp=result['speed_before_ramp'],
                      speed_target=result['speed'])
        status.values = [KeyValue(key=k, value=str(v)) for k,v in values.items()]
        message.status = [status]
        self.imu_status_pub.publish(message)
        if now-self.last_ramp_log >= 1.0:
            self.get_logger().info(f'[IMU] pitch_raw_deg={self.ramp.raw} pitch_filtered_deg={self.ramp.filtered} '
                                  f'enter={self.ramp.enter_count}/{self.ramp.confirm_frames} '
                                  f'exit={self.ramp.exit_count}/{self.ramp.exit_confirm_frames} STATE={state}')
            self.get_logger().info(f"[SPEED] cruise_limit={self.engine.cruise:.1f} ramp_limit={result['ramp_speed_limit']:.1f} "
                                  f"existing_limit={result['speed_before_ramp']:.3f} active_limiter={result['active_limiter']} "
                                  f"final_target={result['speed']:.3f}")
            self.imu_summary_pub.publish(String(data=f'pitch_raw_deg={self.ramp.raw} pitch_filtered_deg={self.ramp.filtered} '
                f'STATE={state} RAMP={self.ramp.active} enter={self.ramp.enter_count}/{self.ramp.confirm_frames} '
                f'exit={self.ramp.exit_count}/{self.ramp.exit_confirm_frames} '
                f"ramp_limit={result['ramp_speed_limit']:.1f} active_limiter={result['active_limiter']} "
                f"target_speed={result['speed']:.3f} error={error}"))
            self.last_ramp_log = now

    def on_start(self, msg):
        if self.engine.official == msg.data:
            return
        self.get_logger().info(f'[START_GATE] official={msg.data}')
        self.engine.set_official(msg.data)
        self.tick()

    def on_ready(self, msg):
        if self.engine.controller_ready != msg.data:
            self.engine.controller_ready = msg.data
            self.tick()

    def on_emergency(self, msg):
        self.engine.external_emergency = msg.data
        self.tick()

    def on_speed(self, msg):
        self.command_speed = max(0.0, msg.data) if math.isfinite(msg.data) else 0.0
        self.engine.actual_speed=self.velocity.speed if self.velocity.fresh(self.now_seconds()) else self.command_speed

    def on_velocity(self,msg):
        v=msg.twist.twist.linear
        if self.depth_enabled:
            p=msg.pose.pose.position
            accepted=self.corner_travel.update(stamp_seconds(msg.header.stamp),self.now_seconds(),self.age(msg.header.stamp),
                (p.x,p.y,p.z),math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z))
            if not accepted and not self.corner_travel.accepted:
                # Odometry discontinuity invalidates relative corner tracking,
                # not independent fresh Depth straight geometry.
                self.depth_track.latched=None
                self.preview_hold.clear()
        if self.velocity.update(stamp_seconds(msg.header.stamp),self.now_seconds(),self.age(msg.header.stamp),(v.x,v.y,v.z),msg.twist.twist.angular.z):
            self.engine.actual_speed=self.velocity.speed

    def age(self, stamp):
        stamp = stamp_seconds(stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        return max(0.0, now-stamp) if stamp > 0 and now > 0 else 0.0

    def part(self, name, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if stamp < self.last_camera_stamp:
            return
        packet = self.cache.setdefault(stamp, {})
        packet[name] = msg
        if len(self.cache) > 5:
            del self.cache[min(self.cache)]
        required = ('center_path', 'left_boundary', 'right_boundary', 'status')
        if not all(key in packet for key in required) or stamp == self.last_camera_stamp:
            return
        status = packet['status']
        if not status.status or self.age(msg.header.stamp) > self.engine.camera_timeout:
            return
        values = {v.key: v.value for v in status.status[0].values}
        try:
            confidence = float(values.get('confidence', '0'))
            observed = (bool(int(values.get('left_observed', '0'))), bool(int(values.get('right_observed', '0'))))
        except ValueError:
            return
        if self.primary_lidar and values.get('path_source') not in ('LIDAR_PATH','NO_PATH'):return
        try:self.recovery.note_width(float(values['track_width']),float(values['track_width_age']),self.now_seconds())
        except (ValueError,KeyError,TypeError):pass
        previous_light = self.engine.light
        self.engine.set_light(values.get('traffic_light', 'UNKNOWN'))
        center = packet['center_path']
        if any(packet[key].header.frame_id != 'team_01/base_link' for key in required[:3]):
            return
        now = self.now_seconds()-self.age(center.header.stamp)
        self.align_history_to_capture(now)
        raw_path=path_array(center)
        self.raw_preview_limit=None
        if (self.corner.c.enabled and confidence>=self.corner.c.exit_min_confidence and valid_path(raw_path) and
            all(valid_boundary(path_array(packet[k])) for k in ('left_boundary','right_boundary'))):
            preview=preview_geometry(raw_path,self.corner.c)
            # Speed-only early warning: median-supported maximum curvature plus
            # visible heading. It grants no extra steering authority.
            if (preview['max_future_curvature']>=self.corner.c.sharp_curvature_enter and
                preview['confirmation_heading_deg']>=self.corner.c.mild_heading_enter_deg):
                self.raw_preview_limit=self.corner.c.sharp_max_speed_mps;self.raw_preview_time=now
        self.engine.camera(raw_path, path_array(packet['left_boundary']),
                           path_array(packet['right_boundary']), confidence, observed,
                           stamp, now, stamp=stamp)
        accepted=self.engine.camera_stamp==stamp
        if accepted:self.engine.path_source=values.get('path_source','CAMERA_PATH')
        if self.primary_lidar and valid_path(raw_path):
            from .navigation import path_geometry,supported_curvature
            heading,curvature=path_geometry(raw_path)
            k=float(np.median(supported_curvature(raw_path)));h=float(np.median(heading[:min(8,len(heading))]))
            difference=self.engine.last_geometry_delta or {}
            lateral=difference.get('lateral_jump');heading_jump=difference.get('heading_jump');curvature_jump=difference.get('curvature_jump')
            self.lidar_metrics=dict(lidar_center_y_near=float(np.interp(1.,raw_path[:,0],raw_path[:,1])),
                path_heading=h,path_curvature=k,lateral_jump=lateral,heading_jump=heading_jump,curvature_jump=curvature_jump,overlap_points=difference.get('overlap_points',0),
                track_width=self.recovery.track_width,observed_path_length=float(np.linalg.norm(np.diff(raw_path,axis=0),axis=1).sum()),
                observed_forward_start_m=float(raw_path[0,0]),observed_forward_end_m=float(raw_path[-1,0]),
                lidar_path_lateral_jump=lateral,lidar_path_heading_jump=heading_jump,lidar_path_curvature_jump=curvature_jump,lidar_path_confidence=confidence,
                left_wall_valid=observed[0],right_wall_valid=observed[1],single_wall_fallback=not all(observed),path_confidence=confidence)
        if self.primary_lidar:
            m=getattr(self,'lidar_metrics',{})
            stable=(accepted and all(observed) and confidence>=.85 and m.get('overlap_points',0)>=3 and
                all(m.get(k) is not None for k in ('lateral_jump','heading_jump','curvature_jump')) and
                m.get('lateral_jump',math.inf)<=self.corner.c.exit_path_stability_m and
                abs(m.get('heading_jump',math.inf))<=math.radians(self.corner.c.mild_heading_exit_deg) and
                abs(m.get('curvature_jump',math.inf))<=self.corner.c.mild_curvature_enter)
            self.primary_stable_frames=getattr(self,'primary_stable_frames',0)+1 if stable else 0
            self.primary_admission.update(stamp,accepted and all(observed) and confidence>=.85 and
                self.engine.path_confirmed and valid_path(raw_path),stable)
        self.path_feedback_pub.publish(String(data=json.dumps(dict(source=values.get('path_source','CAMERA_PATH'),stamp=stamp,accepted=accepted,reason=self.engine.last_camera_issue))))
        issue=self.engine.last_camera_issue
        if accepted and confidence<self.recovery.c.memory_min_confidence:issue='low_path_confidence'
        forward_bad=isinstance(raw_path,np.ndarray) and raw_path.ndim==2 and len(raw_path)>0 and bool(np.any(raw_path[:,0]<=0))
        from .corridor_geometry import normal_hit
        left_near=normal_hit(np.array([.2,0.]),np.array([0.,1.]),self.engine.left,1) if accepted else None
        right_near=normal_hit(np.array([.2,0.]),np.array([0.,1.]),self.engine.right,-1) if accepted else None
        corridor_ok=left_near is None or right_near is None or min(left_near,-right_near)>=self.engine.width/2
        self.recovery.note_input(stamp,self.now_seconds(),accepted,confidence,observed,issue,forward_bad,corridor_ok)
        self.input_info=dict(geometry_source_stamp=stamp,path_source=values.get('path_source','CAMERA_PATH'),path_valid=bool(valid_path(raw_path)),path_confidence=confidence,left_wall_valid=observed[0],right_wall_valid=observed[1],
                             perception_reason=values.get('path_reason',values.get('reason','')),trajectory_issue=issue,single_wall_fallback=bool(accepted and any(observed) and not all(observed)))
        if accepted:
            self.geometry_projection_time=now
            self.last_header=center.header
            if not self.primary_lidar:self.corner.update(raw_path, stamp, self.now_seconds(), speed=self.engine.actual_speed,
                               steering_demand=self.steering_demand,
                               steering_fresh=self.now_seconds()-self.steering_demand_time <= .25,
                               confidence=self.engine.confidence, path_age=self.age(center.header.stamp))
        elif not self.primary_lidar:
            self.corner.reject(stamp,transient=issue=='trajectory_jump')
        self.last_camera_stamp = stamp
        if self.engine.light != previous_light and self.engine.light in ('RED','YELLOW'):
            self.tick()  # Confirmed visual stop is immediate; normal updates use the 50 Hz loop.

    def align_history_to_capture(self,now):
        # Compare trajectories in the same body frame before the existing
        # temporal blend. Odometry twist supplies ego motion; pose is unused.
        # Selected execution history stays in its independent current-time
        # frame; only sensor smoothing history is aligned back to capture.
        dt=now-(self.engine.geometry_motion_time if self.engine.geometry_motion_time is not None else self.engine.camera_time)
        self.history_alignment_dt=dt
        if self.velocity.fresh(self.now_seconds()) and self.engine.path is not None and 0<abs(dt)<=self.engine.history_sec:
            vx,vy,_=self.velocity.linear;w=self.velocity.yaw_rate;angle=w*dt
            dx=vx*dt if abs(w)<1e-6 else (vx*math.sin(angle)+vy*(math.cos(angle)-1))/w
            dy=vy*dt if abs(w)<1e-6 else (vx*(1-math.cos(angle))+vy*math.sin(angle))/w
            paths=dict(center=self.engine.path,left=self.engine.left,right=self.engine.right,selected=self.engine.path)
            try:
                predicted=transformed_paths(paths,(dx,dy,angle))
                self.engine.path=predicted['center'];self.engine.left=predicted['left'];self.engine.right=predicted['right']
                self.engine.geometry_motion_time=now
                self.geometry_projection_time=now
            except ValueError:pass
        elif self.recovery.last_context is not None:
            try:
                predicted=self.recovery.prediction()
                self.engine.path=predicted['center'];self.engine.left=predicted['left'];self.engine.right=predicted['right'];self.engine.last_selected=predicted['selected']
            except ValueError:pass

    def on_obstacles(self, msg):
        if msg.header.frame_id != 'team_01/base_link' or self.age(msg.header.stamp) > self.engine.lidar_timeout:
            return
        now, stamp = self.now_seconds(), stamp_seconds(msg.header.stamp)
        obstacles = []
        current = {}
        for detection in msg.detections:
            p, size = detection.bbox.center.position, detection.bbox.size
            confidence = max((r.hypothesis.score for r in detection.results), default=0.0)
            if not all(math.isfinite(v) for v in (p.x, p.y, size.x, size.y, confidence)) or min(size.x, size.y) <= 0:
                continue
            previous = self.previous_obstacles.get(detection.id)
            closing = max(0.0, (previous[1]-p.x)/(stamp-previous[0])) if previous and stamp > previous[0] else 0.0
            if previous:
                closing = .5*closing+.5*previous[2]
            obstacles.append(Obstacle(detection.id, p.x, p.y, size.y, size.x, confidence, closing))
            current[detection.id] = (stamp, p.x, closing)
        self.previous_obstacles = current
        self.engine.lidar(obstacles, now-self.age(msg.header.stamp), stamp=stamp)

    def tick(self):
        now = self.now_seconds()
        ros_now=self.get_clock().now().nanoseconds*1e-9
        from .preview_fusion import choose_preview,choose_visibility_preview
        self.active_preview=self.braking_preview if self.primary_lidar else choose_preview(self.braking_preview,self.lidar_preview,self.fused_preview,now,ros_now)
        self.visibility_preview=self.braking_preview if self.primary_lidar else choose_visibility_preview(self.braking_preview,self.lidar_preview,now)
        # Extend visibility authority only with fresh, agreeing measured Camera
        # geometry. Steering still uses the original short trajectory.
        self.engine.supported_preview_distance=0.
        preview=self.visibility_preview.speed_context(now)
        far_path=self.visibility_preview.speed_path
        near_path=self.engine.path
        if (not self.depth_enabled and preview is not None and self.visibility_preview.speed_data['confidence']>=.85 and
                (self.engine.path_source=='CAMERA_PATH' or self.primary_lidar and self.engine.path_source=='LIDAR_PATH') and valid_path(near_path) and
                far_path is not None):
            overlap=near_path[:,0]>=far_path[0,0]
            if overlap.sum()>=3 and near_path[-1,0]<=far_path[-1,0] and np.max(np.abs(
                    near_path[overlap,1]-np.interp(near_path[overlap,0],far_path[:,0],far_path[:,1])))<=.15:
                self.engine.supported_preview_distance=preview['supported_preview']
        self.current_preview_hold=None
        if self.depth_enabled and self.depth_track.fresh(now):
            self.engine.supported_preview_distance=self.depth_track.data.get('depth_supported_distance_m',0.)
        elif self.depth_enabled and self.primary_lidar:
            current_lidar=(self.primary_admission.confirmed and all(self.engine.observed) and
                self.engine.confidence>=.85 and 0<=now-self.engine.camera_time<=self.engine.camera_timeout and
                valid_path(self.engine.path) and self.lidar_straight() and self.depth_track.latched is None)
            self.current_preview_hold=self.preview_hold.evaluate(now,self.corner_travel.distance,self.corner_travel.epoch,
                odom_fresh=self.corner_travel.fresh(now),lidar_straight=current_lidar,command=self.command_speed,
                decel=self.engine.max_decel,margin=self.corner.c.corner_brake_margin_m+self.engine.actual_speed*self.corner.c.reaction_time_sec)
            if self.current_preview_hold is not None:self.engine.supported_preview_distance=self.current_preview_hold['remaining_m']
        if self.depth_enabled and valid_path(self.engine.path):
            self.corner.update(self.engine.path,self.engine.camera_stamp,now,speed=self.engine.actual_speed,
                confidence=self.engine.confidence,path_age=max(0.,now-self.engine.camera_time))
        healthy_primary=(self.primary_lidar and self.engine.path_source=='LIDAR_PATH' and
            all(self.engine.observed) and self.engine.confidence>=.85 and
            0<=now-self.engine.camera_time<=self.engine.camera_timeout and
            valid_path(self.engine.path) and all(valid_boundary(p) for p in (self.engine.left,self.engine.right)) and
            self.engine.path_confirmed and
            self.primary_admission.confirmed)
        self.engine.full_speed_confidence=.85 if healthy_primary else 1.
        self.engine.actual_speed=self.velocity.speed if self.velocity.fresh(now) else self.command_speed
        if not self.engine.visual_go or self.engine.external_emergency or (self.engine.require_official and not self.engine.official):self.recovery.reset()
        twist=(*self.velocity.linear[:2],self.velocity.yaw_rate) if self.velocity.fresh(now) else None
        self.engine.update_motion(now,twist)
        self.engine.pass_planner.steering_limit_deg=self.corner.c.steer_limits()[self.corner.level]
        context=self.recovery.prepare(now,ros_now,self.engine.actual_speed,self.final_internal_steering,twist,independent_geometry=self.input_info.get('path_source') in ('LIDAR_PATH','FUSED_PATH'))
        result=self.engine.recovery_step(now,context) if context is not None else self.engine.step(now)
        if self.primary_lidar and context is None and now-self.engine.camera_time>self.engine.camera_timeout and result['reason'] not in ('lidar_timeout','FRONT_OBSTACLE_0P5M','critical_ttc','external_emergency'):
            result.update(path=None,speed=0.,enabled=False,mode='WAIT_PATH',reason='lidar_path_lost')
        if result['reason']=='invalid_target':self.input_info['trajectory_issue']='trajectory_selection_invalid'
        if context is None and result['reason']=='invalid_target' and self.recovery.committed:
            if self.recovery.first_failure is None:
                self.recovery.first_failure=dict(stamp=self.engine.camera_stamp,issue='trajectory_selection_invalid',left_wall_valid=self.engine.observed[0],right_wall_valid=self.engine.observed[1],confidence=self.engine.confidence)
            context=self.recovery.prepare(now,ros_now,self.engine.actual_speed,self.final_internal_steering,twist,force_hold=True)
            if context is not None:
                context['issue']='trajectory_selection_invalid'
                result=self.engine.recovery_step(now,context)
        if context is None and self.recovery.stop_reason and result['reason'] not in ('FRONT_OBSTACLE_0P5M','external_emergency','critical_ttc','confirmed_collision_risk','confirmed_visual_red','waiting_for_official_start'):
            result.update(path=None,speed=0.,enabled=False,mode='RECOVERY_STOP',reason=self.recovery.stop_reason)
        if self.recovery.expired:
            self.engine.path_confirmed=False if self.recovery.reacquire_frames<self.recovery.c.reacquire_confirm_frames else self.engine.path_confirmed
        result['source_speed_limit']=self.recovery.c.corner_path_hold_speed_mps if context is not None else self.engine.cruise
        result['lidar_path_mode']=('PREVIOUS_LIDAR_PATH' if context is not None else
            'INVALID_PATH' if not result['enabled'] else
            'NOT_LIDAR' if self.engine.path_source!='LIDAR_PATH' else
            'SINGLE_WALL_FALLBACK' if not all(self.engine.observed) else
            'NORMAL' if healthy_primary else 'LOW_CONFIDENCE_LIDAR')
        if result['enabled'] and (context is not None or self.input_info.get('path_source') in ('LIDAR_PATH','FUSED_PATH')):
            if context is None:
                result['source_speed_limit']=self.engine.cruise if healthy_primary else self.p('path_manager.lidar_only_max_speed_mps',2.) if self.input_info.get('path_source')=='LIDAR_PATH' else self.p('path_manager.fused_max_speed_mps',2.5)
            if result['source_speed_limit']<result['speed']:result['base_speed_limiter']='PATH_SOURCE'
            result['speed']=min(result['speed'],result['source_speed_limit'])
        result['existing_path_ttc_confidence_limit']=result['speed']
        passing=result['mode'] in ('LEFT_PASS','RIGHT_PASS','MERGE_CENTER')
        if passing and context is None:result['lidar_path_mode']='OBSTACLE_PASS'
        result['obstacle_limit']=self.engine.pass_planner.c.obstacle_pass_speed_mps if passing else self.engine.cruise
        if not result['enabled']:result['obstacle_limit']=0. if result['obstacle_count'] else self.engine.cruise
        result = self.ramp.constrain(result, now)
        result['pre_corner_speed'] = result['speed']
        if self.depth_enabled:
            result=self.constrain_depth(result,now)
        else:
            result = self.corner.constrain(result, now, self.engine.actual_speed)
        result['current_speed_source']='odometry_twist' if self.velocity.fresh(now) else 'control_command_estimate'
        if not self.depth_enabled and result['enabled'] and self.raw_preview_limit is not None and now-self.raw_preview_time<=self.engine.camera_timeout:
            if self.raw_preview_limit<result['speed']:result['active_limiter']='CORNER_PREVIEW'
            result['speed']=min(result['speed'],self.raw_preview_limit)
            result['corner_preview_speed_limit']=min(result['corner_preview_speed_limit'],self.raw_preview_limit)
            result['corner_braking']=max(self.command_speed,self.engine.actual_speed)>result['speed']+.01
        speed_preview=self.visibility_preview.speed_context(now,clear=(
            result['obstacle_count']==0 and result['mode']=='CENTER' and
            self.corner.level==0 and result.get('ttc',math.inf)>3.))
        preview_source=(self.visibility_preview.geometry or {}).get('preview_source','CAMERA_PREVIEW')
        if speed_preview is None:
            speed_preview=preview_speed(self.engine.path,self.engine.confidence,
                now-self.engine.camera_time,clear=False,max_speed=self.engine.cruise,
                decel=self.engine.max_decel,margin=self.engine.long_margin)
            preview_source=self.input_info.get('path_source','NO_PATH')
        if self.depth_enabled:
            speed_preview=dict(supported_preview=result.get('depth_preview_hold_remaining_m') or result['depth_supported_distance_m'],preview_speed_limit=result['corner_preview_braking_limit'],preview_stable_straight=self.depth_track.straight and result['depth_valid'])
            preview_source='DEPTH' if result['depth_valid'] else 'LIDAR_DEPTH_FALLBACK'
        result.update(speed_preview,preview_source=preview_source)
        if result['enabled'] and speed_preview['preview_speed_limit']<result['speed']:
            result['active_limiter']='SUPPORTED_PREVIEW'
            result['speed']=speed_preview['preview_speed_limit']
        far_preview=self.active_preview.context(now)
        if not self.depth_enabled and result['enabled'] and far_preview is not None:
            if far_preview['speed_limit']<result['speed']:result['active_limiter']='CAMERA_BRAKING_PREVIEW'
            result['speed']=min(result['speed'],far_preview['speed_limit'])
            result['corner_preview_speed_limit']=min(result['corner_preview_speed_limit'],far_preview['speed_limit'])
            if far_preview.get('distance_to_corner') is not None:
                result['distance_to_corner']=far_preview['distance_to_corner']
                result['required_braking_distance']=max(0.,self.engine.actual_speed**2-far_preview['speed_limit']**2)/(2*self.corner.c.corner_approach_decel_mps2)
                result['braking_margin']=self.corner.c.corner_brake_margin_m+self.engine.actual_speed*self.corner.c.reaction_time_sec
                result['braking_feasible']=result['required_braking_distance']+result['braking_margin']<=result['distance_to_corner']
            if self.corner.level==0:result['corner_state']='CORNER_APPROACH'
            result['corner_braking']=max(self.command_speed,self.engine.actual_speed)>result['speed']+.01
        if result['enabled'] and self.input_info.get('path_source') in ('LIDAR_PATH','FUSED_PATH'):
            result['corner_braking']=max(self.command_speed,self.engine.actual_speed)>result['speed']+.01
        preview_braking=(not self.depth_enabled and self.corner.c.enabled and self.corner.metrics['upcoming'] and self.corner.geometry_confidence>=.75 and now-self.corner.last_time<=self.engine.camera_timeout)
        if result['enabled'] and (self.corner.level>0 or preview_braking) and max(self.command_speed,self.engine.actual_speed)>result['speed']+.01:
            result['corner_braking']=True
        if result['enabled'] and result['mode'] in ('LEFT_PASS','RIGHT_PASS','MERGE_CENTER') and max(self.command_speed,self.engine.actual_speed)>result['speed']+.01:
            result['corner_braking']=True
        if context is None:
            self.recovery.observe_safe(result,self.corner,self.engine,now,self.engine.camera_stamp)
        if self.recovery.committed:result['corner_state']=self.recovery.phase
        self.recovery_pub.publish(String(data=json.dumps(wire_context(context),allow_nan=False)))
        pass_debug_key=(result['mode'],self.engine.pass_planner.state,self.engine.pass_planner.side,self.input_info.get('path_source'))
        if now-self.last_pass_log>=.2 or pass_debug_key!=self.last_pass_debug_key:
            self.last_pass_debug_key=pass_debug_key
            debug=dict(self.engine.pass_planner.debug,mode=result['mode'],planner_state=self.engine.pass_planner.state,path_source=self.input_info.get('path_source','CAMERA_PATH'))
            closest=result.get('nearest_obstacle')
            debug.update(closest_obstacle_distance=min((o.closest_distance for o in self.engine.obstacles if o.confidence>=1. and valid_path(self.engine.path) and self.engine._blocks_path(o,self.engine.path)),default=None),
                obstacle_in_drive_corridor=closest is not None,emergency_0p5m=result['reason']=='FRONT_OBSTACLE_0P5M',
                required_pass_width=self.engine.width+2*self.engine.margin,vehicle_width=self.engine.width,safety_margin=self.engine.margin,
                path_mode=result['mode'],stop_reason=self.public_stop_reason(result))
            self.pass_debug_pub.publish(String(data=json.dumps(debug)));self.get_logger().info('[OBSTACLE_PASS] '+json.dumps(debug));self.last_pass_log=now
        result['effective_path_source']=('PREVIOUS_LIDAR_PATH' if self.primary_lidar else 'PREVIOUS_PATH') if context is not None and result['enabled'] else self.engine.path_source if result['path'] is not None else ('STOP' if self.primary_lidar else 'NO_PATH')
        ceiling=self.validation_ceiling
        if ceiling<result['speed']:result.update(speed=ceiling,active_limiter='VALIDATION_CEILING')
        result['unconfirmed_target_speed']=result['speed']
        result['speed_increase_pending']=False
        if self.depth_enabled:
            raise_reason=result['active_limiter'] if result['active_limiter']!='EXISTING' else result.get('base_speed_limiter','EXISTING')
            if raise_reason=='DEPTH_UNAVAILABLE' and 'TIMEOUT' in result.get('depth_unavailable_reason',''):raise_reason='SENSOR_TIMEOUT'
            unconfirmed_raw_curve=(raise_reason=='PATH_CURVATURE' and healthy_primary and
                self.corner.level==0 and not result.get('depth_corner_confirmed') and context is None)
            limited,pending=self.speed_increase.update(result['speed'],self.engine.camera_stamp,raise_reason,
                result['enabled'],verified_decrease=not unconfirmed_raw_curve)
            if pending:result.update(speed=limited,active_limiter='SPEED_INCREASE_CONFIRMATION')
            result['speed_increase_pending']=pending
            # Use local current demand; delayed final_speed feedback is not
            # required to declare a verified speed ceiling reduction.
            limiter=self.speed_increase.reason if pending else (result['active_limiter'] if result['active_limiter']!='EXISTING' else result.get('base_speed_limiter','EXISTING'))
            # A single noisy far-end path curvature sample on a confirmed
            # straight is not a confirmed corner. Keep normal deceleration;
            # real corner/obstacle/visibility/sensor/source safety stays urgent.
            unconfirmed_curve=(limiter=='PATH_CURVATURE' and healthy_primary and
                self.corner.level==0 and not result.get('depth_corner_confirmed') and context is None)
            result['verified_speed_reduction']=not unconfirmed_curve
            result['corner_braking']=bool(result['enabled'] and not unconfirmed_curve and
                max(self.command_speed,self.engine.actual_speed)>result['speed']+.01)
        high=result['enabled'] and result['speed']>=9.5
        straight=result['enabled'] and result['corner_severity']=='STRAIGHT'
        if not high:self.high_target_since=None
        elif getattr(self,'high_target_since',None) is None:self.high_target_since=now
        if not straight:self.straight_since=None
        elif getattr(self,'straight_since',None) is None:self.straight_since=now
        result['continuous_high_speed_target_duration']=now-self.high_target_since if high else 0.
        result['continuous_straight_duration']=now-self.straight_since if straight else 0.
        self.trace_source(result,context,now)
        geometry=self.active_preview.geometry
        if geometry is not None and self.active_preview.speed_context(now) is not None and now-getattr(self,'last_preview_log',-math.inf)>=.2:
            self.last_preview_log=now
            payload={k:v for k,v in geometry.items() if k!='curvature_samples'}
            payload.update(camera_far_valid=self.braking_preview.speed_context(now) is not None and bool((self.braking_preview.geometry or {}).get('curvature_supported_preview_m',0.)),
                lidar_far_valid=self.lidar_preview.speed_context(now) is not None and bool((self.lidar_preview.geometry or {}).get('curvature_supported_preview_m',0.)),
                fused_far_valid=self.fused_preview.speed_context(now) is not None and bool((self.fused_preview.geometry or {}).get('curvature_supported_preview_m',0.)))
            payload.update(stamp=ros_now,corner_warning=self.active_preview.context(now),required_braking_distance=result['required_braking_distance'],corner_preview_speed_limit=result['corner_preview_speed_limit'],final_target_speed=result['speed'])
            payload={k:(None if isinstance(v,float) and not math.isfinite(v) else v) for k,v in payload.items()}
            self.get_logger().info('[PREVIEW_GEOMETRY] '+json.dumps(payload,allow_nan=False))
        self.publish_debug(result,now,context)
        output_header=Header(frame_id='team_01/base_link',stamp=self.get_clock().now().to_msg()) if context is not None else self.last_header
        speed_trace=dict(path_source=result['effective_path_source'],lidar_path_mode=result['lidar_path_mode'],
            corner_severity=result['corner_severity'],requested_speed=self.engine.cruise,
            decision_target_speed=result['speed'],source_speed_limit=result['source_speed_limit'],
            corner_speed_limit=result['corner_speed_limit'],preview_speed_limit=result['preview_speed_limit'],
            corner_preview_braking_limit=result['corner_preview_speed_limit'],obstacle_speed_limit=result['obstacle_limit'],
            ttc_speed_limit=result.get('ttc_speed_limit',self.engine.cruise if result['enabled'] else 0.),
            confidence_speed_limit=result.get('confidence_speed_limit',self.engine.cruise if result['enabled'] else 0.),
            confidence_speed_factor=result.get('confidence_speed_factor'),
            visible_path_speed_limit=result.get('visible_path_speed_limit'),
            lateral_acceleration_speed_limit=result.get('lateral_acceleration_speed_limit'),
            ramp_speed_limit=result['ramp_speed_limit'],continuous_high_speed_target_duration=result['continuous_high_speed_target_duration'],
            stop_reason=self.public_stop_reason(result),
            planner_limiter=result.get('base_speed_limiter','EXISTING') if result['active_limiter']=='EXISTING' else result['active_limiter'])
        speed_trace.update({k:v for k,v in result.items() if k.startswith('depth_')})
        speed_trace.update(verified_speed_reduction=result.get('verified_speed_reduction',True),
            primary_lidar_admitted=self.primary_admission.confirmed,
            primary_lidar_stability_bad_frames=self.primary_admission.bad,
            unconfirmed_target_speed=result['unconfirmed_target_speed'],
            speed_increase_pending=result['speed_increase_pending'],
            speed_reduction_reason=self.speed_increase.reason,
            speed_increase_confirm_frames=len(self.speed_increase.pending),
            depth_curvature=self.depth_track.data.get('curvature'),
            depth_heading_change=self.depth_track.data.get('heading_change_deg'),
            depth_support=self.depth_track.data.get('depth_curvature_supported_distance_m'),
            depth_segments=self.depth_track.data.get('segments',[]),
            corner_latched=self.depth_track.latched is not None,
            depth_source_stamp=self.depth_track.data.get('stamp'),
            depth_last_received_stamp=self.depth_track.last_stamp,
            history_alignment_dt=getattr(self,'history_alignment_dt',None),
            history_geometry_time=self.engine.geometry_motion_time,
            lidar_track_width_m=self.recovery.track_width,
            lidar_observed_left=self.engine.observed[0],lidar_observed_right=self.engine.observed[1],
            lidar_source_confidence=self.engine.confidence,
            depth_odom_fresh=self.corner_travel.fresh(now),
            depth_straight_confirmed=self.depth_track.straight,
            severity_change_reason=result.get('depth_unavailable_reason') if result.get('depth_corner_severity')=='UNKNOWN' else result.get('depth_state'))
        self.command_pub.publish(String(data=json.dumps(dict(stamp=ros_now,speed_trace=speed_trace,trajectory=result['path'].tolist() if result['path'] is not None else None,trajectory_header=dict(frame_id=output_header.frame_id,sec=output_header.stamp.sec,nanosec=output_header.stamp.nanosec),mode=result['mode'],path_source=result['effective_path_source'],enabled=result['enabled'],speed=result['speed'],reason=result['reason'],stop_class=self.stop_class(result),confidence=result['confidence'],single_wall=self.input_info.get('single_wall_fallback',False),corner_braking=result['corner_braking']))))
        self.publish_corner(result, now)
        self.publish_ramp(result, now)
        self.path_pub.publish(path_message(result['path'], output_header))
        self.speed_pub.publish(Float32(data=float(result['speed'])))
        self.enabled_pub.publish(Bool(data=result['enabled']))
        self.mode_pub.publish(String(data=result['mode']))
        self.state_pub.publish(String(data='RUNNING' if result['enabled'] else result['mode']))
        self.reason_pub.publish(String(data=result['reason']))
        self.ready_count_pub.publish(Int32(data=self.engine.ready_count))
        self.ready_pub.publish(Bool(data=result['enabled']))
        diag = (f"LIGHT={self.engine.light} MODE={result['mode']} PATH_SOURCE={self.input_info.get('path_source','CAMERA_PATH')} "
                f"PATH_READY={self.engine.ready_count}/{self.engine.confirm_frames} "
                f"PATH_CONF={result['confidence']:.2f} OBSTACLES={result['obstacle_count']} NEAREST={result['nearest_obstacle']} "
                f"left_gap={result['left_gap']:.3f} right_gap={result['right_gap']:.3f} "
                f"speed_target={result['speed']:.3f} TERRAIN={result['terrain_state']} "
                f"ramp_limit={result['ramp_speed_limit']:.1f} CORNER={result['corner_state']} corner_limit={result['corner_speed_limit']:.1f} limiter={result['active_limiter']} reason={result['reason']} stop_reason={result['reason'] if not result['enabled'] else 'none'}")
        self.diag_pub.publish(String(data=diag))
        if self.last_mode != result['mode'] or now-self.last_log >= 1.0:
            self.summary_pub.publish(String(data=diag))
            self.get_logger().info('[DECISION] '+diag)
            self.last_mode, self.last_log = result['mode'], now


    def constrain_depth(self,result,now):
        out=result.copy();out.update(self.corner.policy(now,self.engine.actual_speed))
        d=self.depth_track.policy(now,self.corner_travel.distance,self.corner_travel.fresh(now),self.engine.actual_speed,
            self.corner.c.speeds(self.engine.cruise),self.corner.c.corner_approach_decel_mps2,
            self.corner.c.corner_brake_margin_m,self.corner.c.reaction_time_sec)
        hold=self.current_preview_hold
        d.update(depth_preview_hold=hold is not None,depth_preview_hold_remaining_m=hold['remaining_m'] if hold else None)
        if hold is not None:
            d.update(depth_target_speed=hold['limit'],depth_corner_speed_limit=hold['limit'],
                corner_preview_braking_limit=hold['limit'],depth_state='STRAIGHT_PREVIEW_HOLD')
        out.update(d)
        # Depth-unavailable uses the measured near LiDAR corner and conservative
        # speed cap. RGB curvature never gains fallback speed authority here.
        level=d['depth_level'] if d['depth_corner_severity']!='UNKNOWN' else self.corner.level
        cap=d['depth_target_speed']
        if d['depth_corner_severity']=='UNKNOWN':cap=min(cap,self.corner.c.speeds(self.engine.cruise)[level])
        out.update(corner_severity=SEVERITIES[level],corner_speed_limit=d['depth_corner_speed_limit'],
            corner_steering_limit_deg=self.corner.c.steer_limits()[level],
            corner_state=d['depth_state'],corner_preview_speed_limit=d['corner_preview_braking_limit'],
            distance_to_corner=d['depth_remaining_corner_distance'] if d['depth_remaining_corner_distance'] is not None else math.inf)
        if d['depth_valid'] and d['depth_corner_severity']!='UNKNOWN':
            # Retained geometry keeps its original source stamp. Invalid/late
            # notifications never renew downstream corner authority.
            out.update(camera_source_stamp=self.depth_track.data.get('stamp',0.),
                camera_geometry_confidence=d['depth_confidence'],camera_geometry_age=d['depth_age'])
        if out['enabled']:
            if cap<out['speed']:out['active_limiter']='DEPTH_PREVIEW_HOLD' if hold is not None else 'DEPTH_CORNER' if d['depth_corner_confirmed'] else 'DEPTH_SUPPORTED_RANGE' if d['depth_corner_severity']=='STRAIGHT' else 'DEPTH_UNAVAILABLE'
            out['speed']=min(out['speed'],cap)
        out['corner_braking']=bool(out['enabled'] and max(self.command_speed,self.engine.actual_speed)>out['speed']+.01)
        rgb=SEVERITIES[max(self.rgb_corner.level,self.braking_preview.level)]
        d.update(rgb_corner_severity=rgb,rgb_curvature=self.rgb_corner.metrics['robust_future_curvature'],
            rgb_preview_fresh=0<=now-self.braking_preview.time<=self.braking_preview.c.timeout_sec,
            depth_curvature=self.depth_track.data.get('curvature',0.),rgb_depth_agree=rgb==d['depth_corner_severity'],
            requested_speed=self.engine.cruise,final_target_speed=out['speed'],actual_speed=self.engine.actual_speed,active_limiter=out['active_limiter'])
        if now-self.depth_last_log>=.5:
            self.depth_last_log=now
            self.depth_debug_pub.publish(String(data=json.dumps(d,allow_nan=False)))
            self.get_logger().info('[DEPTH_CORNER] '+json.dumps(d,allow_nan=False))
            self.get_logger().info('[CORNER_ODOM_TRACK] '+json.dumps({k:d[k] for k in ('corner_latched','initial_depth_corner_distance','odom_distance_at_detection','current_cumulative_odom_distance','travelled_since_detection','remaining_corner_distance','latest_depth_measurement','distance_correction','corner_age')},allow_nan=False))
        return out

    @staticmethod
    def public_stop_reason(result):
        if result['enabled']:return ''
        reason=result['reason']
        if reason=='FRONT_OBSTACLE_0P5M':return reason
        if reason=='critical_ttc':return 'CRITICAL_TTC'
        if reason=='lidar_timeout':return 'LIDAR_TIMEOUT'
        if reason in ('image_timeout','camera_timeout'):return 'IMAGE_TIMEOUT'
        if result['mode']=='STOP_BLOCKED':return 'NO_PASSABLE_GAP'
        if reason in ('lidar_path_lost','camera_path_lost','invalid_target','path_invalid') or result['mode'] in ('WAIT_PATH','RECOVERY_STOP'):return 'NO_VALID_FORWARD_PATH'
        return 'OTHER_EXISTING_FATAL_REASON'

    @staticmethod
    def stop_class(result):
        if result['enabled']:return 'RECOVERABLE_HOLD' if result['mode']=='CORNER_PATH_HOLD' else 'NONE'
        recoverable=('camera_path_lost','camera_confidence_expired','low_path_confidence',
                     'path_invalid','trajectory_jump','recovering_path_confirmation','waiting_for_controller_or_path_confirmation')
        return 'RECOVERABLE' if result['reason'] in recoverable else 'SAFETY'

    def publish_debug(self,result,now,context):
        target=None
        if valid_path(result['path']):
            try:
                from .navigation import pure_pursuit
                target=pure_pursuit(result['path'],self.engine.actual_speed,self.engine.wheelbase)[1]
            except ValueError:pass
        image_age=(self.image_age_value+now-self.image_age_time) if self.image_age_value is not None else now-self.recovery.image_time
        d=dict(self.input_info,**self.recovery.diagnostics(now))
        for key in ('raw_path_curvature','supported_path_curvature','lateral_acceleration_speed_limit',
                    'visible_path_speed_limit','confidence_speed_factor','source_speed_limit','base_speed_limiter'):
            d[key]=result.get(key)
        d.update({k:result.get(k) for k in ('supported_preview','preview_speed_limit','preview_stable_straight','preview_source')})
        d.update(path_source=result.get('effective_path_source',self.engine.path_source),input_path_source=self.input_info.get('path_source'),state='CORNER_PATH_HOLD' if context is not None and result['enabled'] else result['mode'],corner_state=result['corner_state'],
            corner_severity=result['corner_severity'],camera_age=image_age if math.isfinite(image_age) else None,
            camera_age_source='camera_image_age' if self.image_age_value is not None else 'camera_health_receipt',
            trajectory_age=now-self.engine.camera_time if math.isfinite(self.engine.camera_time) else None,
            target_x=float(target[0]) if target is not None else None,target_y=float(target[1]) if target is not None else None,
            using_previous_trajectory=bool(context is not None and result['enabled']),requested_speed=result['speed'],stop=not result['enabled'],stop_reason=result['reason'] if not result['enabled'] else '',
            corner_braking=result['corner_braking'],corner_entry_target_speed=min(result['corner_speed_limit'],result['corner_preview_speed_limit']),
            corner_entry_actual_speed=self.velocity.speed if self.velocity.fresh(now) else None,
            corner_entry_speed_error=self.velocity.speed-min(result['corner_speed_limit'],result['corner_preview_speed_limit']) if self.velocity.fresh(now) else None,
            actual_speed_source='odometry_twist' if self.velocity.fresh(now) else 'control_command_estimate',
            last_received_final_speed=self.command_speed,new_path_accepted=bool(self.recovery.input_good),
            previous_trajectory_available=self.recovery.memory is not None,
            braking_preview=self.active_preview.context(now),
            fallback_unavailable_reason=('' if context is not None else self.recovery.stop_reason or ('not_committed' if not self.recovery.committed else 'current_path_valid')),
            hold_age=now-self.recovery.memory['time'] if self.recovery.memory else None,
            hold_timeout=self.recovery.c.corner_path_hold_sec,
            stop_class=self.stop_class(result),image_ready=self.recovery.image_fresh(now),lidar_age=now-self.engine.lidar_time if math.isfinite(self.engine.lidar_time) else None)
        recovery_key=(d['state'],d['corner_state'],d['stop_reason'],d.get('trajectory_issue'),d['using_previous_trajectory'],d['corner_braking'])
        if recovery_key!=self.last_recovery_key or now-self.last_recovery_log>=.2:
            if self.primary_lidar:
                pp=self.pp_feedback if now-self.pp_feedback_time<=self.engine.camera_timeout else {}
                lidar=dict(getattr(self,'lidar_metrics',{}),stamp=self.get_clock().now().nanoseconds*1e-9,
                    lidar_path_mode=result['lidar_path_mode'],both_wall_valid=all(self.engine.observed),
                    path_valid=bool(valid_path(result['path'])),forward_point_count=len(result['path']) if result['path'] is not None else 0,
                    path_source=result['effective_path_source'],using_previous_lidar_path=bool(context is not None and result['enabled']),
                    target_x=pp.get('target_x'),target_y=pp.get('target_y'),lidar_target_y=pp.get('target_y'),
                    PP_steering=-pp['raw_steering'] if 'raw_steering' in pp else None,PP_steering_sign='internal_left_positive',final_steering=self.final_internal_steering,actual_speed=self.velocity.speed if self.velocity.fresh(now) else None)
                far=self.braking_preview;geometry=far.geometry or {};warning=far.context(now) or {}
                camera_speed=far.speed_context(now)
                turn_samples=[v['curvature'] for v in geometry.get('curvature_samples',[]) if warning and warning.get('distance_to_corner') is not None and warning['distance_to_corner']<=v['distance']<=warning['distance_to_corner']+1.2 and v.get('wall_agreement')]
                turn=float(np.median(turn_samples)) if turn_samples else 0.
                camera=dict(stamp=lidar['stamp'],supported_preview_m=camera_speed['supported_preview'] if camera_speed else 0.,
                    curvature_supported_preview_m=geometry.get('curvature_supported_preview_m'),preview_confidence=(far.speed_data or {}).get('confidence'),
                    corner_severity=SEVERITIES[warning.get('level',0)],corner_direction='LEFT' if turn>0 else 'RIGHT' if turn<0 else 'UNKNOWN',distance_to_corner=warning.get('distance_to_corner'),
                    segment_curvatures=geometry.get('segment_curvatures',[]),segment_heading_changes=geometry.get('segment_heading_changes',[]),
                    preview_speed_limit=result.get('preview_speed_limit'),freshness=now-far.time if math.isfinite(far.time) else None)
                for label,value,pub in [('LIDAR_PATH',lidar,self.lidar_steering_pub),('CAMERA_PREVIEW',camera,self.camera_preview_pub)]:
                    data=json.dumps(value);pub.publish(String(data=data));self.get_logger().info('['+label+'] '+data)
            payload=json.dumps(d,separators=(',',':'));self.recovery_debug_pub.publish(String(data=payload));self.get_logger().info('[CORNER_RECOVERY] '+payload)
            self.last_recovery_key=recovery_key;self.last_recovery_log=now
        key=(d['state'],d['corner_state'],d['stop_reason'],d.get('trajectory_issue'),d.get('path_valid'),d.get('single_wall_fallback'),d['using_previous_trajectory'])
        if key!=self.last_debug_state or now-self.last_debug_time>=1.:
            data=json.dumps(d,separators=(',',':'));self.debug_pub.publish(String(data=data));self.get_logger().info('[DECISION_DEBUG] '+data)
            self.last_debug_state=key;self.last_debug_time=now
