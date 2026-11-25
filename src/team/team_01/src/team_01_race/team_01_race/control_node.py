import math
import time
import json
import numpy as np
from builtin_interfaces.msg import Time as TimeMessage
from dataclasses import fields
from .camera_bundle import decode_bundle
from sensor_msgs.msg import Imu
from rclpy.clock import Clock,ClockType
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool, Float32, Float64, String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped
from .ros_support import ConfigNode, SENSOR_QOS, path_array, stamp_seconds
from .navigation import pure_pursuit, valid_path
from .speed_feedback import SpeedFeedback
from .ramp import RampDetector
from .ramp_brake import RampBrake,brake_model
from .drive_stability import DriveStability
from .speed_authority import safety_decrease_priority
from .corner import CornerConfig,SEVERITIES
from .corner_recovery import RecoveryConfig,RecoveryReceiver
from .steering_guard import SteeringGuard, SteeringGuardConfig


class PurePursuitControlNode(ConfigNode):
    def __init__(self):
        super().__init__('team_01_control_adapter')
        primary=self.p('primary_steering_source','lidar')
        if primary not in ('lidar','auto'):raise ValueError('primary_steering_source must be lidar or auto')
        self.primary_lidar=primary=='lidar'
        g = self.geometry()
        self.wheelbase, self.track, self.radius = g['wheelbase_m'], g['steering_track_m'], g['wheel_radius_m']
        self.limit = min(g['max_steering_angle_rad'], self.p('control.max_steering_angle_rad', .523599))
        self.max_speed = self.p('control.max_speed_mps', 7.0)
        self.accel = self.p('control.max_accel_mps2', 1.0)
        self.decel = self.p('control.max_decel_mps2', 2.5)
        assumed_decel=self.p('safety.assumed_max_decel_mps2',2.5)
        if not math.isfinite(assumed_decel) or assumed_decel<=0:
            raise ValueError('assumed braking deceleration must be finite and positive')
        # A stronger command does not establish stronger physical braking.
        self.assumed_decel=min(self.decel,assumed_decel)
        self.corner_enabled = self.p('corner_control.enabled', True)
        self.corner_decel = self.p('corner_control.corner_approach_decel_mps2', 4.0)
        if not math.isfinite(self.corner_decel) or self.corner_decel < self.decel:
            raise ValueError('corner approach deceleration must be finite and >= normal deceleration')
        self.corner_braking = False
        self.corner_braking_time = -math.inf
        self.steer_rate = self.p('control.max_steering_rate_radps', 1.5)
        self.min_lookahead = self.p('control.min_lookahead_m', 1.0)
        self.max_lookahead = self.p('control.max_lookahead_m', 3.0)
        self.lookahead_alpha = self.p('control.lookahead_alpha', .25)
        # Preserve the prior absolute-speed/lookahead mapping during the 7 m/s experiment.
        self.lookahead_speed_reference = self.p('control.lookahead_speed_reference_mps', 10.0)
        if not math.isfinite(self.lookahead_speed_reference) or self.lookahead_speed_reference <= 0:
            raise ValueError('lookahead speed reference must be finite and positive')
        self.max_lateral_accel = self.p('control.max_lateral_accel_mps2', 3.0)
        self.command_timeout = self.p('safety.command_timeout_sec', .25)
        self.path_timeout = self.p('safety.path_timeout_sec', .5)
        guard_config=SteeringGuardConfig()
        for field in fields(guard_config):
            setattr(guard_config,field.name,self.p('steering_guard.'+field.name,getattr(guard_config,field.name)))
        if guard_config.yaw_safety_enabled:
            guard_config.corner_current_curvature_threshold=self.p('corner_control.corner_current_curvature_threshold',.10)
        self.guard=SteeringGuard(guard_config,self.max_speed,self.assumed_decel)
        self.corner_config=CornerConfig()
        for field in fields(self.corner_config):
            setattr(self.corner_config,field.name,self.p('corner_control.'+field.name,getattr(self.corner_config,field.name)))
        self.corner_config.validate(self.max_speed)
        self.corner_policy={}
        self.create_subscription(String,'/team_01/internal/corner_policy',self.on_corner_policy,10)
        self.corner_severity_status_pub=self.create_publisher(String,'/team_01/debug/corner_severity_status',10)
        rc=RecoveryConfig()
        for field in fields(rc):setattr(rc,field.name,self.p('corner_path_recovery.'+field.name,getattr(rc,field.name)))
        rc.corner_path_hold_speed_mps=min(rc.corner_path_hold_speed_mps,self.p('path_manager.previous_path_max_speed_mps',1.2))
        self.recovery_receiver=RecoveryReceiver(rc,self.command_timeout)
        self.create_subscription(Bool,'/team_01/debug/lidar_scan_ready',lambda m:self.recovery_receiver.lidar_health(m.data,self.now_seconds()),1)
        self.create_subscription(Bool,'/team_01/debug/camera_ready',lambda m:self.recovery_receiver.health(m.data,self.now_seconds()),1)
        self.create_subscription(String,'/team_01/internal/corner_path_context',self.on_recovery_context,10)
        self.create_subscription(String,'/team_01/internal/decision_command',self.on_decision_command,10)
        self.recovery_debug={}
        self.command_source='STOP' if self.primary_lidar else 'CAMERA_PATH';self.command_mode='WAIT_LIGHT'
        self.pp_pub=self.create_publisher(String,'/team_01/debug/pp',10)
        self.speed_control_pub=self.create_publisher(String,'/team_01/debug/speed_control',10)
        self.speed_trace={};self.last_speed_trace_log=-math.inf
        self.drive_stability=DriveStability();self.last_stability_log=-math.inf
        self.stability_pub=self.create_publisher(String,'/team_01/debug/drive_stability',1)
        self.recovery_control_pub=self.create_publisher(String,'/team_01/debug/corner_control',10)
        self.create_subscription(String,'/team_01/debug/corner_recovery',self.on_recovery_debug,10)
        self.last_recovery_control_log=-math.inf;self.last_recovery_control_key=None
        self.atomic_time=-math.inf;self.atomic_has_path=False;self.last_command_stamp=-math.inf;self.decision_reason='reason_unavailable'
        self.camera_packets={};self.last_camera_guard_stamp=-math.inf;self.last_decision_mode=None
        self.guard_status_pub=self.create_publisher(String,'/team_01/debug/steering_guard_status',10)
        self.guard_state_pub=self.create_publisher(String,'/team_01/debug/steering_guard_state',10)
        self.guard_limit_pub=self.create_publisher(Float32,'/team_01/debug/steering_guard_speed_limit',10)
        self.last_guard_log=-math.inf;self.last_guard_state=None
        rate = self.p('control.command_rate_hz', 50.0)
        if min(self.wheelbase, self.radius, self.limit, self.max_speed, self.accel, self.decel, self.steer_rate, rate) <= 0:
            raise ValueError('invalid control configuration')
        self.path = None
        self.path_stamp = None
        self.path_time = self.speed_time = self.enabled_time = -math.inf
        self.requested_speed = 0.0
        self.enabled = self.emergency = False
        self.speed = self.steering = 0.0  # Internally + steering means LEFT.
        self.motion_speed_bound=0.0
        self.velocity=SpeedFeedback(self.p('safety.velocity_timeout_sec',.25))
        mass,wheels=brake_model(self.get_parameter('vehicle_model_file').value)
        self.ramp_brake=RampBrake(mass,self.radius,wheels,RampDetector(
            alpha=self.p('imu.pitch_alpha',.30),enter_deg=self.p('ramp.enter_deg',5.),
            exit_deg=self.p('ramp.exit_deg',3.),confirm_frames=self.p('ramp.confirm_frames',3),
            exit_confirm_frames=self.p('ramp.exit_confirm_frames',3),
            timeout=self.p('safety.imu_timeout_sec',.25)))
        self.brake_pub=self.create_publisher(String,'/team_01/debug/ramp_brake',10)
        self.create_subscription(Odometry,'/team_01/odometry',self.on_velocity,SENSOR_QOS)
        self.lookahead = None
        self.last_time = self.now_seconds()
        self.last_log = -math.inf
        self.speed_pub = self.create_publisher(Float32, self.p('speed_topic', '/team_01/speed'), 10)
        self.steer_pub = self.create_publisher(Float32, self.p('steering_topic', '/team_01/steering'), 10)
        self.final_pub = self.create_publisher(Float32, '/team_01/debug/final_speed', 10)
        self.final_steer_pub = self.create_publisher(Float32, '/team_01/debug/final_steering', 10)
        self.desired_pub = self.create_publisher(Float32, '/team_01/debug/desired_steering', 10)
        self.target_pub = self.create_publisher(PointStamped, '/team_01/debug/lookahead_target', 10)
        self.lookahead_pub = self.create_publisher(Float32, '/team_01/debug/lookahead', 10)
        self.ready_pub = self.create_publisher(Bool, '/team_01/internal/controller_ready', 10)
        self.status_pub = self.create_publisher(String, '/team_01/debug/control_status', 10)
        self.reason_pub = self.create_publisher(String, '/team_01/debug/control_stop_reason', 10)
        self.actuators = {name: self.create_publisher(Float64, '/team_01/actuator/'+name, 10)
                          for name in ('rear_left_velocity', 'rear_right_velocity', 'front_left_position', 'front_right_position')}
        self.create_subscription(Bool, '/team_01/internal/corner_braking', self.on_corner_braking, 1)
        self.create_subscription(Path, '/team_01/selected_path', self.on_path, 10)
        self.create_subscription(Float32, '/team_01/internal/requested_speed', self.on_speed, 10)
        self.create_subscription(Bool, '/team_01/internal/drive_enabled', self.on_enabled, 10)
        self.create_subscription(Bool, '/team_01/emergency_stop', self.on_emergency, 10)
        self.create_subscription(Imu,self.p('imu_topic','/team_01/imu'),self.on_imu,SENSOR_QOS)
        if guard_config.yaw_safety_enabled:
            self.create_subscription(String,'/team_01/debug/corner_state',lambda m:self.guard.set_corner(m.data,self.now_seconds()),1)
        self.create_subscription(String,'/team_01/debug/mode',self.on_decision_mode,10)
        if self.primary_lidar:
            self.create_subscription(String,'/team_01/path_manager/geometry_bundle',self.on_camera_bundle,1)
        elif self.p('camera_geometry.prefer_bundle',False):
            self.create_subscription(String,self.p('camera_geometry.bundle_topic','/team_01/camera/geometry_bundle'),self.on_camera_bundle,1)
        else:
            for name in ('center_path','left_boundary','right_boundary'):
                self.create_subscription(Path,'/team_01/camera/'+name,lambda m,name=name:self.camera_part(name,m),1)
            self.create_subscription(DiagnosticArray,'/team_01/camera/path_status',lambda m:self.camera_part('status',m),1)
        self.create_timer(1.0/rate, self.tick)
        self.create_timer(.05,self.clock_watchdog,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def clock_watchdog(self):
        if not self.simulation_clock_healthy(self.command_timeout):self.tick()

    def on_camera_bundle(self,msg):
        try:
            if self.primary_lidar and json.loads(msg.data).get('path_source') not in ('LIDAR_PATH','NO_PATH'):return
            parts=decode_bundle(msg.data,self.get_clock().now().nanoseconds*1e-9,self.guard.c.camera_timeout_sec)
            for name,part in parts.items():self.camera_part(name,part)
        except (ValueError,TypeError,KeyError,IndexError,OverflowError):return

    def on_recovery_debug(self,msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):self.recovery_debug=data
        except (ValueError,TypeError):pass

    def on_recovery_context(self,msg):
        try:
            data=json.loads(msg.data)
            if self.primary_lidar and data.get('holding') and data.get('source_sensor')!='LIDAR_PATH':
                self.recovery_receiver.data=None;return
            self.recovery_receiver.update(data,self.get_clock().now().nanoseconds*1e-9,self.now_seconds())
        except (ValueError,TypeError):return

    def on_decision_command(self,msg):
        now=self.now_seconds()
        try:
            data=json.loads(msg.data);speed=float(data['speed']);stamp=float(data['stamp'])
            if not isinstance(data['enabled'],bool) or not math.isfinite(speed) or not math.isfinite(stamp) or speed<0:raise ValueError('invalid_decision_command')
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            # Reject reordered/future motion commands without replacing the
            # last valid snapshot. Its existing watchdog is not refreshed.
            # A stop always retains immediate priority.
            if data['enabled'] and (not -.05<=age<=self.command_timeout or stamp<=self.last_command_stamp):return
            self.last_command_stamp=max(self.last_command_stamp,stamp)
            if self.primary_lidar and data['enabled'] and data.get('path_source') not in ('LIDAR_PATH','PREVIOUS_LIDAR_PATH'):
                raise ValueError('non_lidar_steering_source')
            if not data['enabled']:speed=0.
            self.command_source=data.get('path_source',self.command_source);self.command_mode=data.get('mode',self.command_mode)
            self.speed_trace=data.get('speed_trace',{})
            previous=self.enabled;self.enabled=data['enabled'];self.requested_speed=speed
            self.decision_reason=str(data['reason'])
            self.atomic_time=self.enabled_time=self.speed_time=now
            self.corner_braking=bool(data.get('corner_braking',False));self.corner_braking_time=now
            self.guard.single_wall_reacquired=bool(self.enabled and data.get('single_wall') and float(data.get('confidence',0))>=self.recovery_receiver.c.memory_min_confidence)
            self.atomic_has_path='trajectory' in data
            if self.atomic_has_path and self.enabled:
                path=np.asarray(data['trajectory'],dtype=float)
                header=data['trajectory_header']
                if header['frame_id']!='team_01/base_link' or not valid_path(path):raise ValueError('invalid_atomic_trajectory')
                sec=int(header['sec']);nanosec=int(header['nanosec'])
                if not 0<=sec<2**31 or not 0<=nanosec<10**9:raise ValueError('invalid_trajectory_stamp')
                source=TimeMessage(sec=sec,nanosec=nanosec)
                self.accept_selected_path(path,source,now)
            if previous and not self.enabled:self.tick()
        except (ValueError,KeyError,TypeError,OverflowError):
            self.enabled=False;self.requested_speed=0.;self.decision_reason='invalid_decision_command';self.tick()

    def on_corner_policy(self,msg):
        try:
            policy=json.loads(msg.data);severity=policy['corner_severity']
            stamp=float(policy['camera_source_stamp']);confidence=float(policy['camera_geometry_confidence'])
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            if (policy.get('frame_id')!='team_01/base_link' or severity not in SEVERITIES or
                not math.isfinite(age) or not -.05<=age<=self.guard.c.camera_timeout_sec or
                not math.isfinite(confidence) or not 0<=confidence<=1.):return
            single_trusted=self.guard.single_wall_reacquired and confidence>=self.recovery_receiver.c.memory_min_confidence
            if confidence<self.guard.c.recovery_min_confidence and not single_trusted:
                # Fresh geometry conflict/low confidence revokes old corner
                # authority immediately; do not wait for its old stamp to age.
                self.guard.severity_time=-math.inf
                self.corner_policy=policy
                return
            level=SEVERITIES.index(severity)
            speed=self.corner_config.speeds(self.max_speed)[level];steer=self.corner_config.steer_limits()[level]
            # The receiving node derives authority from its shared config,
            # never from an arbitrary numeric steering value on the topic.
            self.corner_policy=policy
            self.guard.set_severity(severity,speed,steer,self.now_seconds()-max(0.,age))
        except (ValueError,KeyError,TypeError):return

    def on_decision_mode(self,msg):
        if msg.data in ('STOP_RED','WAIT_LIGHT') and self.last_decision_mode!=msg.data:
            self.guard.reset_mission()
        self.last_decision_mode=msg.data

    def age(self,stamp):
        return self.get_clock().now().nanoseconds*1e-9-stamp_seconds(stamp)

    def on_imu(self,msg):
        q=msg.orientation
        self.ramp_brake.update((q.x,q.y,q.z,q.w),stamp_seconds(msg.header.stamp),
            self.now_seconds(),age=self.age(msg.header.stamp),orientation_available=(
            msg.header.frame_id=='team_01/imu_link' and msg.orientation_covariance[0]!=-1))
        self.guard.imu.update((q.x,q.y,q.z,q.w),msg.angular_velocity.z,stamp_seconds(msg.header.stamp),
            self.now_seconds(),age=self.age(msg.header.stamp),available=(msg.header.frame_id=='team_01/imu_link' and
            msg.orientation_covariance[0]!=-1 and msg.angular_velocity_covariance[0]!=-1))

    def camera_part(self,name,msg):
        stamp=stamp_seconds(msg.header.stamp);age=self.age(msg.header.stamp)
        if stamp<=self.last_camera_guard_stamp or not -.05<=age<=self.guard.c.camera_timeout_sec:
            return
        packet=self.camera_packets.setdefault(stamp,{})
        packet[name]=msg
        if len(self.camera_packets)>8:del self.camera_packets[min(self.camera_packets)]
        required=('center_path','left_boundary','right_boundary','status')
        if not all(k in packet for k in required):return
        self.last_camera_guard_stamp=stamp
        if any(packet[k].header.frame_id!='team_01/base_link' for k in required[:3]):return
        status=packet['status'].status
        values={v.key:v.value for v in status[0].values} if status else {}
        try:confidence=float(values.get('confidence','0'))
        except ValueError:return
        self.guard.camera_geometry(*(path_array(packet[k]) for k in required[:3]),confidence,stamp,self.now_seconds()-max(0.,age))

    def on_velocity(self,msg):
        v=msg.twist.twist.linear
        self.velocity.update(stamp_seconds(msg.header.stamp),self.now_seconds(),self.age(msg.header.stamp),(v.x,v.y,v.z))

    def on_corner_braking(self, msg):
        if self.now_seconds()-self.atomic_time<=self.command_timeout:return
        self.corner_braking = msg.data
        self.corner_braking_time = self.now_seconds()

    def accept_selected_path(self,path,source_stamp,now):
        try:
            if self.age(source_stamp)<-.05:raise ValueError('future_path')
            accepted=self.guard.accept_path(path,stamp_seconds(source_stamp),now,age=self.age(source_stamp))
        except (ValueError,FloatingPointError):
            self.path=None;self.path_time=now;return
        if accepted:self.path=path;self.path_stamp=source_stamp
        self.path_time=now

    def on_path(self,msg):
        if self.primary_lidar:return
        now=self.now_seconds()
        # Current Decision commands carry the selected path and flags in one
        # snapshot. The public Path topic remains available for visualization
        # and legacy publishers, without overwriting a fresh atomic snapshot.
        if self.atomic_has_path and now-self.atomic_time<=self.command_timeout:return
        path=path_array(msg) if msg.header.frame_id=='team_01/base_link' else None
        self.accept_selected_path(path,msg.header.stamp,now)

    def on_speed(self, msg):
        if self.now_seconds()-self.atomic_time<=self.command_timeout:return
        self.requested_speed = msg.data
        self.speed_time = self.now_seconds()

    def on_enabled(self, msg):
        if self.now_seconds()-self.atomic_time<=self.command_timeout:return
        changed = self.enabled != msg.data
        self.enabled = msg.data
        self.enabled_time = self.now_seconds()
        if changed and not self.enabled:
            self.tick()  # RED / visual stop / emergency must bypass normal smoothing.

    def on_emergency(self, msg):
        self.emergency = msg.data
        self.tick()

    @staticmethod
    def toward(current, target, change):
        return current+max(-change, min(change, target-current))

    def tick(self):
        now = self.now_seconds()
        dt = min(.1, max(0.0, now-self.last_time))
        self.last_time = now
        # A zero wheel-speed command is not instantaneous body deceleration.
        # Retain a conservative bound across stop/re-enable cycles so the
        # low-speed steering authority cannot reopen while the car coasts.
        if self.velocity.fresh(now):
            self.motion_speed_bound=max(self.speed,self.velocity.speed+self.accel*max(0.,now-self.velocity.time))
        else:
            self.motion_speed_bound=max(self.speed,self.motion_speed_bound-self.assumed_decel*dt,0.)
        reason = ''
        if self.emergency:
            reason = 'external_emergency'
        elif not self.simulation_clock_healthy(self.command_timeout):
            reason='simulation_clock_timeout'
        elif not self.enabled:
            reason = 'decision_stop:'+{'camera_path_lost':'path_lost','camera_confidence_expired':'low_path_confidence','no_vehicle_width_gap':'obstacle','confirmed_visual_red':'red_light'}.get(self.decision_reason,self.decision_reason)
        elif max(now-self.path_time, now-self.speed_time, now-self.enabled_time) > self.command_timeout:
            reason = 'command_timeout'
        elif not math.isfinite(self.requested_speed) or not valid_path(self.path):
            reason = 'invalid_selected_path_or_speed'
        elif self.path_stamp is not None:
            ros_now = self.get_clock().now().nanoseconds*1e-9
            stamp = stamp_seconds(self.path_stamp)
            if ros_now > 0 and stamp > 0 and ros_now-stamp > self.path_timeout:
                reason = 'selected_camera_path_timeout'
        desired = raw_pp = 0.0
        desired_lookahead=None
        lookahead_speed=self.velocity.speed if self.velocity.fresh(now) else self.motion_speed_bound
        lookahead_speed_source='odometry_twist' if self.velocity.fresh(now) else 'bounded_motion_estimate'
        target = None
        if reason and self.guard.c.enabled and self.motion_speed_bound>self.guard.c.recovery_speed_mps:
            self.guard.braking_hold=True;self.guard.recovery=True;self.guard.exit_count=0
        if not reason:
            try:
                raw_pp, target, lookahead = pure_pursuit(self.path, lookahead_speed, self.wheelbase,
                                                        self.min_lookahead, self.max_lookahead, self.lookahead_speed_reference)
                desired_lookahead=lookahead
                self.lookahead = lookahead if self.lookahead is None else self.lookahead_alpha*lookahead+(1-self.lookahead_alpha)*self.lookahead
                raw_pp, target, _ = pure_pursuit(self.path, lookahead_speed, self.wheelbase,
                                                 self.min_lookahead, self.max_lookahead, self.lookahead_speed_reference,
                                                 lookahead_override=self.lookahead)
                desired = max(-self.limit, min(self.limit, raw_pp))
            except (ValueError, FloatingPointError):
                reason = 'invalid_pure_pursuit_target'
        if self.primary_lidar and self.guard.camera_fresh(now) and valid_path(self.path):
            from .corner import preview_geometry
            metrics=preview_geometry(self.path,self.corner_config)
            level=metrics['candidate_level']
            if 0<=self.get_clock().now().nanoseconds*1e-9-float(self.corner_policy.get('camera_source_stamp',-math.inf))<=self.command_timeout:
                level=max(level,SEVERITIES.index(self.corner_policy.get('corner_severity','STRAIGHT')))
            self.guard.set_severity(SEVERITIES[level],self.corner_config.speeds(self.max_speed)[level],self.corner_config.steer_limits()[level],now)
        recovery_context=self.recovery_receiver.context(self.get_clock().now().nanoseconds*1e-9,now)
        self.guard.recovery_context=recovery_context
        if recovery_context is not None:
            level=SEVERITIES.index(recovery_context['severity'])
            self.guard.recovery_limits=(min(self.recovery_receiver.c.corner_path_hold_speed_mps,self.corner_config.speeds(self.max_speed)[level]),self.corner_config.steer_limits()[level])
        guarded, guard_cap, guard_stop, guard_status = self.guard.evaluate(
            raw_pp,target,self.speed,self.steering,now,dt,motion_speed=self.motion_speed_bound)
        if guard_stop and not reason:
            reason='steering_guard_'+guard_status['reason']
        steering_speed_limit=None;speed_target=0.;verified_reduction=False
        if not reason:
            # Existing lateral-acceleration and rate limits operate on the
            # proposal after the guard. The guard never creates a new sign.
            state_limit = min(self.limit, math.radians(guard_status['state_steering_limit_deg'])) if self.guard.c.enabled else self.limit
            guarded = max(-self.limit, min(self.limit, guarded))
            moving_limit = math.atan(self.max_lateral_accel*self.wheelbase/max(self.motion_speed_bound*self.motion_speed_bound,1e-6))
            moving_target = max(-moving_limit,min(moving_limit,guarded))
            self.steering = self.toward(self.steering,moving_target,self.steer_rate*dt)
            self.steering = max(-state_limit,min(state_limit,self.steering))
            steering_curvature = max(abs(math.tan(guarded)),abs(math.tan(self.steering)))/self.wheelbase
            steering_speed_limit = math.sqrt(self.max_lateral_accel/max(1e-6,steering_curvature))
            speed_target = max(0.,min(self.max_speed,self.requested_speed,steering_speed_limit,guard_cap))
            decel = self.corner_decel if (self.corner_enabled and self.corner_braking and
                now-self.corner_braking_time <= self.command_timeout) else self.decel
            verified_reduction=safety_decrease_priority(self.speed,speed_target,self.requested_speed,guard_cap,steering_speed_limit,
                command_fresh=0<=now-self.atomic_time<=self.command_timeout and self.speed_trace.get('verified_speed_reduction',True),
                limiter=self.speed_trace.get('planner_limiter',''))
            self.speed = self.toward(self.speed,speed_target,(self.accel if speed_target>=self.speed else decel)*dt)
            corner_clamp=(self.corner_enabled and self.corner_braking and now-self.corner_braking_time<=self.command_timeout) or recovery_context is not None or verified_reduction
            if corner_clamp:
                self.speed=min(self.speed,speed_target)
            self.motion_speed_bound=max(self.motion_speed_bound,self.speed)
            moving_limit = min(self.limit,state_limit,math.atan(self.max_lateral_accel*self.wheelbase/max(self.motion_speed_bound*self.motion_speed_bound,1e-6)))
            self.steering = max(-moving_limit,min(moving_limit,self.steering))
        if reason:
            self.speed = self.steering = 0.0
            self.lookahead = None
        guard_status['lateral_acceleration_steering_limit_deg']=math.degrees(math.atan(self.max_lateral_accel*self.wheelbase/max(self.motion_speed_bound**2,1e-6)))
        guard_status['steering_limited_speed_mps']=steering_speed_limit
        guard_status['actual_speed']=self.velocity.speed if self.velocity.fresh(now) else None
        guard_status['actual_speed_source']='odometry_twist' if self.velocity.fresh(now) else 'unavailable'
        guard_status['corner_braking']=bool(self.corner_enabled and self.corner_braking and now-self.corner_braking_time<=self.command_timeout)
        guard_status['corner_braking_clamp']=bool(not reason and (guard_status['corner_braking'] or recovery_context is not None or verified_reduction))
        guard_status['confirmed_safety_speed_reduction']=verified_reduction
        guard_status['decision_target_speed']=self.requested_speed
        guard_status['final_target_speed']=speed_target
        planner_limiter=self.speed_trace.get('planner_limiter',self.corner_policy.get('active_limiter','EXISTING'))
        if planner_limiter=='EXISTING':planner_limiter=self.corner_policy.get('base_speed_limiter') or 'EXISTING'
        guard_status['active_speed_limiter']=('STOP' if reason else 'LATERAL_ACCELERATION' if steering_speed_limit is not None and steering_speed_limit<min(self.requested_speed,guard_cap)-1e-5 else 'CORNER_SEVERITY' if guard_cap<self.requested_speed-1e-5 else planner_limiter)
        guard_status['final_steering_deg']=math.degrees(self.steering)
        guard_status['final_speed']=self.speed
        guard_status['control_stop_reason']=reason
        speed_trace=dict(self.speed_trace,stamp=self.get_clock().now().nanoseconds*1e-9,
            lookahead_speed_source=lookahead_speed_source,
            decision_stamp=self.last_command_stamp if math.isfinite(self.last_command_stamp) else None,path_source=self.command_source,
            decision_target_speed=self.requested_speed,final_target_speed=speed_target,
            final_speed=self.speed,actual_speed=guard_status['actual_speed'],
            command_accel_mps2=self.accel,command_decel_mps2=self.decel,
            corner_command_decel_mps2=self.corner_decel,assumed_braking_decel_mps2=self.assumed_decel,
            active_limiter=guard_status['active_speed_limiter'],
            acceleration_limited=bool(not reason and self.speed<speed_target-1e-5),
            confirmed_safety_speed_reduction=verified_reduction,
            stop=bool(reason),control_stop_reason=reason)
        if reason and not speed_trace.get('stop_reason'):
            speed_trace['stop_reason']='NO_VALID_FORWARD_PATH' if 'NO_VALID_FORWARD_TRAJECTORY' in reason or 'selected_path' in reason else 'OTHER_EXISTING_FATAL_REASON'
        stability=self.drive_stability.snapshot(speed_trace,self.path,
            stamp=self.get_clock().now().nanoseconds*1e-9,requested=self.requested_speed,final_target=speed_target,
            final=self.speed,actual=guard_status['actual_speed'],braking=guard_status['corner_braking'],
            clamp=guard_status['corner_braking_clamp'],limiter=guard_status['active_speed_limiter'],stop_reason=reason,
            source=self.command_source,desired_lookahead=desired_lookahead,effective_lookahead=self.lookahead,
            lookahead_speed=lookahead_speed,target=target,raw_steering=raw_pp,steering=self.steering)
        self.stability_pub.publish(String(data=json.dumps(stability,allow_nan=False)))
        if now-self.last_stability_log>=.2:
            self.get_logger().info('[DRIVE_STABILITY] '+json.dumps(stability,allow_nan=False));self.last_stability_log=now
        self.speed_control_pub.publish(String(data=json.dumps(speed_trace)))
        if now-self.last_speed_trace_log>=.2:
            self.get_logger().info('[SPEED_CONTROL] '+json.dumps(speed_trace));self.last_speed_trace_log=now
        self.guard_state_pub.publish(String(data=guard_status['state']))
        self.guard_limit_pub.publish(Float32(data=float(guard_cap)))
        if now-self.last_guard_log>=1.0 or guard_status['state']!=self.last_guard_state:
            message=json.dumps(guard_status,separators=(',',':'))
            self.guard_status_pub.publish(String(data=message))
            severity_status={**self.corner_policy,**guard_status,'lookahead_m':self.lookahead,'current_speed':self.speed}
            self.corner_severity_status_pub.publish(String(data=json.dumps(severity_status,separators=(',',':'))))
            self.get_logger().info('[CORNER_SEVERITY] '+json.dumps(severity_status,separators=(',',':')))
            self.get_logger().info('[STEERING_GUARD] '+message)
            self.last_guard_log=now;self.last_guard_state=guard_status['state']
        recovery_log=dict(self.recovery_debug,requested_speed=self.requested_speed,final_speed=self.speed,
            corner_braking=guard_status['corner_braking'],corner_braking_clamp=guard_status['corner_braking_clamp'],
            using_previous_trajectory=recovery_context is not None,stop_reason=reason,stop=bool(reason),
            stop_class=(self.recovery_debug.get('stop_class','SAFETY') if reason.startswith('decision_stop:') else 'SAFETY') if reason else ('RECOVERABLE_HOLD' if recovery_context is not None else 'NONE'),
            target_x=float(target[0]) if target is not None and not reason else None,
            target_y=float(target[1]) if target is not None and not reason else None,
            actual_speed=guard_status['actual_speed'],actual_speed_source=guard_status['actual_speed_source'],
            corner_entry_target_speed=min(self.corner_config.speeds(self.max_speed)[SEVERITIES.index(guard_status.get('corner_severity',self.guard.severity))],self.corner_policy.get('corner_preview_speed_limit',self.max_speed)),
            active_speed_limiter=guard_status['active_speed_limiter'],
            steering_speed_limit=steering_speed_limit,
            acceleration_limited=bool(not reason and self.speed<speed_target-1e-5),
            control_stamp=self.get_clock().now().nanoseconds*1e-9)
        if math.isfinite(self.atomic_time):
            # Recovery diagnostics arrive independently. Report the mode from
            # the same atomic command as requested_speed, not a newer/older
            # diagnostic packet's mode.
            recovery_log.update(recovery_debug_state=self.recovery_debug.get('state'),
                state=self.command_mode,path_source=self.command_source,
                decision_command_stamp=self.last_command_stamp)
        recovery_log['corner_entry_actual_speed']=guard_status['actual_speed']
        recovery_log['corner_entry_speed_error']=None if guard_status['actual_speed'] is None else guard_status['actual_speed']-recovery_log['corner_entry_target_speed']
        self.recovery_control_pub.publish(String(data=json.dumps(recovery_log,separators=(',',':'))))
        pp=dict(path_source=self.command_source,mode=self.command_mode,lookahead=self.lookahead,target_x=recovery_log['target_x'],target_y=recovery_log['target_y'],raw_steering=-raw_pp,final_steering=-self.steering,requested_speed=self.requested_speed,final_speed=self.speed,stop_reason=reason,steering_sign='official_right_positive')
        self.pp_pub.publish(String(data=json.dumps(pp)))
        recovery_key=(reason,guard_status['state'],guard_status['corner_braking'],recovery_context is not None)
        if recovery_key!=self.last_recovery_control_key or now-self.last_recovery_control_log>=.2:
            self.get_logger().info('[PP] '+json.dumps(pp))
            self.get_logger().info('[CORNER_RECOVERY] '+json.dumps(recovery_log,separators=(',',':')))
            self.last_recovery_control_key=recovery_key;self.last_recovery_control_log=now
        # The single interface conversion: internal LEFT-positive -> official
        # RIGHT-positive. Joint positions use internal geometry directly.
        official = -self.steering
        self.speed_pub.publish(Float32(data=float(self.speed)))
        self.final_pub.publish(Float32(data=float(self.speed)))
        self.steer_pub.publish(Float32(data=float(official)))
        self.final_steer_pub.publish(Float32(data=float(official)))
        self.desired_pub.publish(Float32(data=float(desired)))
        self.ready_pub.publish(Bool(data=not self.emergency and self.simulation_clock_healthy(self.command_timeout)))
        self.reason_pub.publish(String(data=reason))
        tangent = math.tan(self.steering)
        left = math.atan2(self.wheelbase*tangent, self.wheelbase-.5*self.track*tangent)
        right = math.atan2(self.wheelbase*tangent, self.wheelbase+.5*self.track*tangent)
        brake=self.ramp_brake.command(now,stop_requested=bool(reason or self.requested_speed<=.001) and self.speed<=.001)
        self.brake_pub.publish(String(data=json.dumps(dict(brake,stamp=self.get_clock().now().nanoseconds*1e-9,
            stop_reason=reason,speed_command=self.speed))))
        values = {'rear_left_velocity': self.speed/self.radius+brake['velocity_bias_radps'][0],
                  'rear_right_velocity': self.speed/self.radius+brake['velocity_bias_radps'][1],
                  'front_left_position': max(-self.limit, min(self.limit, left)),
                  'front_right_position': max(-self.limit, min(self.limit, right))}
        for name, value in values.items():
            self.actuators[name].publish(Float64(data=float(value)))
        if target is not None and not reason:
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = 'team_01/base_link'
            point.point.x, point.point.y = float(target[0]), float(target[1])
            self.target_pub.publish(point)
            self.lookahead_pub.publish(Float32(data=float(self.lookahead)))
        if now-self.last_log >= 1.0:
            if brake['active'] and brake['saturated']:
                self.get_logger().warning('[RAMP_BRAKE] required holding torque exceeds wheel torque limit')
            target_text = f'({target[0]:.2f},{target[1]:.2f})' if target is not None else 'none'
            self.get_logger().info(f'[CONTROL] target={target_text} speed_target={self.requested_speed:.2f} final_speed={self.speed:.2f} lookahead={self.lookahead} official_steer={official:.3f} stop={reason}')
            self.status_pub.publish(String(data=f'requested_speed={self.requested_speed:.3f} final_speed={self.speed:.3f} official_steering={official:.6f} corner_braking={self.corner_enabled and self.corner_braking and now-self.corner_braking_time <= self.command_timeout} lookahead={self.lookahead} target={target_text} stop={reason}'))
            self.last_log = now
