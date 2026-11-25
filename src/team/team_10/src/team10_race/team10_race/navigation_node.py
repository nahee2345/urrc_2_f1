"""ROS node for map-free, LiDAR-only continuous-wall racing."""

from __future__ import annotations

import math
import time
from collections import Counter

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from .start_interlock import StartInterlock

from .centerline import build_centerline, build_scan_centerline, choose_centerline
from .controller import PurePursuitController
from .lidar_processing import continuous_segments, scan_to_points
from .safety import perception_is_valid, scan_is_fresh
from .speed_planner import PreviewSpeedPlanner
from .wall_tracker import WallTracker


class NavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_10")
        defaults = {
            "race_start_topic": "/race/start/team_10",
            "speed_topic": "/team_10/speed_10",
            "steering_topic": "/team_10/steering_10",
            "lidar_topic": "/team_10/scan_10",
            "lidar.fov_deg": 270.0,
            "lidar.min_range": 0.25,
            "lidar.max_range": 24.0,
            "lidar.maximum_gap_m": 0.30,
            "lidar.maximum_missing_beams": 2,
            "wall.minimum_points": 8,
            "wall.bin_size_m": 0.20,
            "wall.maximum_residual_m": 0.28,
            "wall.anchor_x_m": 0.30,
            "wall.maximum_forward_m": 16.0,
            "wall.initial_track_width_m": 2.20,
            "wall.minimum_track_width_m": 1.70,
            "wall.maximum_track_width_m": 2.70,
            "wall.width_filter_alpha": 0.12,
            "centerline.preview_distance_m": 15.0,
            "centerline.sample_step_m": 0.25,
            "centerline.minimum_points": 5,
            "centerline.scan_bin_width_m": 0.40,
            "centerline.scan_minimum_points_per_side": 2,
            "centerline.scan_inner_fraction": 0.50,
            "centerline.scan_minimum_span_m": 1.60,
            "centerline.scan_minimum_width_factor": 0.55,
            "centerline.scan_maximum_width_factor": 2.25,
            "centerline.scan_maximum_lateral_jump_m": 0.55,
            "centerline.scan_maximum_heading_change_rad": 0.85,
            "centerline.scan_maximum_curvature_inv_m": 1.50,
            "controller.lookahead_m": 1.40,
            "controller.wheelbase_m": 0.340,
            "controller.maximum_steering_rad": 0.488692,
            "controller.medium_steering_rad": 0.14,
            "controller.corner_steering_rad": 0.32,
            "controller.steering_filter_alpha": 0.45,
            "controller.steering_rate_limit_rad_s": 2.00,
            "speed.max_mps": 2.50,
            "speed.continuous_limits": False,
            "speed.reaction_time_s": 0.30,
            "speed.medium_mps": 1.50,
            "speed.corner_mps": 0.70,
            "speed.preview_distance_m": 14.0,
            "speed.curvature_low": 0.05,
            "speed.curvature_medium": 0.12,
            "speed.curvature_high": 0.28,
            "speed.maximum_accel_mps2": 1.50,
            "speed.maximum_decel_mps2": 3.00,
            "speed.braking_margin_m": 0.60,
            "speed.lateral_accel_limit_mps2": 2.50,
            "speed.corridor_half_width_m": 0.28,
            "speed.emergency_clearance_m": 0.55,
            "speed.boost_entry_time_s": 0.80,
            "speed.boost_max_steering_rad": 0.035,
            "speed.boost_max_wall_heading_delta_rad": 0.10,
            "speed.boost_min_wall_clearance_m": 0.55,
            "speed.boost_min_preview_distance_m": 5.0,
            "speed.boost_min_retention_preview_distance_m": 4.0,
            "speed.boost_curvature_exit_persistence_s": 0.10,
            "safety.scan_timeout_s": 0.25,
            "safety.command_rate_hz": 30.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value

        self.fov = float(p("lidar.fov_deg"))
        self.minimum_range = float(p("lidar.min_range"))
        self.maximum_range = float(p("lidar.max_range"))
        self.maximum_gap = float(p("lidar.maximum_gap_m"))
        self.maximum_missing_beams = int(p("lidar.maximum_missing_beams"))
        self.minimum_wall_points = int(p("wall.minimum_points"))
        self.preview_distance = float(p("centerline.preview_distance_m"))
        self.sample_step = float(p("centerline.sample_step_m"))
        self.minimum_centerline_points = int(p("centerline.minimum_points"))
        self.scan_bin_width = float(p("centerline.scan_bin_width_m"))
        self.scan_minimum_points_per_side = int(p("centerline.scan_minimum_points_per_side"))
        self.scan_inner_fraction = float(p("centerline.scan_inner_fraction"))
        self.scan_minimum_span = float(p("centerline.scan_minimum_span_m"))
        self.scan_minimum_width_factor = float(p("centerline.scan_minimum_width_factor"))
        self.scan_maximum_width_factor = float(p("centerline.scan_maximum_width_factor"))
        self.scan_maximum_lateral_jump = float(p("centerline.scan_maximum_lateral_jump_m"))
        self.scan_maximum_heading_change = float(p("centerline.scan_maximum_heading_change_rad"))
        self.scan_maximum_curvature = float(p("centerline.scan_maximum_curvature_inv_m"))
        self.scan_timeout = float(p("safety.scan_timeout_s"))
        self.maximum_steering = float(p("controller.maximum_steering_rad"))

        self.wall_tracker = WallTracker(
            minimum_points=self.minimum_wall_points,
            bin_size_m=float(p("wall.bin_size_m")),
            maximum_residual_m=float(p("wall.maximum_residual_m")),
            anchor_x_m=float(p("wall.anchor_x_m")),
            maximum_forward_m=float(p("wall.maximum_forward_m")),
            initial_track_width_m=float(p("wall.initial_track_width_m")),
            minimum_track_width_m=float(p("wall.minimum_track_width_m")),
            maximum_track_width_m=float(p("wall.maximum_track_width_m")),
            width_filter_alpha=float(p("wall.width_filter_alpha")),
        )
        self.controller = PurePursuitController(
            wheelbase=float(p("controller.wheelbase_m")),
            lookahead=float(p("controller.lookahead_m")),
            maximum_steering=self.maximum_steering,
            cruise_speed=float(p("speed.max_mps")),
            medium_speed=float(p("speed.medium_mps")),
            corner_speed=float(p("speed.corner_mps")),
            medium_steering=float(p("controller.medium_steering_rad")),
            corner_steering=float(p("controller.corner_steering_rad")),
            steering_filter_alpha=float(p("controller.steering_filter_alpha")),
            steering_rate_limit=float(p("controller.steering_rate_limit_rad_s")),
        )
        self.speed_planner = PreviewSpeedPlanner(
            continuous_limits=bool(p("speed.continuous_limits")),
            reaction_time=float(p("speed.reaction_time_s")),
            wheelbase=float(p("controller.wheelbase_m")),
            max_speed=float(p("speed.max_mps")),
            medium_speed=float(p("speed.medium_mps")),
            corner_speed=float(p("speed.corner_mps")),
            preview_distance=float(p("speed.preview_distance_m")),
            curvature_low=float(p("speed.curvature_low")),
            curvature_medium=float(p("speed.curvature_medium")),
            curvature_high=float(p("speed.curvature_high")),
            max_accel=float(p("speed.maximum_accel_mps2")),
            max_decel=float(p("speed.maximum_decel_mps2")),
            braking_margin=float(p("speed.braking_margin_m")),
            lateral_accel_limit=float(p("speed.lateral_accel_limit_mps2")),
            corridor_half_width=float(p("speed.corridor_half_width_m")),
            emergency_clearance=float(p("speed.emergency_clearance_m")),
            medium_steering=float(p("controller.medium_steering_rad")),
            corner_steering=float(p("controller.corner_steering_rad")),
            boost_entry_time=float(p("speed.boost_entry_time_s")),
            boost_max_steering=float(p("speed.boost_max_steering_rad")),
            boost_max_wall_heading_delta=float(p("speed.boost_max_wall_heading_delta_rad")),
            boost_min_wall_clearance=float(p("speed.boost_min_wall_clearance_m")),
            boost_min_preview_distance=float(p("speed.boost_min_preview_distance_m")),
            boost_min_retention_preview_distance=float(p("speed.boost_min_retention_preview_distance_m")),
            boost_curvature_exit_persistence=float(p("speed.boost_curvature_exit_persistence_s")),
        )

        self.control_callbacks = MutuallyExclusiveCallbackGroup()
        start_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.start_interlock = StartInterlock(
            self, str(p("race_start_topic")), self._start_callback, self.control_callbacks
        )
        self.create_subscription(
            LaserScan, str(p("lidar_topic")), self._fresh_scan_callback,
            QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,
                       reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.control_callbacks
        )
        self.speed_publisher = self.create_publisher(
            Float32, str(p("speed_topic")), 10
        )
        self.steering_publisher = self.create_publisher(
            Float32, str(p("steering_topic")), 10
        )

        self.started = False
        self.last_scan_time: float | None = None
        self.last_processing_time: float | None = None
        self.desired_speed = 0.0
        self.desired_steering = 0.0
        self.command_valid = False
        self.state = "STOP_GATE"
        self.last_log_time = 0.0
        self.scan_count = 0
        self.first_scan_wall_time: float | None = None
        self.first_scan_sim_time: float | None = None
        self.last_scan_sim_time: float | None = None
        self.maximum_scan_wall_interval = 0.0
        self.maximum_scan_sim_interval = 0.0
        self.wall_tracking_failures = 0
        self.scan_timeout_events = 0
        self.steering_saturation_events = 0
        self.centerline_source_counts: Counter[str] = Counter()
        self.target_y_count = 0
        self.target_y_sum = 0.0
        self.target_y_min = math.inf
        self.target_y_max = -math.inf
        self.target_sign_flips = 0
        self.previous_target_sign = 0
        self.raw_steering_min = math.inf
        self.raw_steering_max = -math.inf
        self.filtered_steering_min = math.inf
        self.filtered_steering_max = -math.inf
        self.commanded_speed_max = 0.0
        rate = max(10.0, float(p("safety.command_rate_hz")))
        self.create_timer(1.0 / rate, self._publish_command, callback_group=self.control_callbacks)

    
    @staticmethod
    def _format_points(points) -> str:
        return "[" + ",".join(f"({point.x:.2f},{point.y:+.2f})" for point in points[:5]) + "]"

    
    @staticmethod
    def _format_bins(bins) -> str:
        def value(number: float | None) -> str:
            return "na" if number is None else f"{number:+.2f}"

        accepted = [item for item in bins if item.accepted][:5]
        return "[" + ",".join(
            f"{item.x:.2f}:{value(item.left_y)}/{value(item.right_y)}/{value(item.width_m)}"
            for item in accepted
        ) + "]"

    def _stop(self, state: str) -> None:
        self.desired_speed = 0.0
        self.desired_steering = 0.0
        self.command_valid = False
        self.state = state

    def _start_callback(self, message: Bool) -> None:
        self.started = bool(message.data)
        if not self.started:
            self.controller.reset()
            self.speed_planner.reset()
            self._stop("STOP_GATE")
            self._publish_command()
        else:
            # A newly opened gate still waits for a fresh, valid LiDAR scan.
            self.scan_count = 0
            self.first_scan_wall_time = None
            self.first_scan_sim_time = None
            self.last_scan_sim_time = None
            self.maximum_scan_wall_interval = 0.0
            self.maximum_scan_sim_interval = 0.0
            self.wall_tracking_failures = 0
            self.scan_timeout_events = 0
            self.steering_saturation_events = 0
            self.centerline_source_counts.clear()
            self.target_y_count = 0
            self.target_y_sum = 0.0
            self.target_y_min = math.inf
            self.target_y_max = -math.inf
            self.target_sign_flips = 0
            self.previous_target_sign = 0
            self.raw_steering_min = math.inf
            self.raw_steering_max = -math.inf
            self.filtered_steering_min = math.inf
            self.filtered_steering_max = -math.inf
            self.commanded_speed_max = 0.0
            self.last_scan_time = None
            self._stop("WAIT_SCAN")

    def _fresh_scan_callback(self, scan: LaserScan) -> None:
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if age < -.10 or age > self.scan_timeout or stamp <= getattr(self, '_accepted_scan_stamp', -math.inf):
            self._stop("STALE_SCAN")
            self._publish_command()
            return
        self._accepted_scan_stamp = stamp
        self._scan_callback(scan)

    def _scan_callback(self, scan: LaserScan) -> None:
        now = time.monotonic()
        dt = 1.0 / 30.0 if self.last_processing_time is None else now - self.last_processing_time
        if self.last_processing_time is not None:
            self.maximum_scan_wall_interval = max(self.maximum_scan_wall_interval, dt)
        self.last_processing_time = now
        self.last_scan_time = now
        scan_sim_time = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        if self.first_scan_wall_time is None:
            self.first_scan_wall_time = now
            self.first_scan_sim_time = scan_sim_time
        if self.last_scan_sim_time is not None and scan_sim_time > self.last_scan_sim_time:
            self.maximum_scan_sim_interval = max(
                self.maximum_scan_sim_interval, scan_sim_time - self.last_scan_sim_time
            )
        self.last_scan_sim_time = scan_sim_time
        self.scan_count += 1

        scan_points = scan_to_points(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self.fov,
            self.minimum_range,
            self.maximum_range,
        )
        segments = continuous_segments(
            scan_points,
            self.maximum_gap,
            self.maximum_missing_beams,
            max(5, self.minimum_wall_points),
        )
        observation = self.wall_tracker.update(segments)
        centerline = build_centerline(
            observation, self.preview_distance, self.sample_step
        )
        scan_centerline = build_scan_centerline(
            scan_points,
            self.preview_distance,
            self.scan_bin_width,
            observation.track_width_m,
            self.scan_minimum_points_per_side,
            self.scan_inner_fraction,
            self.scan_minimum_width_factor,
            self.scan_maximum_width_factor,
            self.scan_maximum_lateral_jump,
            self.scan_maximum_heading_change,
            self.scan_maximum_curvature,
        )
        centerline = choose_centerline(
            scan_centerline,
            centerline,
            self.minimum_centerline_points,
            self.scan_minimum_span,
        )
        wall_count = int(observation.left is not None) + int(observation.right is not None)

        if not self.started:
            self._stop("STOP_GATE")
            return
        if not perception_is_valid(
            len(segments),
            wall_count,
            len(centerline.points),
            self.minimum_centerline_points,
            centerline.source == "SCAN_MIDPOINT",
        ):
            self.wall_tracking_failures += 1
            self.controller.reset()
            self.speed_planner.reset()
            self._stop("NO_WALL" if wall_count == 0 else "NO_CENTERLINE")
            return

        command = self.controller.compute(centerline.points, max(0.01, min(0.25, dt)))
        if command is None:
            self.speed_planner.reset()
            self._stop("NO_TARGET")
            return
        plan = self.speed_planner.plan(
            centerline.points,
            [item.point for item in scan_points],
            command.raw_steering_angle,
            command.steering_angle,
            max(0.01, min(0.25, dt)),
        )
        if not plan.valid:
            self._stop("SPEED_PLAN_INVALID")
            return

        self.desired_speed = plan.final_speed
        # Controller math is +left. URRC external contract is +right/-left.
        self.desired_steering = -command.steering_angle
        if abs(command.steering_angle) >= 0.95 * self.maximum_steering:
            self.steering_saturation_events += 1
        self.command_valid = True
        self.state = f"DRIVE_{centerline.source}"
        self.centerline_source_counts[centerline.source] += 1
        self.target_y_count += 1
        self.target_y_sum += command.target.y
        self.target_y_min = min(self.target_y_min, command.target.y)
        self.target_y_max = max(self.target_y_max, command.target.y)
        self.raw_steering_min = min(self.raw_steering_min, command.raw_steering_angle)
        self.raw_steering_max = max(self.raw_steering_max, command.raw_steering_angle)
        self.filtered_steering_min = min(self.filtered_steering_min, command.steering_angle)
        self.filtered_steering_max = max(self.filtered_steering_max, command.steering_angle)
        self.commanded_speed_max = max(self.commanded_speed_max, plan.final_speed)
        target_sign = 1 if command.target.y > 0.05 else (-1 if command.target.y < -0.05 else 0)
        if (
            target_sign
            and self.previous_target_sign
            and target_sign != self.previous_target_sign
        ):
            self.target_sign_flips += 1
        if target_sign:
            self.previous_target_sign = target_sign

        if now - self.last_log_time >= 2.0:
            self.last_log_time = now
            wall_elapsed = max(1e-6, now - (self.first_scan_wall_time or now))
            sim_elapsed = max(1e-6, scan_sim_time - (self.first_scan_sim_time or scan_sim_time))
            samples = max(0, self.scan_count - 1)
            wall_hz = samples / wall_elapsed
            sim_hz = samples / sim_elapsed
            rtf = sim_elapsed / wall_elapsed
            target_mean = self.target_y_sum / max(1, self.target_y_count)
            accepted_bins = sum(item.accepted for item in scan_centerline.bins)
            rejected = ",".join(
                f"{reason}:{count}" for reason, count in scan_centerline.rejection_counts
            ) or "none"
            source_counts = ",".join(
                f"{source}:{count}({100.0 * count / max(1, self.target_y_count):.1f}%)"
                for source, count in sorted(self.centerline_source_counts.items())
            )
            bearing = math.atan2(command.target.y, command.target.x)
            self.get_logger().info(
                f"{self.state} source={centerline.source} walls={wall_count} "
                f"center={len(centerline.points)} scan_raw={scan_centerline.raw_point_count} "
                f"scan_mid={len(scan_centerline.points)} "
                f"scan_bins={accepted_bins}/{len(scan_centerline.bins)} "
                f"scan_span={scan_centerline.forward_span_m:.2f}m "
                f"mid5={self._format_points(scan_centerline.points)} "
                f"bin5=L/R/W{self._format_bins(scan_centerline.bins)} reject={{{rejected}}} "
                f"width={centerline.track_width_m:.2f}m speed={plan.final_speed:.2f}m/s "
                f"target=({command.target.x:.2f},{command.target.y:+.2f}) "
                f"bearing={bearing:+.3f}rad raw={command.raw_steering_angle:+.3f}rad "
                f"filtered={command.steering_angle:+.3f}rad "
                f"raw_range=[{self.raw_steering_min:+.3f},{self.raw_steering_max:+.3f}] "
                f"filtered_range=[{self.filtered_steering_min:+.3f},{self.filtered_steering_max:+.3f}] "
                f"speed_max={self.commanded_speed_max:.2f}m/s "
                f"target_y=n{self.target_y_count}/mean{target_mean:+.3f}/"
                f"range[{self.target_y_min:+.3f},{self.target_y_max:+.3f}]/"
                f"flips{self.target_sign_flips} sources={{{source_counts}}} "
                f"scan_sim={sim_hz:.1f}Hz scan_wall={wall_hz:.1f}Hz "
                f"scan_gap_sim={self.maximum_scan_sim_interval:.3f}s "
                f"scan_gap_wall={self.maximum_scan_wall_interval:.3f}s rtf={rtf:.2f} "
                f"wall_fail={self.wall_tracking_failures} timeout={self.scan_timeout_events} "
                f"steer_sat={self.steering_saturation_events}"
            )

    def _publish_command(self) -> None:
        now = time.monotonic()
        allowed = (
            self.started
            and self.command_valid
            and scan_is_fresh(self.last_scan_time, now, self.scan_timeout)
        )
        speed = self.desired_speed if allowed else 0.0
        steering = self.desired_steering if allowed else 0.0
        if self.started and not allowed and self.state.startswith("DRIVE"):
            self.scan_timeout_events += 1
            self.state = "SCAN_TIMEOUT"
            self.speed_planner.reset()
        speed_message = Float32()
        speed_message.data = float(speed)
        steering_message = Float32()
        steering_message.data = float(steering)
        if now - getattr(self, '_gate_log_time', 0.0) > 1.0:
            age = None if self.last_scan_time is None else now-self.last_scan_time
            self.get_logger().info(f'command state={self.state} started={self.started} valid={self.command_valid} scan_age={age} speed={speed:.3f} steer={steering:.3f}')
            self._gate_log_time = now
        self.speed_publisher.publish(speed_message)
        self.steering_publisher.publish(steering_message)
        self._last_published_command = (speed, steering)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
