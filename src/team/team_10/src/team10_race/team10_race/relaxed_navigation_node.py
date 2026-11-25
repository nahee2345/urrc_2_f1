"""ROS node variant using LiDAR-only relaxed-centerline control."""

from __future__ import annotations

import math
import json
import os
import time
import copy
from collections import Counter, deque
from dataclasses import replace

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .centerline import build_centerline, build_scan_centerline, choose_centerline
from .lidar_processing import continuous_segments, scan_to_points
from .navigation_node import NavigationNode as BaseNavigationNode
from .opponent import (
    OpponentDetector,
    OpponentTracker,
    TrackingMotionTimeline,
    RaceStrategy,
    filter_selected_pass_obstacle,
    fuse_opponent_candidates,
    full_scan_points,
    mask_confirmed_opponents,
    offset_path,
    translate_scan_points,
    transform_scan_points,
    translate_wall_observation,
)
from .relaxed_controller import RelaxedPurePursuitController
from .safety import perception_is_valid
from .wall_arc import build_wall_arc_centerline
from .footprint_trajectory import plan_footprint_trajectory
from .speed_audit import SpeedAudit
from .path_safety import verify_path, extend_visible_path, near_path_agreement
from .soft_centering import SoftCenteringPlanner, measure_corridor


class NavigationNode(BaseNavigationNode):
    def __init__(self) -> None:
        super().__init__()
        self.speed_audit = SpeedAudit('navigation')
        self._audit_plan = None
        self._plan_mono = None
        self._plan_stamp = None
        self._footprint_mode=False
        self._footprint_exit_scans=0
        defaults = {
            "soft_center.vehicle_half_width_m": 0.116,
            "controller.lookahead_speed_gain_s": 0.,
            "controller.maximum_lookahead_m": 2.6,
            "soft_center.wall_clearance_margin_m": 0.35,
            "soft_center.warning_clearance_m": 0.50,
            "soft_center.critical_clearance_m": 0.25,
            "soft_center.deadband_enter_m": 0.30,
            "soft_center.deadband_exit_m": 0.20,
            "soft_center.full_correction_error_m": 0.55,
            "soft_center.recenter_length_m": 8.0,
            "soft_center.recenter_speed_gain_s": 1.0,
            "soft_center.warning_recenter_length_m": 3.0,
            "soft_center.critical_recenter_length_m": 0.90,
            "soft_center.max_target_lateral_rate_mps": 0.60,
            "opponent.mask_longitudinal_margin_m": 0.20,
            "opponent.mask_lateral_margin_m": 0.14,
            "opponent.nominal_length_m": 0.55,
            "opponent.nominal_width_m": 0.25,
            "opponent.wall_clearance_m": 0.25,
            "opponent.clearance_m": 0.33,
            "opponent.pass_transition_length_m": 3.0,
            "opponent_lidar_topic": "/team_10/opponent_scan_10",
            "opponent.primary_x_m": 0.080,
            "opponent.secondary_x_m": 0.200,
            "opponent.secondary_max_age_s": 0.15,
            "rear_opponent_lidar_topic": "/team_10/rear_opponent_scan_10",
            "opponent.rear_x_m": -0.245,
            "opponent.rear_yaw_rad": math.pi,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.vehicle_half_width = float(p("soft_center.vehicle_half_width_m"))
        self.wall_clearance_margin = float(p("soft_center.wall_clearance_margin_m"))
        self.warning_clearance = float(p("soft_center.warning_clearance_m"))
        self.critical_clearance = float(p("soft_center.critical_clearance_m"))
        self.opponent_mask_longitudinal_margin = float(
            p("opponent.mask_longitudinal_margin_m")
        )
        self.opponent_mask_lateral_margin = float(
            p("opponent.mask_lateral_margin_m")
        )
        self.opponent_nominal_length = float(p("opponent.nominal_length_m"))
        self.opponent_nominal_width = float(p("opponent.nominal_width_m"))
        self.pass_transition_length = float(p("opponent.pass_transition_length_m"))
        self.primary_lidar_x = float(p("opponent.primary_x_m"))
        self.secondary_lidar_x = float(p("opponent.secondary_x_m"))
        self.secondary_max_age = float(p("opponent.secondary_max_age_s"))
        self.rear_lidar_x = float(p("opponent.rear_x_m"))
        self.rear_lidar_yaw = float(p("opponent.rear_yaw_rad"))
        planner = SoftCenteringPlanner(
            deadband_enter_m=float(p("soft_center.deadband_enter_m")),
            deadband_exit_m=float(p("soft_center.deadband_exit_m")),
            full_correction_error_m=float(p("soft_center.full_correction_error_m")),
            recenter_length_m=float(p("soft_center.recenter_length_m")),
            recenter_speed_gain_s=float(p("soft_center.recenter_speed_gain_s")),
            warning_recenter_length_m=float(p("soft_center.warning_recenter_length_m")),
            critical_recenter_length_m=float(p("soft_center.critical_recenter_length_m")),
            warning_clearance_m=self.warning_clearance,
            critical_clearance_m=self.critical_clearance,
        )
        self.controller = RelaxedPurePursuitController(
            lookahead_speed_gain=float(p("controller.lookahead_speed_gain_s")),
            maximum_lookahead=float(p("controller.maximum_lookahead_m")),
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
            soft_centering_planner=planner,
            target_lateral_rate_limit=float(p("soft_center.max_target_lateral_rate_mps")),
        )
        self.original_target_y_min = math.inf
        self.original_target_y_max = -math.inf
        self.soft_target_y_min = math.inf
        self.soft_target_y_max = -math.inf
        self.steering_sign_flips = 0
        self.previous_steering_sign = 0
        self.minimum_wall_clearance = math.inf
        self.last_speed_state = "RESET"
        self.speed_limiter_counts = Counter()
        self.boost_gate_primary_counts = Counter()
        self.braking_reason_counts = Counter()
        self.boost_exit_counts = Counter()
        self.curvature_run_counts = Counter()
        self.curvature_run_scans = 0
        self.curvature_run_band_mask = 0
        self.opponent_detector = OpponentDetector(
            minimum_points=2,
            allow_rear_candidates=True,
        )
        self.opponent_tracker = OpponentTracker(
            nominal_width_m=self.opponent_nominal_width
        )
        self.opponents = []
        self.pass_candidates = []
        self.opponent_candidate_count = 0
        self.primary_opponent_candidate_count = 0
        self.secondary_opponent_candidate_count = 0
        self.opponent_detection_updates = 0
        self.opponent_detection_hits = 0
        self.opponent_encounter_active = False
        self.opponent_encounter_complete = False
        self.opponent_encounter_updates = 0
        self.opponent_encounter_hits = 0
        self.opponent_elapsed = 0.0
        # Optional validation-only isolation; normal launches leave this unset.
        trial_variant = os.environ.get("HANLA_TRIAL_VARIANT", "current")
        isolation = {
            "delayed-prepare": {"prepare_observation_s": 1.0},
            "unfiltered-return": {"filter_return_offset": False},
            "no-rear": {"rear_awareness_enabled": False},
            "no-recovery": {"safety_recovery_enabled": False},
            "fresh-only": {"rear_awareness_enabled": False,
                           "safety_recovery_enabled": False,
                           "side_occupancy_enabled": False},
            "side-only": {"rear_awareness_enabled": False,
                          "safety_recovery_enabled": False},
            "early-prepare": {"prepare_observation_s": 0.0},
            "early-legacy-margin": {"prepare_observation_s": 0.0},
            "legacy-single": {"prepare_observation_s": 0.0,
                              "safety_recovery_enabled": False},
        }.get(trial_variant, {})
        self.trial_trace_enabled = os.environ.get("HANLA_TRIAL_TRACE") == "1"
        self.race_strategy = RaceStrategy(
            vehicle_half_width_m=self.vehicle_half_width,
            wall_clearance_m=float(p("opponent.wall_clearance_m")),
            opponent_clearance_m=(0.32 if trial_variant in {"early-legacy-margin", "legacy-single"}
                                  else float(p("opponent.clearance_m"))),
            medium_speed_mps=self.speed_planner.medium_speed,
            **isolation,
        )
        self.previous_race_state = self.race_strategy.state
        self.previous_pass_safety_state = self.race_strategy.pass_safety_state
        self.previous_rear_approaching = False
        self.opponent_masked_points = 0
        self.secondary_scan = None
        self.rear_scan = None
        self.rear_scan_wall_time = None
        self.rear_opponent_candidate_count = 0
        self.tracking_last_sim_time = None
        self.tracking_wall_cloud = []
        self.tracking_motion = TrackingMotionTimeline(
            self.speed_planner.max_speed, self.maximum_steering, self.controller.wheelbase)
        self.secondary_history = deque(maxlen=8)
        self.rear_history = deque(maxlen=8)
        self.association_window = deque(maxlen=11)
        self.association_tail = 0
        self.association_last_logged = -math.inf
        self.gap_audit_enabled = os.environ.get('HANLA_GAP_AUDIT') == '1'
        self.gap_scan_arrivals = {source: deque(maxlen=8) for source in ('primary','secondary','rear')}
        self.gap_audit_window = deque(maxlen=11)
        self.gap_audit_active = False
        self.gap_audit_tail = 0
        self.gap_audit_logged = -math.inf
        self.gap_audit_track_id = None
        self.secondary_scan_wall_time: float | None = None
        self.secondary_scan_count = 0
        self.secondary_first_wall_time: float | None = None
        self.secondary_first_sim_time: float | None = None
        self.secondary_last_sim_time: float | None = None
        self.create_subscription(
            LaserScan, str(p("rear_opponent_lidar_topic")),
            self._rear_opponent_scan_callback, qos_profile_sensor_data,
            callback_group=self.control_callbacks,
        )
        self.create_subscription(
            LaserScan,
            str(p("opponent_lidar_topic")),
            self._opponent_scan_callback,
            qos_profile_sensor_data,
            callback_group=self.control_callbacks,
        )

    def _rear_opponent_scan_callback(self, scan: LaserScan) -> None:
        """Opponent-only cache; these points never enter wall fitting."""
        if self.gap_audit_enabled:
            self.gap_scan_arrivals['rear'].append((scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9,time.monotonic()))
        if not self._cache_tracking_scan(self.rear_history, scan):
            return
        self.rear_scan = scan
        self.rear_scan_wall_time = time.monotonic()

    def _opponent_scan_callback(self, scan: LaserScan) -> None:
        """Cache only the opponent-dedicated scan; never touch wall control."""

        now = time.monotonic()
        sim_time = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        if self.gap_audit_enabled:
            self.gap_scan_arrivals['secondary'].append((sim_time,now))
        if not self._cache_tracking_scan(self.secondary_history, scan):
            return
        self.secondary_scan = scan
        self.secondary_scan_wall_time = now
        self.secondary_scan_count += 1
        if self.secondary_first_wall_time is None:
            self.secondary_first_wall_time = now
            self.secondary_first_sim_time = sim_time
        self.secondary_last_sim_time = sim_time

    @staticmethod
    def _cache_tracking_scan(history, scan):
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9
        if history:
            last = history[-1].header.stamp
            if stamp <= last.sec + last.nanosec*1e-9:
                return False
        history.append(scan)
        return True

    @staticmethod
    def _tracking_scan_at(history, timestamp, max_age):
        return next((scan for scan in reversed(history)
                     if 0 <= timestamp-(scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9) <= max_age), None)

    def _start_callback(self, message) -> None:
        super()._start_callback(message)
        if bool(message.data):
            self.original_target_y_min = math.inf
            self.original_target_y_max = -math.inf
            self.soft_target_y_min = math.inf
            self.soft_target_y_max = -math.inf
            self.steering_sign_flips = 0
            self.previous_steering_sign = 0
            self.minimum_wall_clearance = math.inf
            self.last_speed_state = "RESET"
            self.speed_limiter_counts.clear()
            self.boost_gate_primary_counts.clear()
            self.braking_reason_counts.clear()
            self.boost_exit_counts.clear()
            self.curvature_run_counts.clear()
            self.curvature_run_scans = 0
            self.curvature_run_band_mask = 0
            self.race_strategy.reset()
            self.opponent_detection_updates = 0
            self.opponent_detection_hits = 0
            self.opponent_encounter_active = False
            self.opponent_encounter_complete = False
            self.opponent_encounter_updates = 0
            self.opponent_encounter_hits = 0
            self.previous_race_state = self.race_strategy.state
            self.previous_pass_safety_state = self.race_strategy.pass_safety_state
            self.previous_rear_approaching = False
            self.secondary_scan_count = 0
            self.secondary_first_wall_time = None
            self.secondary_first_sim_time = None
            self.secondary_last_sim_time = None
        else:
            # The official sequence publishes false before the lights. Clear
            # old-race tracks there, then let LiDAR confirm a pre-positioned
            # opponent while the car is stationary. Do not discard that
            # confirmation when lights-out publishes true.
            self.opponent_tracker.reset()
            self.opponents = []
            self.pass_candidates = []
            self.opponent_candidate_count = 0
            self.primary_opponent_candidate_count = 0
            self.secondary_opponent_candidate_count = 0
            self.opponent_elapsed = 0.0
            self.tracking_last_sim_time = None
            self.tracking_wall_cloud = []
            self.tracking_motion.reset()
            self.secondary_history.clear()
            self.rear_history.clear()
            self.association_window.clear()
            self.association_tail = 0
            self.association_last_logged = -math.inf
            self.opponent_masked_points = 0
            self.opponent_encounter_active = False
            self.opponent_encounter_complete = False
            self.opponent_encounter_updates = 0
            self.opponent_encounter_hits = 0
            self.previous_race_state = self.race_strategy.state
            self.previous_pass_safety_state = self.race_strategy.pass_safety_state

    @staticmethod
    def _finite_text(value: float) -> str:
        return "na" if not math.isfinite(value) else f"{value:.3f}"


    def _speed_limiter_reason(self, plan, speed_source: str, raw: float, filtered: float) -> str:
        """Classify the active limiter without changing the speed plan."""

        tolerance = 1e-6
        maximum = self.speed_planner.max_speed
        state_ceiling = plan.state_ceiling
        if plan.target_speed >= maximum - tolerance:
            return "MAX_SPEED"
        candidates = (
            ("BRAKE_COMMIT", plan.speed_limit_from_brake_commit),
            ("VISIBILITY", plan.speed_limit_from_visibility),
            ("CORRIDOR", plan.speed_limit_from_corridor),
            ("STEERING", plan.speed_limit_from_steering),
            ("CURVATURE", plan.speed_limit_from_curvature),
        )
        below_state = [
            (name, value) for name, value in candidates
            if value < state_ceiling - tolerance
        ]
        if below_state:
            minimum = min(value for _, value in below_state)
            for name, value in below_state:
                if value <= minimum + tolerance:
                    if name == "CURVATURE" and plan.state == "BRAKE":
                        return "BRAKING"
                    return name
        if speed_source not in {"SCAN_MIDPOINT", "SCAN_PLUS_WALL", "POLY_BOTH"}:
            return "FALLBACK_CONFIDENCE"
        if plan.state == "BRAKE":
            return "BRAKING"
        if plan.state == "CORNER":
            steering = max(abs(raw), abs(filtered))
            if steering >= self.speed_planner.medium_steering:
                return "STEERING"
            return "CURVATURE"
        return "STRAIGHT_GATE"

    def _boost_gate_failure_reason(
        self, plan, speed_source: str, wall_heading_delta: float,
        minimum_wall_clearance: float, raw: float, filtered: float,
    ) -> str | None:
        """Return one exclusive boost-gate failure using explicit precedence."""

        if plan.state == "STRAIGHT_BOOST":
            return None
        if speed_source not in {"SCAN_MIDPOINT", "SCAN_PLUS_WALL", "POLY_BOTH"}:
            return "CONFIDENCE"
        if plan.preview_max_curvature > self.speed_planner.curvature_low:
            return "CURVATURE"
        if max(abs(raw), abs(filtered)) > self.speed_planner.boost_max_steering:
            return "STEERING"
        if (
            not math.isfinite(wall_heading_delta)
            or wall_heading_delta > self.speed_planner.boost_max_wall_heading_delta
        ):
            return "WALL_PARALLEL"
        if minimum_wall_clearance < self.speed_planner.boost_min_wall_clearance:
            return "CLEARANCE"
        if plan.preview_distance < self.speed_planner.boost_min_preview_distance:
            return "PREVIEW_DISTANCE"
        return "ENTRY_DWELL"

    def _braking_reason(self, plan) -> str | None:
        """Separate physical braking need from the conservative BRAKE state ceiling."""

        if plan.state != "BRAKE" or self.speed_planner.continuous_limits:
            return None
        if plan.preview_max_curvature <= 1.20 * self.speed_planner.curvature_low:
            return "PREVIEW_THRESHOLD_EDGE"
        stopping_window = (
            plan.required_braking_distance + self.speed_planner.braking_margin
        )
        if plan.distance_to_curve > stopping_window + 0.25:
            return "EARLY_STATE_CEILING"
        if plan.distance_to_curve <= self.speed_planner.braking_margin:
            return "BRAKING_MARGIN"
        return "PHYSICAL_DISTANCE"

    def _stop(self, state: str) -> None:
        previous = self.state
        super()._stop(state)
        strategy = getattr(self, "race_strategy", None)
        if state != previous and strategy is not None and strategy.state != "CRUISE":
            self.get_logger().warning(
                f"opponent_navigation_stop={state} race_state={strategy.state}"
            )

    def _record_curvature_run(self, plan) -> None:
        """Count scan persistence and distance bands without affecting control."""

        if plan.preview_max_curvature > self.speed_planner.curvature_low:
            self.curvature_run_scans += 1
            for index, value in enumerate(plan.curvature_bands):
                if value > self.speed_planner.curvature_low:
                    self.curvature_run_band_mask |= 1 << index
            return
        if self.curvature_run_scans:
            length = str(self.curvature_run_scans) if self.curvature_run_scans < 4 else "4+"
            self.curvature_run_counts[f"scans_{length}"] += 1
            for index in range(4):
                if self.curvature_run_band_mask & (1 << index):
                    self.curvature_run_counts[f"band_{5 * index}_{5 * (index + 1)}"] += 1
        self.curvature_run_scans = 0
        self.curvature_run_band_mask = 0

    @staticmethod
    def _wall_heading_delta(observation, scan_centerline) -> float:
        left, right = observation.left, observation.right
        if left is not None and right is not None:
            start = max(left.x_min, right.x_min, 0.20)
            stop = min(left.x_max, right.x_max, 10.0)
            if stop > start:
                positions = (start, 0.5 * (start + stop), stop)
                return max(
                    abs(math.atan(left.slope(x)) - math.atan(right.slope(x)))
                    for x in positions
                )
        bins = [
            item for item in scan_centerline.bins
            if item.accepted and item.left_y is not None and item.right_y is not None
        ]
        if len(bins) < 3 or bins[-1].x - bins[0].x < 2.0:
            return math.inf
        mean_x = sum(item.x for item in bins) / len(bins)
        denominator = sum((item.x - mean_x) ** 2 for item in bins)
        if denominator < 1e-6:
            return math.inf
        left_mean = sum(item.left_y for item in bins) / len(bins)
        right_mean = sum(item.right_y for item in bins) / len(bins)
        left_slope = sum(
            (item.x - mean_x) * (item.left_y - left_mean) for item in bins
        ) / denominator
        right_slope = sum(
            (item.x - mean_x) * (item.right_y - right_mean) for item in bins
        ) / denominator
        return abs(math.atan(left_slope) - math.atan(right_slope))

    def _scan_callback(self, scan) -> None:
        begin = time.perf_counter()
        self._audit_plan = None
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9
        receipt_sim = self.get_clock().now().nanoseconds*1e-9
        try:
            self._process_scan(scan)
        finally:
            self._plan_mono = time.monotonic()
            self._plan_stamp = stamp
            if self.speed_audit.enabled:
                self.speed_audit.emit(event='plan', sim=self.get_clock().now().nanoseconds*1e-9,
                    scan_stamp=stamp, sensor_age_receipt_sim=receipt_sim-stamp,
                    sensor_age_done_sim=self.get_clock().now().nanoseconds*1e-9-stamp,
                    processing_wall_s=time.perf_counter()-begin,
                    started=self.started, valid=self.command_valid, state=self.state,
                    navigation_desired=self.desired_speed,
                    steering_contract=self.desired_steering, plan=self._audit_plan)

    def _publish_command(self):
        super()._publish_command()
        if getattr(self, 'speed_audit', None) and self.speed_audit.enabled:
            now = time.monotonic()
            speed,steering = self._last_published_command
            self.speed_audit.emit(event='output', sim=self.get_clock().now().nanoseconds*1e-9,
                speed=speed, steering=steering, state=self.state,
                plan_age_wall=None if self._plan_mono is None else now-self._plan_mono,
                sensor_age_sim=None if self._plan_stamp is None else self.get_clock().now().nanoseconds*1e-9-self._plan_stamp)

    def destroy_node(self):
        if getattr(self, 'speed_audit', None):
            self.speed_audit.close()
        return super().destroy_node()

    def _process_scan(self, scan) -> None:
        now = time.monotonic()
        dt = 1.0 / 30.0 if self.last_processing_time is None else now - self.last_processing_time
        if self.last_processing_time is not None:
            self.maximum_scan_wall_interval = max(self.maximum_scan_wall_interval, dt)
        self.last_processing_time = now
        self.last_scan_time = now
        scan_sim_time = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        sim_dt = (scan_sim_time-self.last_scan_sim_time
                  if self.last_scan_sim_time is not None else 1./30.)
        if self.gap_audit_enabled:
            self.gap_scan_arrivals['primary'].append((scan_sim_time,now))
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
        # First obtain a non-mutating provisional wall/corridor estimate. The
        # wall tracker ranks long outer chains and continuity with the previous
        # frame ahead of compact interior fragments.
        provisional_observation = self.wall_tracker.observe(segments)
        provisional_polynomial = build_centerline(
            provisional_observation, self.preview_distance, self.sample_step
        )

        if (len(provisional_polynomial.points) < self.minimum_centerline_points
                or (provisional_observation.left is None) != (provisional_observation.right is None)):
            raw_provisional = build_wall_arc_centerline(segments, provisional_observation, self.sample_step)
            if len(raw_provisional.points) >= self.minimum_centerline_points:
                provisional_polynomial = raw_provisional

        bounded_dt = max(0.01, min(0.25, dt))
        if self.speed_planner.continuous_limits:
            bounded_dt = max(0.001, min(0.25, sim_dt))
        self.opponent_elapsed += bounded_dt
        if self.scan_count % 3 == 0:
            primary_opponent_points = translate_scan_points(full_scan_points(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                scan.range_min,
                scan.range_max,
                self.minimum_range,
                self.maximum_range,
            ), self.primary_lidar_x)
            base_observation = translate_wall_observation(
                provisional_observation, self.primary_lidar_x
            )
            base_centerline = tuple(
                type(point)(point.x + self.primary_lidar_x, point.y)
                for point in provisional_polynomial.points
            )
            primary_candidates = self.opponent_detector.detect(
                primary_opponent_points,
                base_observation,
                base_centerline,
            )
            primary_candidates = [replace(candidate, timestamp=scan_sim_time,
                                          sources=("primary",))
                                  for candidate in primary_candidates]
            raw_tracking_observations = [("primary",scan_sim_time,primary_opponent_points)]
            self.primary_opponent_candidate_count = len(primary_candidates)
            secondary_candidates = []
            secondary = self._tracking_scan_at(self.secondary_history,scan_sim_time,self.secondary_max_age)
            if secondary is not None:
                secondary_opponent_points = translate_scan_points(full_scan_points(
                    secondary.ranges,
                    secondary.angle_min,
                    secondary.angle_increment,
                    secondary.range_min,
                    secondary.range_max,
                    self.minimum_range,
                    self.maximum_range,
                ), self.secondary_lidar_x)
                secondary_candidates = self.opponent_detector.detect(
                    secondary_opponent_points,
                    base_observation,
                    base_centerline,
                )
                secondary_stamp = secondary.header.stamp.sec + secondary.header.stamp.nanosec * 1e-9
                secondary_candidates = [replace(candidate,timestamp=secondary_stamp,
                                                sources=("secondary",))
                                        for candidate in secondary_candidates]
                raw_tracking_observations.append(("secondary",secondary_stamp,secondary_opponent_points))
            self.secondary_opponent_candidate_count = len(secondary_candidates)
            opponent_candidates = fuse_opponent_candidates(
                primary_candidates, secondary_candidates
            )
            rear_candidates = []
            # Rear observations belong to the dedicated mount, including when
            # its stream is temporarily stale (existing tracks then coast).
            opponent_candidates = [c for c in opponent_candidates
                                   if c.position != "REAR"]
            rear = self._tracking_scan_at(self.rear_history,scan_sim_time,self.secondary_max_age)
            if rear is not None:
                rear_points = transform_scan_points(full_scan_points(
                    rear.ranges, rear.angle_min, rear.angle_increment,
                    rear.range_min, rear.range_max,
                    self.minimum_range, self.maximum_range,
                ), self.rear_lidar_x, self.rear_lidar_yaw)
                rear_candidates = self.opponent_detector.detect(
                    rear_points, base_observation, base_centerline,
                    dedicated_rear_sensor=True,
                )
                rear_stamp = rear.header.stamp.sec + rear.header.stamp.nanosec * 1e-9
                rear_candidates = [replace(candidate,timestamp=rear_stamp,sources=("rear",))
                                   for candidate in rear_candidates]
                raw_tracking_observations.append(("rear",rear_stamp,rear_points))
                # The validated rear mount owns rear observations. Do not feed
                # uncorroborated returns from the occluded forward mounts into
                # the tracker as additional physical rear vehicles.
                opponent_candidates = fuse_opponent_candidates(
                    opponent_candidates, rear_candidates
                )
            self.rear_opponent_candidate_count = len(rear_candidates)
            if self.gap_audit_enabled:
                def wall_record(wall):
                    return None if wall is None else dict(side=wall.side,
                        coefficients=wall.coefficients,x_min=wall.x_min,x_max=wall.x_max,
                        point_count=wall.point_count,residual_m=wall.residual_m)
                sensor_audit=dict(sim_time=scan_sim_time,
                    arrivals={source:list(values) for source,values in self.gap_scan_arrivals.items()},
                    observation=dict(left=wall_record(base_observation.left),right=wall_record(base_observation.right),
                                     track_width_m=base_observation.track_width_m),
                    centerline=[(p.x,p.y) for p in base_centerline],
                    sensors={source:dict(timestamp=stamp,candidates=[dict(x=c.x,y=c.y,
                        bearing=c.bearing,point_count=c.point_count,extent=c.extent,
                        classification=c.position,sources=c.sources,beam_indices=c.beam_indices)
                        for c in {'primary':primary_candidates,'secondary':secondary_candidates,'rear':rear_candidates}[source]])
                        for source,stamp,_ in raw_tracking_observations})
            self.pass_candidates = opponent_candidates
            self.opponent_detection_updates += 1
            if opponent_candidates:
                self.opponent_detection_hits += 1
                if not self.opponent_encounter_complete:
                    self.opponent_encounter_active = True
            if self.opponent_encounter_active:
                self.opponent_encounter_updates += 1
                if opponent_candidates:
                    self.opponent_encounter_hits += 1
            self.opponent_candidate_count = len(opponent_candidates)
            # Tracker time is acquisition time, independent of simulator RTF.
            tracking_dt = (scan_sim_time-self.tracking_last_sim_time
                           if self.tracking_last_sim_time is not None else .1)
            wall_cloud = [type(p.point)(p.point.x+self.primary_lidar_x,p.point.y)
                          for chain in segments
                          if math.hypot(max(p.point.x for p in chain)-min(p.point.x for p in chain),
                                        max(p.point.y for p in chain)-min(p.point.y for p in chain)) > .75
                          for p in chain]
            previous_cloud = self.tracking_motion.cloud
            motion = self.tracking_motion.update(
                wall_cloud,scan_sim_time,
                self.desired_speed,
                -self.desired_speed*math.tan(self.desired_steering)/self.controller.wheelbase,
            )
            if motion is not None:
                # Navigation keeps its original detector candidates. Only the
                # tracker receives individually time-aligned sensor facets.
                tracking_candidates = [c for c in primary_candidates+secondary_candidates
                                       if c.position != 'REAR'] + rear_candidates
                self.opponents = self.opponent_tracker.update(
                    tracking_candidates,tracking_dt,timestamp=scan_sim_time,
                    ego_motion=motion,raw_observations=raw_tracking_observations,
                    motion_timeline=self.tracking_motion,
                )
                self.tracking_last_sim_time = scan_sim_time
                self.tracking_wall_cloud = wall_cloud
            if self.gap_audit_enabled:
                self.gap_audit_window.append(sensor_audit)
                lost=next((t for t in self.opponent_tracker._tracks.values()
                           if t.missed==1 and t.hits>=self.opponent_tracker.confirmation_hits and t.plausible and t.x<0),None)
                recovered=any(t.missed==0 and t.hits>=self.opponent_tracker.confirmation_hits and t.plausible and t.x<0
                              and (t.track_id==self.gap_audit_track_id or t.hits<=6)
                              for t in self.opponent_tracker._tracks.values())
                if lost and not self.gap_audit_active:
                    self.gap_audit_active=True
                    self.gap_audit_track_id=lost.track_id
                    for record in self.gap_audit_window:
                        if record['sim_time']>self.gap_audit_logged:
                            self.get_logger().info('opponent_sensor_audit='+json.dumps(record))
                            self.gap_audit_logged=record['sim_time']
                if self.gap_audit_active:
                    if scan_sim_time>self.gap_audit_logged:
                        self.get_logger().info('opponent_sensor_audit='+json.dumps(sensor_audit))
                        self.gap_audit_logged=scan_sim_time
                    if recovered:self.gap_audit_active=False;self.gap_audit_tail=10
                elif self.gap_audit_tail:
                    self.get_logger().info('opponent_sensor_audit='+json.dumps(sensor_audit))
                    self.gap_audit_logged=scan_sim_time;self.gap_audit_tail-=1
            if self.trial_trace_enabled:
                association=dict(self.opponent_tracker.diagnostics)
                association['sim_time']=scan_sim_time
                association['ego_estimate']=dict(self.tracking_motion.diagnostics)
                association['sensor_timestamps']={source:stamp for source,stamp,_ in raw_tracking_observations}
                association['tracks']=[dict(id=t.track_id,x=t.x,y=t.y,
                                             classification=self.opponent_tracker._position(t.x,t.y),
                                             hits=t.hits,missed=t.missed,plausible=t.plausible)
                                       for t in self.opponent_tracker._tracks.values()]
                trigger = bool(self.tracking_motion.diagnostics.get('rejected_reason')) or any(
                    c['reason']=='new_no_admissible_unconsumed_edge' and c['x']<0
                    and any(p['hits']>=self.opponent_tracker.confirmation_hits and p['x']<0 for p in association['predictions'])
                    for c in association['candidates']) or any(
                    t['missed']==1 and t['plausible'] and t['x']<0 for t in association['tracks'])
                if self.tracking_motion.diagnostics.get('rejected_reason'):
                    association['registration_clouds']={
                        'previous':[(p.x,p.y) for p in previous_cloud],
                        'current':[(p.x,p.y) for p in wall_cloud]}
                if trigger:
                    association['raw_tracking_scans']=[dict(source=source,timestamp=stamp,
                        points=[(p.beam_index,p.range,p.point.x,p.point.y) for p in points])
                        for source,stamp,points in raw_tracking_observations]
                self.association_window.append(association)
                if trigger:
                    self.association_tail=10
                    for record in self.association_window:
                        if record['sim_time']>self.association_last_logged:
                            self.get_logger().info("opponent_association_trace="+json.dumps(record))
                            self.association_last_logged=record['sim_time']
                elif self.association_tail:
                    self.get_logger().info("opponent_association_trace="+json.dumps(association))
                    self.association_last_logged=scan_sim_time
                    self.association_tail-=1
            self.opponent_elapsed = 0.0

        opponents = self.opponents
        final_scan_points, masked_beams = mask_confirmed_opponents(
            scan_points,
            opponents,
            longitudinal_margin_m=self.opponent_mask_longitudinal_margin,
            lateral_margin_m=self.opponent_mask_lateral_margin,
            nominal_length_m=self.opponent_nominal_length,
            nominal_width_m=self.opponent_nominal_width,
            sensor_x_m=self.primary_lidar_x,
        )
        self.opponent_masked_points = len(masked_beams)
        if opponents:
            final_segments = continuous_segments(
                final_scan_points,
                self.maximum_gap,
                self.maximum_missing_beams,
                max(5, self.minimum_wall_points),
            )
        else:
            # Preserve the exact solo input objects and update sequence.
            final_segments = segments

        observation = self.wall_tracker.update(final_segments)
        polynomial = build_centerline(observation, self.preview_distance, self.sample_step)
        scan_centerline = build_scan_centerline(
            final_scan_points,
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
            polynomial,
            self.minimum_centerline_points,
            self.scan_minimum_span,
        )
        if (len(centerline.points) < self.minimum_centerline_points
                or (centerline.source.startswith("POLY_")
                    and (observation.left is None) != (observation.right is None))):
            # Use all current returns for arc clearance: a tracked object must
            # never remove an obstacle from the geometry's free-space check.
            arc = build_wall_arc_centerline(segments, observation, self.sample_step)
            if len(arc.points) >= self.minimum_centerline_points:
                centerline = arc

        corridor = measure_corridor(
            observation,
            scan_centerline,
            self.vehicle_half_width,
            self.wall_clearance_margin,
            self.warning_clearance,
            self.critical_clearance,
        )
        trajectory = None
        reliable_straight=(centerline.source=='SCAN_MIDPOINT'
            and centerline.forward_span_m>=4.
            and corridor.wall_safety_state=='NORMAL'
            and min(corridor.left_clearance,corridor.right_clearance)>.50
            and abs(self.controller._steering)<.08)
        self._footprint_exit_scans=(self._footprint_exit_scans+1 if reliable_straight else 0)
        if self._footprint_exit_scans>=5:self._footprint_mode=False
        if not self.started:
            self._footprint_mode=False
            self._footprint_exit_scans=0
        if self.started and (self._footprint_mode
                or centerline.source.startswith('RAW_')
                or len(centerline.points)<self.minimum_centerline_points
                or (observation.left is None) != (observation.right is None)):
            self._footprint_mode=True
            trajectory=plan_footprint_trajectory(scan,segments,observation,
                self.controller._steering,self.speed_planner.speed,
                maximum_steering=self.maximum_steering,
                steering_rate=self.controller.steering_rate_limit)
            if trajectory is not None:
                centerline=trajectory.centerline
            else:
                # Never keep an unverified raw path merely to retain movement.
                centerline=replace(centerline,points=(),source='NO_SAFE_TRAJECTORY')
        wall_count = int(observation.left is not None) + int(observation.right is not None)
        # A measured arc is a wall estimate even when its near-vertical tangent
        # makes the y(x) polynomial ill-conditioned. It carries current raw support.
        if centerline.source.startswith("RAW_") and centerline.raw_point_count >= self.minimum_wall_points:
            wall_count = max(1, wall_count)

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

        race_decision = self.race_strategy.update(
            opponents,
            corridor,
            self.speed_planner.speed,
            bounded_dt,
            pass_candidates=self.pass_candidates,
        )
        rear_approaching = (
            race_decision.closest_rear_distance < 6.0
            and race_decision.rear_closing_rate > 0.25
        )
        if rear_approaching != self.previous_rear_approaching:
            self.get_logger().info(
                f"rear_awareness_transition={self.previous_rear_approaching}"
                f"->{rear_approaching}"
                f" rear_distance={self._finite_text(race_decision.closest_rear_distance)}"
                f" rear_closing_rate={race_decision.rear_closing_rate:+.3f}"
                f" desired_lateral_offset={race_decision.desired_lateral_offset:+.3f}"
                f" race_state={race_decision.race_state}"
            )
            self.previous_rear_approaching = rear_approaching
        if (
            self.opponent_encounter_active
            and (
                race_decision.race_state == "RETURN"
                or any(
                    item.track_id == self.race_strategy.pass_track_id and item.x < -0.30
                    for item in opponents
                )
            )
        ):
            self.opponent_encounter_active = False
            self.opponent_encounter_complete = True
        if race_decision.race_state != self.previous_race_state:
            self.get_logger().info(
                f"race_state_transition={self.previous_race_state}"
                f"->{race_decision.race_state}"
                f" selected_pass_side={race_decision.selected_pass_side}"
                f" opponent_relative_x="
                f"{self._finite_text(race_decision.primary_opponent_x)}"
                f" opponent_relative_y="
                f"{self._finite_text(race_decision.primary_opponent_y)}"
            )
            self.previous_race_state = race_decision.race_state
        if race_decision.pass_safety_state != self.previous_pass_safety_state:
            self.get_logger().info(
                f"pass_safety_transition={self.previous_pass_safety_state}"
                f"->{race_decision.pass_safety_state}"
                f" race_state={race_decision.race_state}"
                f" current_wall_clearance="
                f"{self._finite_text(race_decision.current_wall_clearance)}"
                f" current_opponent_clearance="
                f"{self._finite_text(race_decision.current_opponent_clearance)}"
                f" predicted_pass_clearance="
                f"{self._finite_text(race_decision.predicted_pass_clearance)}"
                f" remaining_pass_gap="
                f"{self._finite_text(race_decision.remaining_pass_gap)}"
            )
            self.previous_pass_safety_state = race_decision.pass_safety_state
        active_pass = race_decision.race_state in {"PASS_LEFT", "PASS_RIGHT"}
        hold_pass_line = (
            active_pass
            and race_decision.pass_safety_state in {"WARNING", "HOLD"}
        )
        race_centerline = offset_path(
            centerline.points,
            race_decision.desired_lateral_offset,
            transition_length_m=self.pass_transition_length,
            initial_lateral_offset_m=(race_decision.desired_lateral_offset
                                      if hold_pass_line else 0.0),
        )
        pass_corridor = corridor
        if hold_pass_line:
            # Preserve real wall safety, but normal centering must refer to
            # the chosen PASS line instead of pulling across the opponent.
            pass_corridor = replace(corridor, vehicle_lateral_error=(
                corridor.corridor_center_y + race_decision.desired_lateral_offset))
        command = (self.controller.follow_trajectory(trajectory,pass_corridor,bounded_dt)
                   if trajectory is not None else self.controller.compute(
            race_centerline,
            pass_corridor,
            bounded_dt,
            self.speed_planner.speed,
            follow_observed_path=centerline.source.startswith("RAW_"),
        ))
        if command is None:
            self.speed_planner.reset()
            self._stop("NO_TARGET")
            return
        known_clearances = [
            value for value in (corridor.left_clearance, corridor.right_clearance)
            if math.isfinite(value)
        ]
        instantaneous_clearance = min(known_clearances, default=0.0)
        wall_heading_delta = self._wall_heading_delta(observation, scan_centerline)
        speed_path = command.soft_path
        speed_source = centerline.source
        if (
            not centerline.source.startswith("RAW_")
            and
            polynomial.forward_span_m > centerline.forward_span_m
            and (
                polynomial.source == "POLY_BOTH"
                or (centerline.source == "SCAN_MIDPOINT" and math.isfinite(wall_heading_delta))
            )
        ):
            speed_path = polynomial.points
            speed_source = (
                polynomial.source if polynomial.source == "POLY_BOTH" else "SCAN_PLUS_WALL"
            )
        speed_obstacles = [item.point for item in scan_points]
        body_safety = None
        extension_curvature = None
        extension_diagnostics = {}
        if self.speed_planner.continuous_limits and trajectory is None:
            checked_path = command.soft_path
            if polynomial.source=='POLY_BOTH' and corridor.wall_safety_state=='NORMAL':
                # Two independent current wall fits avoid the staircase
                # curvature of sparse midpoint bins. Accept only a fit that
                # agrees with the actual near control path, then certify it.
                smooth = copy.copy(self.controller.soft_centering_planner).build(
                    polynomial.points,pass_corridor,self.speed_planner.speed).points
                if near_path_agreement(checked_path,smooth):
                    checked_path = smooth
                    speed_source = 'POLY_BOTH'
            if (speed_source in {'SCAN_MIDPOINT','SCAN_PLUS_WALL','POLY_BOTH'}
                    and math.isfinite(wall_heading_delta) and wall_heading_delta<=.10
                    and instantaneous_clearance>=.55):
                checked_path,body_safety,extension_curvature = extend_visible_path(
                    scan,checked_path,self.speed_planner.preview_distance,
                    required_length=(self.speed_planner.emergency_clearance
                        +self.speed_planner.reaction_time*self.speed_planner.max_speed
                        +self.speed_planner.max_speed**2/(2*self.speed_planner.max_decel)),
                    diagnostics=extension_diagnostics,
                    near_distance=max(3.,self.controller.current_lookahead+.30))
            if body_safety is None:
                body_safety = verify_path(scan,checked_path)
            # Curvature preview and the full-body stopping horizon refer to
            # this same current-scan path. Steering still follows near geometry.
            speed_path = checked_path
        if race_decision.race_state in {"PASS_LEFT", "PASS_RIGHT"}:
            passed_opponent = next(
                (
                    item for item in opponents
                    if item.track_id == self.race_strategy.pass_track_id
                ),
                None,
            )
            speed_obstacles = list(filter_selected_pass_obstacle(
                speed_obstacles, passed_opponent, sensor_x_m=self.primary_lidar_x
            ))
        plan = self.speed_planner.plan(
            speed_path,
            speed_obstacles,
            command.raw_steering_angle,
            command.steering_angle,
            bounded_dt,
            centerline_source=speed_source,
            wall_heading_delta=wall_heading_delta,
            minimum_wall_clearance=instantaneous_clearance,
            swept_free_distance_m=(trajectory.free_distance if trajectory is not None else
                (None if body_safety is None else body_safety.distance)),
        )
        if trajectory is not None and now-getattr(self,'_trajectory_log_time',0.)>1.:
            self.get_logger().info('trajectory_scan='+json.dumps(dict(sim=scan_sim_time,
                steering=trajectory.steering,free_distance=trajectory.free_distance,
                minimum_clearance=trajectory.minimum_clearance,margin=.08,
                support=centerline.raw_point_count,points=[(p.x,p.y) for p in centerline.points],
                corridor_limit=plan.speed_limit_from_corridor)))
            self._trajectory_log_time=now
        if not plan.valid:
            self._stop("SPEED_PLAN_INVALID")
            return

        limiter_reason = self._speed_limiter_reason(
            plan, speed_source, command.raw_steering_angle, command.steering_angle
        )
        self.speed_limiter_counts[limiter_reason] += 1
        boost_gate_reason = self._boost_gate_failure_reason(
            plan, speed_source, wall_heading_delta, instantaneous_clearance,
            command.raw_steering_angle, command.steering_angle,
        )
        if boost_gate_reason is not None:
            self.boost_gate_primary_counts[boost_gate_reason] += 1
        braking_reason = self._braking_reason(plan)
        if braking_reason is not None:
            self.braking_reason_counts[braking_reason] += 1
        if self.last_speed_state == "STRAIGHT_BOOST" and plan.state != "STRAIGHT_BOOST":
            self.boost_exit_counts[boost_gate_reason or plan.state] += 1
        self.last_speed_state = plan.state
        self._record_curvature_run(plan)

        self.desired_speed = min(plan.final_speed, race_decision.speed_cap)
        if self.speed_audit.enabled:
            caps = dict(CURVATURE=plan.speed_limit_from_curvature,
                STEERING=plan.speed_limit_from_steering, CORRIDOR=plan.speed_limit_from_corridor,
                STATE=plan.state_ceiling, VISIBILITY=plan.speed_limit_from_visibility,
                BRAKE_COMMIT=plan.speed_limit_from_brake_commit,
                MAX=self.speed_planner.max_speed)
            self._audit_plan = dict(current=plan.current_speed, target=plan.target_speed,
                final=plan.final_speed, unconstrained=self.speed_planner.max_speed,
                candidates=caps, race_cap=race_decision.speed_cap, state=plan.state,
                source=speed_source, preview=plan.preview_distance,
                free_distance=(trajectory.free_distance if trajectory is not None else
                    (None if body_safety is None else body_safety.distance)),
                body_reason=None if body_safety is None else body_safety.reason,
                curvature=plan.preview_max_curvature, curvature_bands=plan.curvature_bands,
                distance_to_curve=plan.distance_to_curve, braking_distance=plan.required_braking_distance,
                clearance=instantaneous_clearance, footprint_clearance=(trajectory.minimum_clearance if trajectory is not None else
                    (None if body_safety is None else body_safety.clearance)),
                raw_steering=command.raw_steering_angle, filtered_steering=command.steering_angle,
                target_xy=(command.target.x,command.target.y), dt=bounded_dt,
                lookahead=self.controller.current_lookahead,
                extension_curvature=extension_curvature,
                extension_diagnostics=extension_diagnostics,
                boost_eligible=plan.boost_eligible, boost_hold=plan.straight_hold_time,
                boost_block=boost_gate_reason, limiter=limiter_reason,
                wall_heading_delta=wall_heading_delta,
                tracking=dict(self.tracking_motion.diagnostics))
        # Internal geometry is +left; the URRC external steering contract is +right.
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
        self.original_target_y_min = min(self.original_target_y_min, command.original_target.y)
        self.original_target_y_max = max(self.original_target_y_max, command.original_target.y)
        self.soft_target_y_min = min(self.soft_target_y_min, command.soft_target_y)
        self.soft_target_y_max = max(self.soft_target_y_max, command.soft_target_y)
        self.raw_steering_min = min(self.raw_steering_min, command.raw_steering_angle)
        self.raw_steering_max = max(self.raw_steering_max, command.raw_steering_angle)
        self.filtered_steering_min = min(self.filtered_steering_min, command.steering_angle)
        self.filtered_steering_max = max(self.filtered_steering_max, command.steering_angle)
        self.commanded_speed_max = max(self.commanded_speed_max, plan.final_speed)
        known_clearances = [
            value for value in (corridor.left_clearance, corridor.right_clearance)
            if math.isfinite(value)
        ]
        if known_clearances:
            self.minimum_wall_clearance = min(self.minimum_wall_clearance, *known_clearances)

        target_sign = 1 if command.target.y > 0.05 else (-1 if command.target.y < -0.05 else 0)
        if target_sign and self.previous_target_sign and target_sign != self.previous_target_sign:
            self.target_sign_flips += 1
        if target_sign:
            self.previous_target_sign = target_sign
        steering_sign = (
            1 if command.steering_angle > 0.02
            else (-1 if command.steering_angle < -0.02 else 0)
        )
        if (
            steering_sign
            and self.previous_steering_sign
            and steering_sign != self.previous_steering_sign
        ):
            self.steering_sign_flips += 1
        if steering_sign:
            self.previous_steering_sign = steering_sign

        if self.trial_trace_enabled and self.started:
            trace = {
                "sim_time": scan_sim_time, "wall_time": time.time(),
                "state": race_decision.race_state,
                "target_id": self.race_strategy.pass_track_id,
                "tracks": [dict(id=t.track_id, x=t.x, y=t.y,
                                vx=t.relative_vx, vy=t.relative_vy,
                                closing_rate=t.closing_rate,
                                missed=t.missed, classification=t.position)
                           for t in opponents],
                "strategy_speed": self.speed_planner.speed,
                "rear_sensor_candidates": self.rear_opponent_candidate_count,
                "race_speed_cap": race_decision.speed_cap,
                "commanded_speed": self.desired_speed,
                "side_wait_track_id": self.race_strategy.side_wait_track_id,
                "desired_offset": race_decision.desired_lateral_offset,
                "target_offset": self.race_strategy.last_target_offset,
                "recovery_correction": self.race_strategy.last_recovery_correction,
                "lateral_rate": self.race_strategy.lateral_rate,
                "left_gap": race_decision.left_pass_clearance,
                "right_gap": race_decision.right_pass_clearance,
                "wall_clearance": race_decision.current_wall_clearance,
                "opponent_clearance": race_decision.current_opponent_clearance,
                "predicted_clearance": race_decision.predicted_pass_clearance,
                "safety": race_decision.pass_safety_state,
                "steering": command.steering_angle,
                "path_mode": "PARALLEL_PASS_HOLD" if hold_pass_line else "LEGACY",
                "path_lateral_error": pass_corridor.vehicle_lateral_error,
                "heading_steering": command.heading_steering_angle,
                "original_target_y": command.original_target.y,
            }
            self.get_logger().info("opponent_trial_trace=" + json.dumps(trace))
        if now - self.last_log_time >= 2.0:
            self.last_log_time = now
            wall_elapsed = max(1e-6, now - (self.first_scan_wall_time or now))
            sim_elapsed = max(1e-6, scan_sim_time - (self.first_scan_sim_time or scan_sim_time))
            samples = max(0, self.scan_count - 1)
            secondary_wall_elapsed = max(
                1e-6, now - (self.secondary_first_wall_time or now)
            )
            secondary_sim_elapsed = max(
                1e-6,
                (self.secondary_last_sim_time or 0.0)
                - (self.secondary_first_sim_time or 0.0),
            )
            secondary_samples = max(0, self.secondary_scan_count - 1)
            accepted_bins = sum(item.accepted for item in scan_centerline.bins)
            rejected = ",".join(
                f"{reason}:{count}" for reason, count in scan_centerline.rejection_counts
            ) or "none"
            sources = ",".join(
                f"{source}:{count}" for source, count in sorted(self.centerline_source_counts.items())
            )
            curvature_bands = ",".join(f"{value:.3f}" for value in plan.curvature_bands)
            limiter_counts = ",".join(
                f"{reason}:{count}"
                for reason, count in sorted(self.speed_limiter_counts.items())
            )
            boost_gate_counts = ",".join(
                f"{reason}:{count}"
                for reason, count in sorted(self.boost_gate_primary_counts.items())
            )
            braking_counts = ",".join(
                f"{reason}:{count}"
                for reason, count in sorted(self.braking_reason_counts.items())
            )
            boost_exit_counts = ",".join(
                f"{reason}:{count}"
                for reason, count in sorted(self.boost_exit_counts.items())
            )
            curvature_run_counts = ",".join(
                f"{reason}:{count}"
                for reason, count in sorted(self.curvature_run_counts.items())
            )
            opponent_tracks = ",".join(
                f"{item.track_id}:{item.x:.2f}:{item.y:+.2f}:"
                f"{item.relative_vx:+.2f}:{item.relative_vy:+.2f}:"
                f"{item.position}:{item.missed}"
                for item in opponents
            )
            boost_gate_text = boost_gate_reason or "ACTIVE"
            braking_text = braking_reason or "NONE"
            state_ceiling = plan.state_ceiling
            self.get_logger().info(
                f"{self.state} source={centerline.source} walls={wall_count} "
                f"center={len(centerline.points)} scan_mid={len(scan_centerline.points)} "
                f"scan_bins={accepted_bins}/{len(scan_centerline.bins)} reject={{{rejected}}} "
                f"left_clearance={self._finite_text(corridor.left_clearance)} "
                f"right_clearance={self._finite_text(corridor.right_clearance)} "
                f"min_clearance={self._finite_text(self.minimum_wall_clearance)} "
                f"corridor_center_y={corridor.corridor_center_y:+.3f} "
                f"vehicle_lateral_error={corridor.vehicle_lateral_error:+.3f} "
                f"center_deadband_state={command.center_deadband_state} "
                f"wall_safety_state={command.wall_safety_state} "
                f"original_centerline_target_y={command.original_target.y:+.3f} "
                f"soft_target_y={command.soft_target_y:+.3f} "
                f"recenter_offset={command.recenter_offset:+.3f} "
                f"recenter_length={command.recenter_length:.2f} "
                f"final_target_y={command.target.y:+.3f} "
                f"heading_steering={command.heading_steering_angle:+.3f} "
                f"lateral_steering={command.lateral_steering_angle:+.3f} "
                f"raw_steering={command.raw_steering_angle:+.3f} "
                f"filtered_steering={command.steering_angle:+.3f} "
                f"original_target_y_range=[{self.original_target_y_min:+.3f},{self.original_target_y_max:+.3f}] "
                f"soft_target_y_range=[{self.soft_target_y_min:+.3f},{self.soft_target_y_max:+.3f}] "
                f"final_target_y_range=[{self.target_y_min:+.3f},{self.target_y_max:+.3f}] "
                f"raw_steering_range=[{self.raw_steering_min:+.3f},{self.raw_steering_max:+.3f}] "
                f"filtered_steering_range=[{self.filtered_steering_min:+.3f},{self.filtered_steering_max:+.3f}] "
                f"target_sign_flips={self.target_sign_flips} "
                f"steering_sign_flips={self.steering_sign_flips} "
                f"speed={plan.final_speed:.2f} speed_max={self.commanded_speed_max:.2f} "
                f"speed_state={plan.state} speed_preview={plan.preview_distance:.1f} "
                f"speed_source={speed_source} curv_bands=[{curvature_bands}] "
                f"limiter={limiter_reason} target_speed={plan.target_speed:.2f} "
                f"limit_max={self.speed_planner.max_speed:.2f} "
                f"limit_curve={plan.speed_limit_from_curvature:.2f} "
                f"limit_steer={plan.speed_limit_from_steering:.2f} "
                f"limit_corridor={plan.speed_limit_from_corridor:.2f} "
                f"limit_state={state_ceiling:.2f} limiter_counts={{{limiter_counts}}} "
                f"boost_gate_reason={boost_gate_text} "
                f"boost_gate_counts={{{boost_gate_counts}}} "
                f"braking_reason={braking_text} "
                f"braking_counts={{{braking_counts}}} "
                f"boost_exit_counts={{{boost_exit_counts}}} "
                f"curvature_runs={{{curvature_run_counts}}} "
                f"curvature_run_scans={self.curvature_run_scans} "
                f"curve_d={self._finite_text(plan.distance_to_curve)} "
                f"brake_d={plan.required_braking_distance:.2f} "
                f"boost_hold={plan.straight_hold_time:.2f} "
                f"wall_heading_delta={self._finite_text(plan.wall_heading_delta)} "
                f"sources={{{sources}}} scan_sim={samples / sim_elapsed:.1f}Hz "
                f"scan_wall={samples / wall_elapsed:.1f}Hz rtf={sim_elapsed / wall_elapsed:.2f} "
                f"opponent_scan_sim={secondary_samples / secondary_sim_elapsed:.1f}Hz "
                f"opponent_scan_wall={secondary_samples / secondary_wall_elapsed:.1f}Hz "
                f"wall_fail={self.wall_tracking_failures} timeout={self.scan_timeout_events} "
                f"steer_sat={self.steering_saturation_events}"
                f" opponent_count={len(opponents)}"
                f" opponent_tracks=[{opponent_tracks}]"
                f" pass_track_id={self.race_strategy.pass_track_id}"
                f" opponent_candidate_count={self.opponent_candidate_count}"
                f" primary_opponent_candidates={self.primary_opponent_candidate_count}"
                f" secondary_opponent_candidates={self.secondary_opponent_candidate_count}"
                f" opponent_detection_persistence="
                f"{self.opponent_detection_hits / max(1, self.opponent_detection_updates):.3f}"
                f" opponent_encounter_persistence="
                f"{self.opponent_encounter_hits / max(1, self.opponent_encounter_updates):.3f}"
                f" opponent_encounter_updates={self.opponent_encounter_updates}"
                f" opponent_masked_points={self.opponent_masked_points}"
                f" closest_front_distance={self._finite_text(race_decision.closest_front_distance)}"
                f" closest_rear_distance={self._finite_text(race_decision.closest_rear_distance)}"
                f" front_closing_rate={race_decision.front_closing_rate:+.3f}"
                f" rear_closing_rate={race_decision.rear_closing_rate:+.3f}"
                f" opponent_relative_x={self._finite_text(race_decision.primary_opponent_x)}"
                f" opponent_relative_y={self._finite_text(race_decision.primary_opponent_y)}"
                f" race_state={race_decision.race_state}"
                f" selected_pass_side={race_decision.selected_pass_side}"
                f" left_pass_clearance={self._finite_text(race_decision.left_pass_clearance)}"
                f" right_pass_clearance={self._finite_text(race_decision.right_pass_clearance)}"
                f" desired_lateral_offset={race_decision.desired_lateral_offset:+.3f}"
                f" pass_safety_state={race_decision.pass_safety_state}"
                f" current_wall_clearance="
                f"{self._finite_text(race_decision.current_wall_clearance)}"
                f" current_opponent_clearance="
                f"{self._finite_text(race_decision.current_opponent_clearance)}"
                f" predicted_pass_clearance="
                f"{self._finite_text(race_decision.predicted_pass_clearance)}"
                f" remaining_pass_gap="
                f"{self._finite_text(race_decision.remaining_pass_gap)}"
                f" opponent_lateral_velocity="
                f"{race_decision.opponent_lateral_velocity:+.3f}"
            )


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
