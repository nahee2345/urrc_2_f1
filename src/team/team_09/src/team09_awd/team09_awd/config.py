"""Single source of vehicle geometry, sensor extrinsics and operating limits.

Values are SI (m, s, rad) unless a key explicitly ends in _deg. No map data,
spawn coordinates, user names, or host transport environment is loaded here.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import re

DEFAULT = {
    "entry_id": "team_09",
    "geometry": {
        "wheelbase_m": 0.33, "track_m": 0.18,
        "wheel_radius_m": 0.035, "wheel_width_m": 0.026,
        "wheel_z_m": -0.04,
        "body_size_m": [0.49, 0.16, 0.05], "body_z_m": 0.0,
        "body_mass_kg": 1.2, "wheel_mass_kg": 0.06,
        "knuckle_mass_kg": 0.09, "max_wheel_angle_deg": 30.0,
        "footprint_padding_m": 0.002, "tyre_friction": 1.0,
        "steer_p_gain": 8.0, "steer_effort_nm": 10.0,
        "steer_joint_rate_radps": 4.0, "drive_effort_nm": 5.0,
        "drive_joint_rate_radps": 1500.0,
        "sensor_housings": [
            {"name": "camera_house", "xyz": [0.155, 0.0, 0.05], "size": [0.035, 0.035, 0.04]},
            {"name": "lidar_house", "xyz": [0.04, 0.0, 0.055], "size": [0.035, 0.035, 0.04]}
        ]
    },
    "sensors": {
        "lidar_pose": [0.04, 0.0, 0.075, 0.0, 0.0, 0.0],
        "camera_pose": [0.18, 0.0, 0.055, 0.0, 0.105, 0.0],
        "start_camera_pose": [0.18, 0.0, 0.055, 0.0, -0.50, 0.15],
        "load_imu_contact_systems": True,
        "lidar_mode": "2d", "lidar_fov_deg": 360.0,
        "lidar_horizontal_samples": 720, "lidar_vertical_samples_3d": 16,
        "lidar_vertical_fov_deg": 30.0, "lidar_min_m": 0.08, "lidar_max_m": 60.0,
        "lidar_hz": 30.0, "camera_hz": 30.0, "imu_hz": 100.0,
        "camera_width": 640, "camera_height": 480,
        "camera_hfov_rad": 1.5707963, "start_camera_hfov_rad": 1.1,
        "camera_near_m": 0.03, "camera_far_m": 100.0,
        "cloud_min_ground_height_m": 0.035, "cloud_max_ground_height_m": 0.65,
        "rgbd_pair_tolerance_s": 0.04, "rgbd_process_interval_s": 0.09,
        "rgbd_valid_s": 0.20, "rgbd_queue_size": 8
    },
    "control": {
        "max_speed_mps": 12.0, "use_depth": True,
        "lateral_accel_mps2": 2.5, "brake_mps2": 4.0, "accel_mps2": 1.5,
        "collision_margin_m": 0.06, "planner_hz": 20.0,
        "steer_rate_max_radps": 0.9, "steer_rate_base_radps": 0.06,
        "steer_rate_speed_gain": 1.2, "curvature_lateral_bound_mps2": 3.0,
        "near_clearance_m": 0.12, "near_speed_mps": 0.45,
        "close_clearance_m": 0.22, "close_speed_mps": 1.2,
        "reactive_speed_mps": 8.0, "yaw_residual_limit_radps": 0.6,
        "yaw_residual_speed_mps": 5.0, "minimum_latency_s": 0.20,
        "sensor_latency_reserve_s": 0.10, "max_scan_age_s": 0.30,
        "max_pose_age_s": 0.20, "sensor_wall_timeout_s": 0.60,
        "max_roll_deg": 35.0, "max_pitch_deg": 50.0,
        "reaccel_guard": True, "guard_min_speed_mps": 3.0,
        "guard_clearance_m": 0.60, "guard_closing_mps": 0.08,
        "guard_wall_quality": 0.35, "guard_heading_rad": 0.04,
        "guard_release_s": 0.50, "guard_max_sample_gap_s": 0.30
    },
    "right_wall": {
        "enabled": True,
        "clearance_m": 0.22,
        "min_corner_clearance_m": 0.07,
        "sweep_margin_m": 0.018,
        "corner_curvature_1pm": 0.055,
        "corner_fraction": 0.78,
        "corner_setup_m": 3.2,
        "corner_blend_m": 2.3,
        "lookahead_base_m": 0.95,
        "lookahead_time_s": 0.45,
        "lookahead_max_m": 4.5,
        "horizon_m": 27.0,
        "preview_latency_s": 0.20
    },
    "road_heading": {
        "band_min_m": 0.18, "band_max_m": 0.45,
        "relief_gain": 1.0, "relief_max_m": 0.14, "relief_length_m": 2.8,
        "vision_weight": 0.20, "vision_heading_agreement_rad": 0.20
    },
    "start_signal": {
        "on_frames": 3, "off_frames": 3,
        "red_hue_band": 12, "min_red_saturation": 140, "min_red_value": 180,
        "off_red_ratio": 0.12, "off_value_fraction": 0.65,
        "scene_delta_limit": 12.0, "black_value_max": 3,
        "scene_min_nonblack_fraction": 0.05,
        "max_frame_gap_s": 0.25, "lock_timeout_s": 3.0,
        "search_margin_fraction": 0.10,
        "blob_min_area_fraction": 0.000048828125,
        "blob_max_area_fraction": 0.0048828125,
        "blob_min_height_fraction": 0.00625,
        "blob_max_height_fraction": 0.09375,
        "blob_min_aspect": 0.4, "blob_max_aspect": 2.5,
        "row_height_fraction": 0.7, "gap_ratio_max": 1.6,
        "min_gap_width_fraction": 0.00625,
        "background_padding_height_fraction": 0.008333333333333333,
        "debug_hz": 5.0, "camera_timeout_wall_s": 1.0,
        "camera_max_age_s": 0.50, "camera_future_tolerance_s": 0.10
    },
    "actuator": {
        "hz": 100.0, "joint_state_hz": 100.0,
        "max_accel_mps2": 3.0, "max_brake_mps2": 6.0,
        "command_timeout_s": 0.35, "state_timeout_s": 0.25, "state_pair_skew_s": 0.03,
        "max_reverse_mps": 0.20, "joint_speed_limit_mps": 30.0
    },
    "logging": {"csv_hz": 10.0, "status_hz": 5.0, "log_dir": ""}
}
SETTINGS = deepcopy(DEFAULT)


def merge(dst, src, prefix=""):
    for key, value in src.items():
        if key not in dst:
            raise ValueError(f"Unknown setting: {prefix}{key}")
        if isinstance(dst[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f"Expected object: {prefix}{key}")
            merge(dst[key], value, prefix + key + ".")
        else:
            dst[key] = value


def validate(cfg):
    if not re.fullmatch(r"team_[0-9]{2}", cfg["entry_id"]):
        raise ValueError("entry_id must have the form team_09")
    def check_finite(value):
        if isinstance(value, dict):
            for v in value.values(): check_finite(v)
        elif isinstance(value, list):
            for v in value: check_finite(v)
        elif isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError("Nonfinite configuration")
    check_finite(cfg)
    g, s, c, a = (cfg[x] for x in ("geometry", "sensors", "control", "actuator"))
    for key in ("wheelbase_m", "track_m", "wheel_radius_m", "wheel_width_m", "body_mass_kg"):
        if g[key] <= 0: raise ValueError(key + " must be positive")
    if not 0 < g["max_wheel_angle_deg"] < 60: raise ValueError("Invalid steering angle")
    if s["lidar_mode"] not in ("2d", "3d"): raise ValueError("lidar_mode must be 2d or 3d")
    if not 90 <= s["lidar_fov_deg"] <= 360: raise ValueError("Invalid lidar_fov_deg")
    for key in ("lidar_pose", "camera_pose", "start_camera_pose"):
        if len(s[key]) != 6: raise ValueError(key + " requires xyz/rpy")
    if not 0 < c["max_speed_mps"] <= 12:
        raise ValueError("This release is limited to 12 m/s; higher speeds need separate validation")
    if min(c["planner_hz"], c["brake_mps2"], c["accel_mps2"], a["hz"], a["max_brake_mps2"]) <= 0:
        raise ValueError("Rates and acceleration limits must be positive")
    if c["brake_mps2"] > a["max_brake_mps2"]:
        raise ValueError("Planner braking assumption exceeds actuator deceleration limit")
    wall = cfg["right_wall"]
    if not isinstance(wall["enabled"], bool):
        raise ValueError("right_wall.enabled must be a boolean")
    for key, value in wall.items():
        if key != "enabled" and (isinstance(value, bool) or not isinstance(value, (float, int)) or value <= 0):
            raise ValueError("right_wall." + key + " must be positive")
    if not wall["sweep_margin_m"] < wall["min_corner_clearance_m"] < wall["clearance_m"]:
        raise ValueError("right_wall margins must increase from sweep to corner to straight")
    if not 0. < wall["corner_fraction"] <= 1. or not wall["lookahead_base_m"] <= wall["lookahead_max_m"] < wall["horizon_m"]:
        raise ValueError("Invalid right-wall corner fraction or lookahead")
    road = cfg["road_heading"]
    if not 0 < wall["sweep_margin_m"] < road["band_min_m"] < road["band_max_m"]:
        raise ValueError("Invalid road_heading side band")
    if not 0 < road["relief_max_m"] <= road["band_min_m"] or min(road["relief_gain"], road["relief_length_m"]) <= 0:
        raise ValueError("Invalid road_heading wall relief")
    if not 0 <= road["vision_weight"] <= .5 or not 0 < road["vision_heading_agreement_rad"] < .5:
        raise ValueError("Invalid road_heading camera blend")
    v = cfg["start_signal"]
    for key in ("on_frames", "off_frames"):
        if not isinstance(v[key], int) or isinstance(v[key], bool) or v[key] < 2:
            raise ValueError("start_signal." + key + " must be an integer >= 2")
    for key in ("off_red_ratio", "off_value_fraction", "scene_min_nonblack_fraction"):
        if not 0 < v[key] < 1: raise ValueError("Invalid start_signal." + key)
    for key in ("max_frame_gap_s", "lock_timeout_s", "scene_delta_limit", "debug_hz", "camera_timeout_wall_s", "camera_max_age_s", "camera_future_tolerance_s"):
        if v[key] <= 0: raise ValueError("start_signal." + key + " must be positive")
    if v["lock_timeout_s"] <= v["max_frame_gap_s"]:
        raise ValueError("Signal lock timeout must exceed allowed frame gap")
    if not 0 <= v["black_value_max"] < v["min_red_value"] <= 255 or not 0 <= v["min_red_saturation"] <= 255:
        raise ValueError("Invalid signal HSV thresholds")
    if not 0 < v["red_hue_band"] < 90 or not 0 <= v["search_margin_fraction"] < .5:
        raise ValueError("Invalid signal hue/search range")
    if not 0 < v["blob_min_area_fraction"] < v["blob_max_area_fraction"] < 1:
        raise ValueError("Invalid signal blob area range")
    if not 0 < v["blob_min_height_fraction"] < v["blob_max_height_fraction"] < 1:
        raise ValueError("Invalid signal blob size range")
    if not 0 < v["blob_min_aspect"] < v["blob_max_aspect"] or v["gap_ratio_max"] <= 1:
        raise ValueError("Invalid signal lamp geometry")
    if min(v["row_height_fraction"], v["min_gap_width_fraction"], v["background_padding_height_fraction"]) <= 0:
        raise ValueError("Invalid signal spacing/padding")
    return cfg


def load(path=None):
    cfg = deepcopy(DEFAULT)
    if path:
        merge(cfg, json.loads(Path(path).read_text(encoding="utf-8")))
    validate(cfg)
    SETTINGS.clear(); SETTINGS.update(cfg)
    return SETTINGS


class Geometry:
    @property
    def wheelbase(self): return SETTINGS["geometry"]["wheelbase_m"]
    @property
    def track(self): return SETTINGS["geometry"]["track_m"]
    @property
    def radius(self): return SETTINGS["geometry"]["wheel_radius_m"]
    @property
    def half_length(self):
        g = SETTINGS["geometry"]
        return max(g["body_size_m"][0]/2, self.wheelbase/2+self.radius)
    @property
    def half_width(self):
        g = SETTINGS["geometry"]; angle = math.radians(g["max_wheel_angle_deg"])
        # Max cylindrical wheel support along body y over the allowed steering range.
        a = min(angle, math.atan2(self.radius, g["wheel_width_m"]/2))
        return max(g["body_size_m"][1]/2, self.track/2 + self.radius*math.sin(a)
                   + g["wheel_width_m"]/2*math.cos(a)) + g["footprint_padding_m"]
    @property
    def ground_z(self): return SETTINGS["geometry"]["wheel_z_m"] - self.radius
    @property
    def max_curvature(self):
        t = math.tan(math.radians(SETTINGS["geometry"]["max_wheel_angle_deg"]))
        return t/(self.wheelbase+self.track*t/2)
    @property
    def max_centre_angle(self): return math.atan(self.wheelbase*self.max_curvature)
    @property
    def max_wheel_angle(self): return math.radians(SETTINGS["geometry"]["max_wheel_angle_deg"])

G = Geometry()


def topic(suffix):
    return "/" + SETTINGS["entry_id"] + "/" + suffix.lstrip("/")


def frame(suffix):
    return SETTINGS["entry_id"] + "/" + suffix.lstrip("/")
