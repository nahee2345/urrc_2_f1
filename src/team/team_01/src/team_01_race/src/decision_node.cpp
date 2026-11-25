// LEGACY reference: not built or launched. Active camera/obstacle/Pure Pursuit nodes are in team_01_race/*.py.
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "team_01_race/predictive_planner.hpp"
#include "team_01_race/path_readiness.hpp"

using SteadyClock = std::chrono::steady_clock;

class DecisionNode : public rclcpp::Node
{
public:
  DecisionNode()
  : Node("team_01_decision"), config_(read_config()), planner_(config_)
  {
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/team_01/internal/scan");
    const auto start_topic = declare_parameter<std::string>("race_start_topic", "/race/start/team_01");
    const auto speed_topic = declare_parameter<std::string>(
      "requested_speed_topic", "/team_01/internal/requested_speed");
    const auto steering_topic = declare_parameter<std::string>(
      "requested_steering_topic", "/team_01/internal/requested_steering");
    const auto controller_ready_topic = declare_parameter<std::string>(
      "controller_ready_topic", "/team_01/internal/controller_ready");
    sensor_timeout_sec_ = declare_parameter<double>("safety.sensor_timeout_sec", 0.25);

    path_ready_confirm_frames_ = declare_parameter<int>("safety.path_ready_confirm_frames", 3);
    if (path_ready_confirm_frames_ < 1) throw std::invalid_argument("invalid path ready frame count");
    path_diag_pub_ = create_publisher<std_msgs::msg::String>("/team_01/debug/path_diagnostics", 10);
    path_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/team_01/debug/path_ready", 10);
    path_ready_count_pub_ = create_publisher<std_msgs::msg::Int32>("/team_01/debug/path_ready_count", 10);
    speed_pub_ = create_publisher<std_msgs::msg::Float32>(speed_topic, 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float32>(steering_topic, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/team_01/debug/state", 10);
    reason_pub_ = create_publisher<std_msgs::msg::String>("/team_01/debug/stop_reason", 10);
    requested_speed_debug_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team_01/debug/requested_speed", 10);
    confidence_pub_ = create_publisher<std_msgs::msg::Float32>("/team_01/debug/path_confidence", 10);
    curvature_pub_ = create_publisher<std_msgs::msg::Float32>("/team_01/debug/maximum_curvature", 10);
    ttc_pub_ = create_publisher<std_msgs::msg::Float32>("/team_01/debug/minimum_ttc", 10);
    lookahead_pub_ = create_publisher<std_msgs::msg::Float32>("/team_01/debug/lookahead", 10);
    passing_pub_ = create_publisher<std_msgs::msg::String>("/team_01/debug/passing_side", 10);
    centerline_lateral_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team_01/debug/centerline_lateral", 10);
    raw_trajectory_lateral_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team_01/debug/raw_trajectory_lateral", 10);
    filtered_trajectory_lateral_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team_01/debug/filtered_trajectory_lateral", 10);
    target_x_pub_ = create_publisher<std_msgs::msg::Float32>("/team_01/debug/target_x", 10);
    target_y_pub_ = create_publisher<std_msgs::msg::Float32>("/team_01/debug/target_y", 10);
    predicted_curvature_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team_01/debug/predicted_curvature", 10);
    steering_ff_pub_ = create_publisher<std_msgs::msg::Float32>("/team_01/debug/steering_ff", 10);
    steering_fb_pub_ = create_publisher<std_msgs::msg::Float32>("/team_01/debug/steering_fb", 10);
    desired_steering_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team_01/debug/desired_steering", 10);
    raw_fb_component_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team_01/debug/raw_fb_component", 10);
    limited_fb_component_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team_01/debug/limited_fb_component", 10);
    selected_trajectory_pub_ = create_publisher<std_msgs::msg::String>(
      "/team_01/debug/selected_trajectory", 10);
    raw_curvature_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team_01/debug/raw_maximum_curvature", 10);
    raw_target_y_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team_01/debug/raw_target_y", 10);
    left_boundary_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/team_01/debug/left_boundary_valid", 10);
    right_boundary_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/team_01/debug/right_boundary_valid", 10);
    straight_guard_active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/team_01/debug/straight_guard_active", 10);
    straight_guard_reason_pub_ = create_publisher<std_msgs::msg::String>(
      "/team_01/debug/straight_guard_reason", 10);
    obstacle_count_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/team_01/debug/confirmed_obstacle_count", 10);
    started_pub_ = create_publisher<std_msgs::msg::Bool>("/team_01/debug/race_started", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        const auto frame = team_01_race::extract_predictive_frame(*scan, config_);
        latest_frame_ = frame;
        latest_plan_ = planner_.plan(frame, steady_seconds());
        if (latest_plan_.straight_guard_rejected) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[STRAIGHT_GUARD] reason=%s raw_y=%.3f filtered_y=%.3f raw_target_y=%.3f "
            "validated_target_y=%.3f raw_curvature=%.3f validated_curvature=%.3f "
            "trajectory=%s obstacles=%d delta_ff=%.3f delta_fb=%.3f desired=%.3f final=%.3f",
            latest_plan_.straight_guard_reason.c_str(), latest_plan_.raw_centerline_lateral_m,
            latest_plan_.centerline_lateral_m, latest_plan_.raw_target_y_m,
            latest_plan_.target_y_m, latest_plan_.raw_maximum_curvature,
            latest_plan_.predicted_curvature,
            latest_plan_.passing_side == team_01_race::PassingSide::NONE ? "CENTER" :
            team_01_race::passing_side_name(latest_plan_.passing_side),
            latest_plan_.confirmed_obstacle_count, latest_plan_.steering_ff_rad,
            latest_plan_.steering_fb_rad, latest_plan_.desired_internal_steering_rad,
            latest_plan_.official_steering_rad);
        }
        scan_received_ = true;
        last_scan_ = SteadyClock::now();
        if (race_started_) {
          if (!latest_plan_.valid || !frame.scan_valid) {
            readiness_.reset();
            if (!latest_plan_.valid) path_confirmed_ = false;
          }
          if (!path_confirmed_) {
            path_confirmed_ = readiness_.update(
              latest_plan_, config_, frame.scan_valid, path_ready_confirm_frames_);
          }
        }
        publish_command();
      });
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      start_topic, rclcpp::QoS(10).reliable().durability_volatile(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (!start_received_ || race_started_ != message->data) {
          RCLCPP_INFO(get_logger(), "[START_GATE] received: %s", message->data ? "TRUE" : "FALSE");
        }
        start_received_ = true;
        if (message->data && !race_started_) {
          RCLCPP_INFO(get_logger(), "[START_DIAG] %s", path_diagnostics().c_str());
          planner_.reset();
          latest_plan_ = team_01_race::Plan{};
          readiness_.reset();
          path_confirmed_ = false;
          has_run_ = false;
        }
        race_started_ = message->data;
        if (!race_started_) {
          readiness_.reset();
          path_confirmed_ = false;
        }
        publish_command();
      });
    controller_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      controller_ready_topic, 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        // Ready is status, not a request for another command. Re-emitting a
        // command for every unchanged ready heartbeat creates a feedback loop
        // with the control adapter's command callbacks.
        if (controller_ready_ == message->data) return;
        controller_ready_ = message->data;
        publish_command();
      });
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/team_01/emergency_stop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        external_emergency_ = message->data;
        publish_command();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {publish_command();});
    RCLCPP_INFO(get_logger(), "Waiting for official %s signal", start_topic.c_str());
    publish_command();
  }

private:
  template<typename T>
  T parameter(const char * name, T value) {return declare_parameter<T>(name, value);}

  team_01_race::PlannerConfig read_config()
  {
    team_01_race::PlannerConfig config;
#define PARAM(field, name) config.field = parameter<decltype(config.field)>(name, config.field)
    PARAM(planning_horizon_m, "planner.planning_horizon_m");
    PARAM(trajectory_spacing_m, "planner.trajectory_spacing_m");
    PARAM(temporal_history_sec, "planner.temporal_history_sec");
    PARAM(trajectory_temporal_alpha, "planner.trajectory_temporal_alpha");
    PARAM(cruise_speed_mps, "planner.cruise_speed_mps");
    PARAM(min_path_confidence, "planner.min_path_confidence");
    PARAM(max_lateral_accel_mps2, "planner.max_lateral_accel_mps2");
    PARAM(default_track_width_m, "planner.default_track_width_m");
    PARAM(min_track_width_m, "planner.min_track_width_m");
    PARAM(max_track_width_m, "planner.max_track_width_m");
    PARAM(min_lookahead_m, "control.min_lookahead_m");
    PARAM(max_lookahead_m, "control.max_lookahead_m");
    PARAM(wheelbase_m, "control.wheelbase_m");
    PARAM(max_steering_angle_rad, "control.max_steering_angle_rad");
    PARAM(max_feedback_steering_rad, "control.max_feedback_steering_rad");
    PARAM(heading_gain, "control.heading_gain");
    PARAM(lateral_gain, "control.lateral_gain");
    PARAM(curvature_alpha, "control.curvature_alpha");
    PARAM(lookahead_alpha, "control.lookahead_alpha");
    PARAM(steering_deadband_rad, "control.steering_deadband_rad");
    PARAM(straight_curvature_threshold, "straight_guard.curvature_threshold");
    PARAM(straight_heading_threshold_rad, "straight_guard.heading_threshold_rad");
    PARAM(straight_lateral_variation_m, "straight_guard.lateral_variation_m");
    PARAM(boundary_heading_difference_rad, "straight_guard.boundary_heading_difference_rad");
    PARAM(trajectory_jump_threshold_m, "straight_guard.trajectory_jump_threshold_m");
    PARAM(target_jump_threshold_m, "straight_guard.target_jump_threshold_m");
    PARAM(curvature_spike_threshold, "straight_guard.curvature_spike_threshold");
    PARAM(straight_enter_frames, "straight_guard.enter_frames");
    PARAM(straight_exit_frames, "straight_guard.exit_frames");
    PARAM(path_change_confirm_frames, "straight_guard.path_change_confirm_frames");
    PARAM(detection_distance_m, "obstacle.detection_distance_m");
    PARAM(min_vehicle_width_m, "obstacle.min_vehicle_width_m");
    PARAM(max_vehicle_width_m, "obstacle.max_vehicle_width_m");
    PARAM(min_vehicle_depth_m, "obstacle.min_vehicle_depth_m");
    PARAM(max_vehicle_depth_m, "obstacle.max_vehicle_depth_m");
    PARAM(obstacle_confirm_frames, "obstacle.confirm_frames");
    PARAM(vehicle_lateral_margin_m, "obstacle.vehicle_lateral_margin_m");
    PARAM(vehicle_longitudinal_margin_m, "obstacle.vehicle_longitudinal_margin_m");
    PARAM(ego_width_m, "obstacle.ego_width_m");
    PARAM(emergency_ttc_sec, "safety.emergency_ttc_sec");
    PARAM(caution_ttc_sec, "safety.caution_ttc_sec");
    PARAM(assumed_max_decel_mps2, "safety.assumed_max_decel_mps2");
    PARAM(min_valid_scan_beams, "perception.min_valid_scan_beams");
    PARAM(min_boundary_points, "perception.min_boundary_points");
    PARAM(cluster_gap_m, "perception.cluster_gap_m");
#undef PARAM
    config.validate();
    return config;
  }

  static double steady_seconds()
  {
    return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
  }

  std::string path_diagnostics() const
  {
    auto mean_y = [](const std::vector<team_01_race::Point2> & points) {
      double sum = 0.0;
      for (const auto & p : points) sum += p.y;
      return points.empty() ? 0.0 : sum / points.size();
    };
    const auto & p = latest_plan_;
    const double scan_age = scan_received_ ?
      std::chrono::duration<double>(SteadyClock::now() - last_scan_).count() : -1.0;
    const bool ready = race_started_ && scan_received_ && scan_age <= sensor_timeout_sec_ &&
      path_confirmed_ && controller_ready_ && !external_emergency_ && !p.emergency_stop &&
      p.valid && p.centerline_valid && p.target_valid;
    std::ostringstream out;
    out << "scan_received=" << scan_received_ << " scan_age="
        << scan_age
        << " SCAN_VALID=" << latest_frame_.scan_valid << " valid_beams=" << latest_frame_.valid_beams
        << " finite_lidar_points=" << latest_frame_.finite_point_count
        << " LEFT_VALID=" << p.left_boundary_valid << " RIGHT_VALID=" << p.right_boundary_valid
        << " left_points=" << latest_frame_.left.size() << " right_points=" << latest_frame_.right.size()
        << " LEFT_MEAN_Y=" << mean_y(latest_frame_.left) << " RIGHT_MEAN_Y=" << mean_y(latest_frame_.right)
        << " current_centerline_observed=" << p.current_centerline_observed
        << " boundary_clearance=" << p.minimum_boundary_clearance_m
        << " CENTERLINE_VALID=" << p.centerline_valid << " centerline_points=" << p.centerline.size();
    if (!p.centerline.empty()) out << " first=(" << p.centerline.front().x << "," << p.centerline.front().y
      << ") last=(" << p.centerline.back().x << "," << p.centerline.back().y << ")";
    out << " CENTER_REFERENCE_Y=" << p.centerline_lateral_m << " TRAJECTORY_VALID=" << p.valid
        << " trajectory_points=" << p.trajectory.size() << " SELECTED_TRAJECTORY="
        << (p.passing_side == team_01_race::PassingSide::NONE ? "CENTER" : team_01_race::passing_side_name(p.passing_side))
        << " raw_clusters=" << latest_frame_.raw_cluster_count
        << " obstacle_candidates=" << latest_frame_.obstacle_candidates.size()
        << " confirmed_obstacles=" << p.confirmed_obstacle_count
        << " reason=" << p.reason << " TARGET_VALID=" << p.target_valid
        << " TARGET_X=" << p.target_x_m << " TARGET_Y=" << p.target_y_m
        << " CURVATURE=" << p.predicted_curvature << " DELTA_FF=" << p.steering_ff_rad
        << " DELTA_FB=" << p.steering_fb_rad << " FINAL_STEERING=" << p.official_steering_rad
        << " target_speed=" << p.target_speed_mps << " PATH_READY_COUNT=" << readiness_.count()
        << "/" << path_ready_confirm_frames_ << " PATH_READY=" << ready;
    return out.str();
  }

  void publish_command()
  {
    const bool stale = !scan_received_ ||
      std::chrono::duration<double>(SteadyClock::now() - last_scan_).count() > sensor_timeout_sec_;
    std::string state = "WAIT_START";
    std::string reason = "waiting_for_race_start";
    double speed = 0.0;
    double steering = 0.0;

    if (!race_started_) {
      state = "WAIT_START";
      reason = "waiting_for_race_start";
    } else if (external_emergency_ || latest_plan_.emergency_stop) {
      state = "EMERGENCY_STOP";
      reason = external_emergency_ ? "external_emergency_stop" : latest_plan_.reason;
    } else if (stale) {
      readiness_.reset();
      path_confirmed_ = false;
      state = "FAULT";
      reason = "lidar_timeout";
    } else if (!path_confirmed_ || !controller_ready_) {
      state = has_run_ && !path_confirmed_ ? "FAULT" : "WAIT_PATH_READY";
      if (has_run_ && !latest_plan_.valid) reason = latest_plan_.reason;
      else reason = !path_confirmed_ ? "waiting_for_stable_centerline" : "waiting_for_controller_ready";
    } else if (!latest_plan_.valid || !latest_plan_.centerline_valid || !latest_plan_.target_valid) {
      state = "FAULT";
      reason = latest_plan_.reason;
    } else {
      state = "RUNNING";
      has_run_ = true;
      reason = latest_plan_.reason;
      speed = latest_plan_.target_speed_mps;
      steering = latest_plan_.official_steering_rad;
    }

    if (state != previous_state_) {
      RCLCPP_INFO(get_logger(), "[START_GATE] %s -> %s (%s)",
        previous_state_.c_str(), state.c_str(), reason.c_str());
      previous_state_ = state;
    }
    std_msgs::msg::String diag;
    diag.data = path_diagnostics(); path_diag_pub_->publish(diag);
    std_msgs::msg::Bool ready;
    ready.data = path_confirmed_ && !stale && controller_ready_ && race_started_ &&
      !external_emergency_ && latest_plan_.valid && !latest_plan_.emergency_stop;
    path_ready_pub_->publish(ready);
    std_msgs::msg::Int32 ready_count;
    ready_count.data = readiness_.count(); path_ready_count_pub_->publish(ready_count);
    if (race_started_ && (state == "WAIT_PATH_READY" || (state == "FAULT" && !path_confirmed_))) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "[PATH_READY] %s", diag.data.c_str());
    }
    std_msgs::msg::Float32 number;
    number.data = static_cast<float>(speed); speed_pub_->publish(number);
    number.data = static_cast<float>(steering); steering_pub_->publish(number);
    number.data = static_cast<float>(speed); requested_speed_debug_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.confidence); confidence_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.maximum_curvature); curvature_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.minimum_ttc_sec); ttc_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.lookahead_m); lookahead_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.centerline_lateral_m);
    centerline_lateral_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.raw_centerline_lateral_m);
    raw_trajectory_lateral_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.centerline_lateral_m);
    filtered_trajectory_lateral_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.target_x_m); target_x_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.target_y_m); target_y_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.predicted_curvature);
    predicted_curvature_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.steering_ff_rad); steering_ff_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.steering_fb_rad); steering_fb_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.desired_internal_steering_rad);
    desired_steering_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.raw_maximum_curvature);
    raw_curvature_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.raw_target_y_m);
    raw_target_y_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.raw_fb_component_rad);
    raw_fb_component_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.limited_fb_component_rad);
    limited_fb_component_pub_->publish(number);
    std_msgs::msg::String text;
    text.data = state; state_pub_->publish(text);
    text.data = reason; reason_pub_->publish(text);
    text.data = team_01_race::passing_side_name(latest_plan_.passing_side); passing_pub_->publish(text);
    text.data = latest_plan_.passing_side == team_01_race::PassingSide::NONE ?
      "CENTER" : std::string(team_01_race::passing_side_name(latest_plan_.passing_side)) + "_PASS";
    selected_trajectory_pub_->publish(text);
    text.data = latest_plan_.straight_guard_reason;
    straight_guard_reason_pub_->publish(text);
    std_msgs::msg::Bool started;
    started.data = race_started_; started_pub_->publish(started);
    started.data = latest_plan_.left_boundary_valid; left_boundary_valid_pub_->publish(started);
    started.data = latest_plan_.right_boundary_valid; right_boundary_valid_pub_->publish(started);
    started.data = latest_plan_.straight_guard_active; straight_guard_active_pub_->publish(started);
    std_msgs::msg::Int32 count;
    count.data = latest_plan_.confirmed_obstacle_count; obstacle_count_pub_->publish(count);
  }

  team_01_race::PlannerConfig config_;
  team_01_race::PredictivePlanner planner_;
  team_01_race::Plan latest_plan_;
  double sensor_timeout_sec_{0.25};
  bool scan_received_{false};
  bool race_started_{false};
  bool start_received_{false};
  std::string previous_state_{"WAIT_START"};
  team_01_race::BoundaryFrame latest_frame_;
  team_01_race::PathReadiness readiness_;
  bool path_confirmed_{false};
  bool has_run_{false};
  int path_ready_confirm_frames_{3};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr path_ready_count_pub_;
  bool external_emergency_{false};
  bool controller_ready_{false};
  SteadyClock::time_point last_scan_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_, steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr requested_speed_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub_, curvature_pub_, ttc_pub_, lookahead_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr centerline_lateral_pub_, target_x_pub_, target_y_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_trajectory_lateral_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr filtered_trajectory_lateral_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr predicted_curvature_pub_, steering_ff_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_fb_pub_, desired_steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_fb_component_pub_, limited_fb_component_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_curvature_pub_, raw_target_y_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, reason_pub_, passing_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr straight_guard_reason_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr started_pub_, left_boundary_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_boundary_valid_pub_, straight_guard_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr obstacle_count_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_, controller_ready_sub_, emergency_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionNode>());
  rclcpp::shutdown();
}
