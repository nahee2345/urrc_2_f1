#pragma once
#include "team_01_race/predictive_planner.hpp"

namespace team_01_race {
// Called only for new scan frames, never for timer/controller callbacks.
class PathReadiness
{
public:
  void reset() {count_ = 0; previous_ = Plan{};}
  int count() const {return count_;}
  bool update(const Plan & plan, const PlannerConfig & config, bool scan_valid, int required)
  {
    const bool usable = scan_valid && plan.valid && plan.centerline_valid &&
      plan.current_centerline_observed && plan.target_valid && !plan.emergency_stop &&
      !plan.straight_guard_rejected && std::isfinite(plan.official_steering_rad) &&
      std::abs(plan.official_steering_rad) < 0.95 * config.max_steering_angle_rad;
    if (!usable) {reset(); return false;}
    bool consistent = count_ == 0 ||
      (std::abs(plan.target_y_m - previous_.target_y_m) <= config.target_jump_threshold_m &&
      std::abs(plan.predicted_curvature - previous_.predicted_curvature) <= config.curvature_spike_threshold &&
      (plan.official_steering_rad * previous_.official_steering_rad >= 0.0 ||
      std::max(std::abs(plan.official_steering_rad), std::abs(previous_.official_steering_rad)) <=
      config.steering_deadband_rad));
    if (count_ && plan.centerline.size() != previous_.centerline.size()) consistent = false;
    if (consistent && count_) {
      for (std::size_t i = 0; i < plan.centerline.size(); ++i) {
        if (std::abs(plan.centerline[i].y - previous_.centerline[i].y) >
          config.trajectory_jump_threshold_m) consistent = false;
      }
    }
    count_ = consistent ? std::min(count_ + 1, required) : 1;
    previous_ = plan;
    return count_ >= required;
  }
private:
  int count_{0};
  Plan previous_;
};
}
