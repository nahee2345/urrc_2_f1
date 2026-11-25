#include <gtest/gtest.h>
#include <cmath>
#include "team_01_race/control_math.hpp"
#include "team_01_race/path_readiness.hpp"
#include "team_01_race/predictive_planner.hpp"

using team_01_race::BoundaryFrame;
using team_01_race::Cluster;
using team_01_race::PassingSide;
using team_01_race::Point2;
using team_01_race::PredictivePlanner;

namespace {
BoundaryFrame boundaries(double curve = 0.0, bool left = true, bool right = true,
  double half_width = 0.75, double lateral_offset = 0.0)
{
  BoundaryFrame frame;
  frame.scan_valid = true;
  frame.valid_beams = 181;
  for (int i = 1; i <= 50; ++i) {
    const double x = i * 0.1;
    const double center = lateral_offset + curve * x * x;
    if (left) frame.left.push_back({x, center + half_width});
    if (right) frame.right.push_back({x, center - half_width});
  }
  return frame;
}

Cluster vehicle(double x, double y, double width = 0.30, double depth = 0.30)
{
  return {{x, y}, width, depth, false};
}

team_01_race::Plan confirm_obstacle(PredictivePlanner & planner, BoundaryFrame frame)
{
  planner.plan(frame, 0.0);
  planner.plan(frame, 0.1);
  return planner.plan(frame, 0.2);
}
}  // namespace

TEST(PredictiveTrajectory, StraightTargetsCruiseSpeedAndZeroSteering)
{
  PredictivePlanner planner({});
  const auto result = planner.plan(boundaries(), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.official_steering_rad, 0.0, 1e-6);
  EXPECT_NEAR(result.target_speed_mps, 6.0, 1e-6);
  ASSERT_FALSE(result.trajectory.empty());
  EXPECT_NEAR(result.trajectory.back().x, 5.0, 1e-6);
}

TEST(PredictiveTrajectory, RightCurveProducesEarlyOfficialRightSteering)
{
  PredictivePlanner planner({});
  const auto result = planner.plan(boundaries(-0.025), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.official_steering_rad, 0.0);
  EXPECT_LT(result.lookahead_m, 3.0);
}

TEST(PredictiveTrajectory, LeftCurveProducesEarlyOfficialLeftSteering)
{
  PredictivePlanner planner({});
  const auto result = planner.plan(boundaries(0.025), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.official_steering_rad, 0.0);
}

TEST(PredictiveTrajectory, OneBoundaryReconstructsCenterlineWithTrackWidth)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  const auto result = planner.plan(boundaries(0.0, true, false), 0.1);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.target_speed_mps, 0.0);
  EXPECT_GT(result.confidence, 0.40);
}

TEST(PredictiveTrajectory, BriefLossContinuesPreviousTrajectoryWithoutSearchState)
{
  PredictivePlanner planner({});
  const auto initial = planner.plan(boundaries(-0.015), 0.0);
  ASSERT_TRUE(initial.valid);
  BoundaryFrame missing;
  missing.scan_valid = true;
  missing.valid_beams = 181;
  const auto continued = planner.plan(missing, 0.1);
  ASSERT_TRUE(continued.valid);
  EXPECT_EQ(continued.reason, "temporal_trajectory_continuation");
  EXPECT_GT(continued.target_speed_mps, 0.0);
}

TEST(PredictiveTrajectory, HalfMeterBoundariesDoNotCauseFixedStop)
{
  PredictivePlanner planner({});
  const auto result = planner.plan(boundaries(0.0, true, true, 0.5), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.emergency_stop);
  EXPECT_GT(result.target_speed_mps, 0.0);
}

TEST(FeedbackStability, AlternatingLidarJitterDoesNotFlipSteeringEveryFrame)
{
  PredictivePlanner planner({});
  int sign_flips = 0;
  int previous_sign = 0;
  for (int frame = 0; frame < 12; ++frame) {
    const double jitter = frame % 2 == 0 ? 0.08 : -0.08;
    const auto result = planner.plan(boundaries(0.0, true, true, 0.75, jitter), frame * 0.05);
    ASSERT_TRUE(result.valid);
    const int sign = result.official_steering_rad > 1e-5 ? 1 :
      (result.official_steering_rad < -1e-5 ? -1 : 0);
    if (previous_sign != 0 && sign != 0 && sign != previous_sign) ++sign_flips;
    if (sign != 0) previous_sign = sign;
  }
  EXPECT_LE(sign_flips, 1);
}

TEST(FeedbackStability, PersistentCurveIsPreservedByTemporalFilters)
{
  PredictivePlanner planner({});
  ASSERT_TRUE(planner.plan(boundaries(), 0.0).valid);
  team_01_race::Plan result;
  for (int frame = 1; frame <= 8; ++frame) {
    result = planner.plan(boundaries(-0.025), frame * 0.05);
  }
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.official_steering_rad, 0.0);
  EXPECT_LT(result.lookahead_m, 3.0);
  EXPECT_LT(result.predicted_curvature, 0.0);
}

TEST(FeedbackStability, FilterParametersDoNotChangeStraightCruiseSpeed)
{
  team_01_race::PlannerConfig config;
  config.trajectory_temporal_alpha = 0.30;
  config.curvature_alpha = 0.35;
  config.lookahead_alpha = 0.25;
  PredictivePlanner planner(config);
  const auto result = planner.plan(boundaries(), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(config.cruise_speed_mps, 6.0);
  EXPECT_DOUBLE_EQ(result.target_speed_mps, config.cruise_speed_mps);
}

TEST(FeedbackStability, FilteredTrajectoryMovesLessThanAlternatingRawTrajectory)
{
  PredictivePlanner planner({});
  auto previous = planner.plan(boundaries(0.0, true, true, 0.75, 0.08), 0.0);
  ASSERT_TRUE(previous.valid);
  for (int frame = 1; frame <= 8; ++frame) {
    const double jitter = frame % 2 == 0 ? 0.08 : -0.08;
    const auto current = planner.plan(
      boundaries(0.0, true, true, 0.75, jitter), frame * 0.05);
    ASSERT_TRUE(current.valid);
    const double raw_delta = std::abs(
      current.raw_centerline_lateral_m - previous.raw_centerline_lateral_m);
    const double filtered_delta = std::abs(
      current.centerline_lateral_m - previous.centerline_lateral_m);
    EXPECT_LT(filtered_delta, raw_delta);
    previous = current;
  }
}

TEST(StraightGuard, RejectsSingleFrameLateralTrajectoryJump)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  planner.plan(boundaries(), 0.05);
  const auto stable = planner.plan(boundaries(), 0.10);
  ASSERT_TRUE(stable.straight_guard_active);

  const auto outlier = planner.plan(
    boundaries(0.0, true, true, 1.5, 0.90), 0.15);
  ASSERT_TRUE(outlier.valid);
  EXPECT_TRUE(outlier.straight_guard_rejected);
  EXPECT_EQ(outlier.straight_guard_reason, "trajectory_jump");
  EXPECT_NEAR(outlier.centerline_lateral_m, stable.centerline_lateral_m, 1e-6);
  EXPECT_EQ(outlier.passing_side, PassingSide::NONE);
}

TEST(StraightGuard, AcceptsPersistentSameDirectionPathChange)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  planner.plan(boundaries(), 0.05);
  planner.plan(boundaries(), 0.10);
  planner.plan(boundaries(-0.08), 0.15);
  const auto tentative = planner.plan(boundaries(-0.08), 0.20);
  ASSERT_TRUE(tentative.straight_guard_rejected);
  const auto confirmed = planner.plan(boundaries(-0.08), 0.25);
  ASSERT_TRUE(confirmed.valid);
  EXPECT_FALSE(confirmed.straight_guard_rejected);
  EXPECT_LT(confirmed.centerline_lateral_m, -0.02);
}

TEST(StraightGuard, RejectsSingleFrameCurvatureSpike)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  planner.plan(boundaries(), 0.05);
  const auto stable = planner.plan(boundaries(), 0.10);
  auto spike = boundaries();
  for (auto & point : spike.left) {
    if (point.x > 2.3 && point.x < 2.7) point.y += 0.50;
  }
  for (auto & point : spike.right) {
    if (point.x > 2.3 && point.x < 2.7) point.y += 0.50;
  }
  const auto rejected = planner.plan(spike, 0.15);
  ASSERT_TRUE(rejected.valid);
  EXPECT_TRUE(rejected.straight_guard_rejected);
  EXPECT_EQ(rejected.straight_guard_reason, "curvature_spike");
  EXPECT_GT(rejected.raw_maximum_curvature, 0.75);
  EXPECT_NEAR(rejected.centerline_lateral_m, stable.centerline_lateral_m, 1e-6);
}

TEST(StraightGuard, RealCurveReleasesGuardWithoutSuppressingPredictiveSteering)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  planner.plan(boundaries(), 0.05);
  ASSERT_TRUE(planner.plan(boundaries(), 0.10).straight_guard_active);
  const auto curve_one = planner.plan(boundaries(-0.08), 0.15);
  const auto curve_two = planner.plan(boundaries(-0.08), 0.20);
  const auto curve_three = planner.plan(boundaries(-0.08), 0.25);
  ASSERT_TRUE(curve_one.valid);
  ASSERT_TRUE(curve_two.valid);
  EXPECT_FALSE(curve_one.straight_guard_rejected);
  ASSERT_TRUE(curve_three.valid);
  EXPECT_FALSE(curve_three.straight_guard_rejected);
  EXPECT_GT(curve_two.official_steering_rad, 0.0);
  EXPECT_FALSE(curve_three.straight_guard_active);
}

TEST(StraightGuard, ConfirmedObstacleStillSelectsPassTrajectory)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  planner.plan(boundaries(), 0.05);
  planner.plan(boundaries(), 0.10);
  auto frame = boundaries();
  frame.obstacle_candidates.push_back(vehicle(2.5, -0.30));
  planner.plan(frame, 0.15);
  planner.plan(frame, 0.20);
  const auto passing = planner.plan(frame, 0.25);
  ASSERT_TRUE(passing.valid);
  EXPECT_TRUE(passing.obstacle_confirmed);
  EXPECT_EQ(passing.passing_side, PassingSide::LEFT);
  EXPECT_FALSE(passing.straight_guard_rejected);
}


TEST(ObstacleTracking, LaserScanExtractsCompactInteriorCluster)
{
  team_01_race::PlannerConfig config;
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -team_01_race::kPi / 2.0;
  scan.angle_max = team_01_race::kPi / 2.0;
  scan.angle_increment = team_01_race::kPi / 360.0;
  scan.range_min = 0.04;
  scan.range_max = 6.0;
  scan.ranges.assign(361, std::numeric_limits<float>::infinity());
  for (int degree_twice = -20; degree_twice <= 20; ++degree_twice) {
    const double fraction = std::abs(degree_twice) / 20.0;
    scan.ranges[static_cast<std::size_t>(degree_twice + 180)] =
      static_cast<float>(2.0 + 0.22 * fraction);
  }
  const auto frame = team_01_race::extract_predictive_frame(scan, config);
  ASSERT_TRUE(frame.scan_valid);
  ASSERT_FALSE(frame.obstacle_candidates.empty());
  EXPECT_LT(frame.obstacle_candidates.front().centroid.x, 5.0);
}

TEST(ObstacleTracking, VehicleSizedClusterRequiresTemporalConfirmation)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  frame.obstacle_candidates.push_back(vehicle(2.5, 0.0));
  EXPECT_FALSE(planner.plan(frame, 0.0).obstacle_confirmed);
  EXPECT_FALSE(planner.plan(frame, 0.1).obstacle_confirmed);
  EXPECT_TRUE(planner.plan(frame, 0.2).obstacle_confirmed);
}

TEST(ObstacleTracking, CompactTrackEdgeFragmentNeverBecomesConfirmedObstacle)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  frame.obstacle_candidates.push_back(vehicle(2.5, 0.70, 0.20, 0.20));
  for (int cycle = 0; cycle < 8; ++cycle) {
    const auto result = planner.plan(frame, cycle * 0.05);
    EXPECT_FALSE(result.obstacle_confirmed);
    EXPECT_EQ(result.confirmed_obstacle_count, 0);
    EXPECT_EQ(result.passing_side, PassingSide::NONE);
  }
}

TEST(ObstacleTracking, BoundaryConnectedCompactClusterIsRejected)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  auto wall_fragment = vehicle(2.5, 0.30, 0.20, 0.20);
  wall_fragment.boundary_connected = true;
  frame.obstacle_candidates.push_back(wall_fragment);
  for (int cycle = 0; cycle < 8; ++cycle) {
    const auto result = planner.plan(frame, cycle * 0.05);
    EXPECT_EQ(result.confirmed_obstacle_count, 0);
    EXPECT_EQ(result.passing_side, PassingSide::NONE);
  }
}

TEST(ObstacleTracking, ClearLeftSpaceBuildsLeftPassingTrajectory)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  frame.obstacle_candidates.push_back(vehicle(2.5, -0.30));
  const auto result = confirm_obstacle(planner, frame);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.passing_side, PassingSide::LEFT);
  EXPECT_LT(result.official_steering_rad, 0.0);
}

TEST(ObstacleTracking, ClearRightSpaceBuildsRightPassingTrajectory)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  frame.obstacle_candidates.push_back(vehicle(2.5, 0.30));
  const auto result = confirm_obstacle(planner, frame);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.passing_side, PassingSide::RIGHT);
  EXPECT_GT(result.official_steering_rad, 0.0);
}

TEST(ObstacleTracking, NoPassingSpaceAndImminentCollisionEmergencyStops)
{
  PredictivePlanner planner({});
  auto frame = boundaries(0.0, true, true, 0.35);
  frame.obstacle_candidates.push_back(vehicle(0.45, 0.0, 0.30, 0.20));
  const auto result = confirm_obstacle(planner, frame);
  EXPECT_TRUE(result.emergency_stop);
  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.target_speed_mps, 0.0);
}

TEST(PredictiveTrajectory, HistoryExpiresIntoFaultInsteadOfBlindCruise)
{
  PredictivePlanner planner({});
  ASSERT_TRUE(planner.plan(boundaries(), 0.0).valid);
  BoundaryFrame invalid;
  const auto result = planner.plan(invalid, 0.6);
  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.target_speed_mps, 0.0);
}

TEST(ControlMath, AckermannUsesOfficialRightPositiveConvention)
{
  const auto right_turn = team_01_race::ackermann_joint_angles(0.3, 0.34, 0.18, 0.523599);
  EXPECT_LT(right_turn.first, 0.0);
  EXPECT_LT(right_turn.second, 0.0);
  EXPECT_GT(std::abs(right_turn.second), std::abs(right_turn.first));
  EXPECT_LE(std::abs(right_turn.first), 0.523599);
  EXPECT_LE(std::abs(right_turn.second), 0.523599);
}

TEST(FeedbackContribution, LimitsBothSignsAndPreservesFeedForwardAndSpeed)
{
  for (const double direction : {-1.0, 1.0}) {
    team_01_race::PlannerConfig unlimited_config;
    unlimited_config.max_feedback_steering_rad = 0.523599;
    PredictivePlanner unlimited(unlimited_config);
    PredictivePlanner limited({});
    const auto frame = boundaries(direction * 0.025, true, true, 1.2, direction * 0.8);
    const auto before = unlimited.plan(frame, 0.0);
    const auto after = limited.plan(frame, 0.0);
    ASSERT_TRUE(before.valid);
    ASSERT_TRUE(after.valid);
    EXPECT_GT(std::abs(after.raw_fb_component_rad), 0.10);
    EXPECT_NEAR(after.limited_fb_component_rad, direction * 0.10, 1e-9);
    EXPECT_DOUBLE_EQ(after.steering_fb_rad, before.steering_fb_rad);
    EXPECT_DOUBLE_EQ(after.steering_ff_rad, before.steering_ff_rad);
    EXPECT_DOUBLE_EQ(after.target_speed_mps, before.target_speed_mps);
    EXPECT_NEAR(after.desired_internal_steering_rad,
      after.steering_ff_rad + after.limited_fb_component_rad, 1e-9);
    EXPECT_LE(std::abs(after.official_steering_rad), 0.523599);
  }
}

TEST(FeedbackContribution, PreservesSmallCorrectionsAndValidatesLimit)
{
  PredictivePlanner planner({});
  const auto result = planner.plan(boundaries(0.0, true, true, 0.75, 0.1), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(std::abs(result.raw_fb_component_rad), 0.0);
  EXPECT_LT(std::abs(result.raw_fb_component_rad), 0.10);
  EXPECT_DOUBLE_EQ(result.limited_fb_component_rad, result.raw_fb_component_rad);
  for (const double limit : {0.0, -0.1, 0.6, std::numeric_limits<double>::quiet_NaN()}) {
    team_01_race::PlannerConfig config;
    config.max_feedback_steering_rad = limit;
    EXPECT_THROW(config.validate(), std::invalid_argument);
  }
}

TEST(PathReadiness, RequiresThreeNewObservedFramesAndResetsOnInvalid)
{
  team_01_race::PlannerConfig config;
  team_01_race::PathReadiness gate;
  PredictivePlanner planner(config);
  auto p = planner.plan(boundaries(), 0.0);
  EXPECT_FALSE(gate.update(p, config, true, 3));
  EXPECT_FALSE(gate.update(p, config, true, 3));
  EXPECT_EQ(gate.count(), 2);
  EXPECT_FALSE(gate.update(p, config, false, 3));
  EXPECT_EQ(gate.count(), 0);
  EXPECT_FALSE(gate.update(p, config, true, 3));
  EXPECT_FALSE(gate.update(p, config, true, 3));
  EXPECT_TRUE(gate.update(p, config, true, 3));
}

TEST(PathReadiness, RejectsSingleBoundaryAndHistoryAtStartup)
{
  team_01_race::PlannerConfig config;
  team_01_race::PathReadiness gate;
  PredictivePlanner planner(config);
  auto p = planner.plan(boundaries(0.0, true, false), 0.0);
  ASSERT_TRUE(p.valid);
  for (int i = 0; i < 5; ++i) EXPECT_FALSE(gate.update(p, config, true, 3));
  p = planner.plan(boundaries(), 0.1);
  ASSERT_TRUE(p.valid);
  team_01_race::BoundaryFrame missing;
  p = planner.plan(missing, 0.2);
  EXPECT_FALSE(gate.update(p, config, false, 3));
  planner.reset();
  EXPECT_FALSE(planner.plan(missing, 0.3).valid);
}

TEST(PathValidity, RejectsNonfiniteBoundaryAndInvalidTargets)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  frame.left[5].y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(planner.plan(frame, 0.0).valid);
  auto p = planner.plan(boundaries(), 0.1);
  ASSERT_TRUE(p.valid);
  p.trajectory[1].x = -1.0;
  EXPECT_FALSE(planner.path_valid(p.trajectory));
  p.trajectory[1].x = 0.2;
  p.trajectory[1].y = 100.0;
  EXPECT_FALSE(planner.path_valid(p.trajectory));
}

TEST(SteeringDirection, OffsetTargetsReturnTowardCenterWithOneInterfaceConversion)
{
  for (double offset : {-0.2, 0.2}) {
    PredictivePlanner planner({});
    const auto p = planner.plan(boundaries(0.0, true, true, 0.75, offset), 0.0);
    ASSERT_TRUE(p.valid);
    EXPECT_GT(p.target_y_m * offset, 0.0);
    EXPECT_GT(p.desired_internal_steering_rad * offset, 0.0);
    EXPECT_LT(p.official_steering_rad * offset, 0.0);
    const auto joints = team_01_race::ackermann_joint_angles(p.official_steering_rad, .34, .18, .523599);
    EXPECT_GT(joints.first * offset, 0.0);
    EXPECT_GT(joints.second * offset, 0.0);
  }
}

TEST(CenterlineRegression, PartialFarWallUsesMeasuredWidthWithoutArtificialBend)
{
  auto frame = boundaries(0.0, true, true, 1.2, 0.6);
  frame.left.erase(frame.left.begin() + 38, frame.left.end());
  PredictivePlanner planner({});
  const auto p = planner.plan(frame, 0.0);
  ASSERT_TRUE(p.valid);
  for (const auto & point : p.centerline) EXPECT_NEAR(point.y, 0.6, 1e-6);
  EXPECT_NEAR(p.predicted_curvature, 0.0, 1e-6);
}

TEST(CenterlineRegression, RejectedFramesNeverRenewOldTargetLifetime)
{
  PredictivePlanner planner({});
  for (int i = 0; i < 4; ++i) ASSERT_TRUE(planner.plan(boundaries(), i * .05).valid);
  auto jump = boundaries(0.0, true, true, 1.5, .8);
  auto p = planner.plan(jump, .2);
  ASSERT_TRUE(p.straight_guard_rejected);
  for (int i = 3; i <= 7; ++i) p = planner.plan(jump, i * .1);
  EXPECT_FALSE(p.valid);
  EXPECT_EQ(p.reason, "straight_guard_history_expired");
}

TEST(BoundaryCoordinates, ConnectedWallsKeepTheirSideWhenCrossingForwardAxis)
{
  // Ray-cast a straight corridor viewed at an angle: the left wall crosses
  // y=0 ahead. Both walls must retain their near-field identity.
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -team_01_race::kPi / 2;
  scan.angle_increment = team_01_race::kPi / 720;
  scan.range_min = .04;
  scan.range_max = 6;
  for (int i = 0; i <= 720; ++i) {
    const double a = scan.angle_min + i * scan.angle_increment;
    const double denominator = std::sin(a) + .5 * std::cos(a);
    const double r = denominator > 0 ? 1.2 / denominator : -1.2 / denominator;
    scan.ranges.push_back(r <= 6 ? r : std::numeric_limits<float>::infinity());
  }
  team_01_race::PlannerConfig config;
  auto frame = team_01_race::extract_predictive_frame(scan, config);
  ASSERT_TRUE(frame.scan_valid);
  EXPECT_TRUE(std::any_of(frame.left.begin(), frame.left.end(),
    [](const Point2 & p) {return p.y < -.12;}));
  PredictivePlanner planner(config);
  const auto p = planner.plan(frame, 0.0);
  ASSERT_TRUE(p.valid);
  EXPECT_LT(p.target_y_m, 0.0);
  EXPECT_GT(p.official_steering_rad, 0.0);
}

TEST(CenterlineRegression, IsolatedFartherSurfaceCannotPullSameXMidpoint)
{
  auto frame = boundaries(0.0, true, true, 1.2, .6);
  frame.right.push_back({1.81, -1.81});
  PredictivePlanner planner({});
  const auto p = planner.plan(frame, 0.0);
  ASSERT_TRUE(p.valid);
  for (const auto & point : p.centerline) EXPECT_NEAR(point.y, .6, 1e-5);
}

TEST(CenterlineRegression, RotatedCorridorUsesPerpendicularWidthAndContinuousRangeEdge)
{
  // Physical width remains 2.4 m at 55 degrees even though its vehicle-x
  // cross section is wider than the configured 4 m maximum.
  auto frame = boundaries(0.0, true, true, 2.1, .2);
  for (auto & point : frame.left) point.y -= 1.45 * point.x;
  for (auto & point : frame.right) point.y -= 1.45 * point.x;
  PredictivePlanner planner({});
  const auto p = planner.plan(frame, 0.0);
  ASSERT_TRUE(p.valid);
  EXPECT_LT(p.target_y_m, 0.0);
  EXPECT_GT(p.official_steering_rad, 0.0);
  EXPECT_TRUE(p.current_centerline_observed);
  for (const auto & point : p.centerline) {
    EXPECT_NEAR(point.y, .2 - 1.45 * point.x, .15);
  }
}

TEST(BoundaryCoordinates, FarFragmentsInheritTheNearWallSideAcrossMissingRays)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -team_01_race::kPi / 2;
  scan.angle_increment = team_01_race::kPi / 720;
  scan.range_min = .04;
  scan.range_max = 6;
  for (int i = 0; i <= 720; ++i) {
    const double a = scan.angle_min + i * scan.angle_increment;
    const double denominator = std::sin(a) + .5 * std::cos(a);
    double r = denominator > 0 ? 1.2 / denominator : -1.2 / denominator;
    const double x = r * std::cos(a);
    if (denominator > 0 && ((x > 3.6 && x < 3.85) || (x > 4.1 && x < 4.25))) {
      r = std::numeric_limits<double>::infinity();
    }
    scan.ranges.push_back(r <= 6 ? r : std::numeric_limits<float>::infinity());
  }
  team_01_race::PlannerConfig config;
  const auto frame = team_01_race::extract_predictive_frame(scan, config);
  EXPECT_TRUE(std::any_of(frame.left.begin(), frame.left.end(),
    [](const Point2 & p) {return p.x > 4.25 && p.y < 0;}));
  EXPECT_FALSE(std::any_of(frame.right.begin(), frame.right.end(),
    [](const Point2 & p) {return p.x > 4.25 && p.y > -1.2;}));
  PredictivePlanner planner(config);
  const auto p = planner.plan(frame, 0.0);
  ASSERT_TRUE(p.valid);
  EXPECT_LT(p.target_y_m, 0.0);
}

TEST(PathValidity, MeasuredBoundaryProtectsCurrentVehicleEnvelope)
{
  for (const double side : {-1.0, 1.0}) {
    PredictivePlanner planner({});
    const auto p = planner.plan(boundaries(0.0, true, true, .75, side * .55), 0.0);
    EXPECT_FALSE(p.valid);
    EXPECT_TRUE(p.emergency_stop);
    EXPECT_EQ(p.reason, "insufficient_current_boundary_clearance");
    EXPECT_DOUBLE_EQ(p.target_speed_mps, 0.0);
    EXPECT_NEAR(p.minimum_boundary_clearance_m, .20, 1e-6);
  }
}
