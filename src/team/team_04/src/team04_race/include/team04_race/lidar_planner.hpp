#pragma once
#include <vector>
#include <string>
#include <functional>
namespace team04 {
struct Point { double x=0,y=0,z=0; };
struct PlannerConfig {
 // Stage5 hooks default to the exact stage4 path. Used only by TrafficPlanner.
 bool boundary_only=false;
 int traffic_side=0;
 double traffic_travel=0;
 double stop_before_arc=1e100;
 const std::vector<Point>* additional_obstacles=nullptr;
 std::function<bool(const std::vector<Point>&,const std::vector<double>&)> dynamic_safe;
 std::string path_mode="centerline";
 double visibility_reference_speed=3.5,visibility_buffer=.8,reaction_time=.23,acceleration=.3,steering_rate=.65,previous_steering=0;
 double min_obstacle_height=.035,max_obstacle_height=.38;
 double planning_horizon=6.0;
 double vehicle_width=.24,safety_margin=.16,wheelbase=.32;
 double lookahead_min=1.0,lookahead_gain=.6,lookahead_max=2.0;
 double max_speed=.3,lateral_acceleration=.35,braking=.7;
};
struct Plan {
 unsigned rejected_boundary=0,rejected_raw_obstacle=0,rejected_dynamic=0;
 bool valid=false;std::string reason="NO_BOUNDARIES";
 std::vector<Point> obstacles,left,right,path;
 Point target;double steering=0,speed=0,lateral_error=0,heading_error=0;
 double curvature=0,width=0,free_distance=0,ground_a=0,ground_b=0,ground_c=0;
 unsigned observed_sparse_links=0,finite_points=0,ground_support=0;
 std::vector<Point> center_path;
 std::vector<std::vector<Point>> candidates;
 std::vector<double> candidate_times,speed_profile,curvature_profile;
 std::string path_mode="centerline",fallback_reason,limiting_factor;
 bool racing_selected=false;unsigned candidate_count=0;
 double applied_visibility_buffer=0,effective_lookahead=0,predicted_time=0,center_time=0,min_clearance=0,visibility_speed_limit=0,curvature_speed_limit=0,steering_speed_limit=0,max_offset=0;
};
Plan planObservedCorridorLidar(const std::vector<Point>& input,const PlannerConfig& cfg,double commanded_speed);
Plan planCenterlineLidar(const std::vector<Point>& input,const PlannerConfig& cfg,double commanded_speed);
Plan planLidar(const std::vector<Point>& input,const PlannerConfig& cfg,double commanded_speed);
}
