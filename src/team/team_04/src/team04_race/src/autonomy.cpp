#include "team04_race/lidar_planner.hpp"
#include "team04_race/traffic.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <array>
#include <cmath>
#include <algorithm>
class Autonomy:public rclcpp::Node {
 team04::PlannerConfig cfg;
 rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud;
 rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start;
 rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speedPub,steerPub,debugSpeed;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePub,profilePub;
 rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub,centerPub;
 rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr targetPub;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundaryPub,candidatePub;
 rclcpp::TimerBase::SharedPtr timer;
 bool permitted=false,seen=false;double speed=0,steering=0,timeout,acceleration,steerRate;
 double processMs=0,age=0,arrivalHz=0;uint64_t frames=0,validFrames=0,stops=0;
 std::string speedTopic,steerTopic,lastReason="STARTUP";
 std::chrono::steady_clock::time_point receipt{},lastReceipt{},lastDebug{};
 rclcpp::Time stamp{0,0,RCL_ROS_TIME};
 team04::Plan plan;
 team04::TrafficPlanner traffic;
 std::string trafficMode,lastTrafficState,lastTrafficRisk;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trafficPub;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objectsPub;
 public: Autonomy():Node("autonomy_04") {
  trafficMode=declare_parameter("traffic_mode",std::string("solo_racing"));
  if(trafficMode!="solo_racing" && trafficMode!="traffic_racing")throw std::invalid_argument("unknown traffic mode");
  trafficPub=create_publisher<std_msgs::msg::String>("/team04/debug/traffic_state",5);
  objectsPub=create_publisher<visualization_msgs::msg::MarkerArray>("/team04/debug/tracked_vehicles",1);
  cfg.visibility_reference_speed=declare_parameter("visibility_buffer_reference_speed_mps",3.5);
  cfg.visibility_buffer=declare_parameter("visibility_buffer_m",.8);
  cfg.path_mode=declare_parameter("path_mode",std::string("centerline"));cfg.reaction_time=declare_parameter("reaction_time_s",.23);
  cfg.max_speed=declare_parameter("max_speed",.3);
  double trafficCap=declare_parameter("traffic_max_speed",1.0);
  if(!std::isfinite(trafficCap)||trafficCap<=0||trafficCap>1.2)throw std::invalid_argument("traffic speed requires measured low-speed validation");
  if(trafficMode=="traffic_racing")cfg.max_speed=std::min(cfg.max_speed,trafficCap);
  cfg.lookahead_min=declare_parameter("lookahead_min",1.0);cfg.lookahead_gain=declare_parameter("lookahead_speed_gain",.6);cfg.lookahead_max=declare_parameter("lookahead_max",2.0);
  cfg.lateral_acceleration=declare_parameter("max_lateral_acceleration",.35);cfg.braking=declare_parameter("braking_deceleration",.7);cfg.safety_margin=declare_parameter("safety_margin",.16);
  cfg.min_obstacle_height=declare_parameter("min_obstacle_height",.035);cfg.max_obstacle_height=declare_parameter("max_obstacle_height",.38);
  cfg.planning_horizon=declare_parameter("planning_horizon",6.0);timeout=declare_parameter("sensor_timeout_s",.18);
  acceleration=declare_parameter("acceleration_limit",.3);steerRate=declare_parameter("steering_rate_limit",.65);
  cfg.acceleration=acceleration;cfg.steering_rate=steerRate;
  if(cfg.path_mode!="centerline" && cfg.path_mode!="racing_line")throw std::invalid_argument("unknown path mode");
  if(!std::isfinite(cfg.visibility_reference_speed)||cfg.visibility_reference_speed<=0)throw std::invalid_argument("invalid visibility calibration speed");
  if(!std::isfinite(cfg.visibility_buffer)||cfg.visibility_buffer<.2)throw std::invalid_argument("invalid measured-range contraction reserve");
  if(cfg.reaction_time<.18 || !std::isfinite(cfg.reaction_time))throw std::invalid_argument("reaction reserve must cover sensor timeout");
  for(double v:{cfg.max_speed,timeout,acceleration,steerRate,cfg.lookahead_min,cfg.lookahead_max,cfg.lookahead_gain,cfg.lateral_acceleration,cfg.braking,cfg.min_obstacle_height,cfg.max_obstacle_height,cfg.safety_margin,cfg.planning_horizon})if(!std::isfinite(v))throw std::invalid_argument("nonfinite autonomy parameter");
  if(!std::isfinite(cfg.max_speed)||cfg.max_speed<=0||cfg.max_speed>5||timeout<=0||timeout>.2||acceleration<=0||steerRate<=0||cfg.lookahead_min<.6||cfg.lookahead_max<cfg.lookahead_min||cfg.lateral_acceleration<=0||cfg.braking<=0||cfg.min_obstacle_height<=0||cfg.max_obstacle_height<=cfg.min_obstacle_height||cfg.safety_margin<.05||cfg.planning_horizon<1.2)throw std::invalid_argument("invalid autonomy safety parameters");
  speedTopic=declare_parameter("speed_topic",std::string("/team04/speed"));steerTopic=declare_parameter("steering_topic",std::string("/team04/steering"));
  speedPub=create_publisher<std_msgs::msg::Float32>(speedTopic,1);steerPub=create_publisher<std_msgs::msg::Float32>(steerTopic,1);
  debugSpeed=create_publisher<std_msgs::msg::Float32>("/team04/debug/target_speed",1);
  profilePub=create_publisher<std_msgs::msg::String>("/team04/debug/speed_profile",1);
  centerPub=create_publisher<nav_msgs::msg::Path>("/team04/debug/centerline",1);
  candidatePub=create_publisher<visualization_msgs::msg::MarkerArray>("/team04/debug/racing_candidates",1);
  statePub=create_publisher<std_msgs::msg::String>("/team04/debug/state",10);
  pathPub=create_publisher<nav_msgs::msg::Path>("/team04/debug/path",1);
  targetPub=create_publisher<geometry_msgs::msg::PointStamped>("/team04/debug/target_point",1);
  boundaryPub=create_publisher<visualization_msgs::msg::MarkerArray>("/team04/debug/track_boundaries",1);
  start=create_subscription<std_msgs::msg::Bool>(declare_parameter("race_start_topic",std::string("/race/start/team04")),rclcpp::QoS(10).reliable().durability_volatile(),[this](std_msgs::msg::Bool::ConstSharedPtr m){permitted=m->data;if(!permitted)halt("WAITING_FOR_START");});
  cloud=create_subscription<sensor_msgs::msg::PointCloud2>("/team04/points",rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){process(*m);});
  timer=create_wall_timer(std::chrono::milliseconds(20),[this]{watchdog();});
  halt("STARTUP");
 }
 void commands(){std_msgs::msg::Float32 a,b;a.data=speed;b.data=steering;steerPub->publish(b);speedPub->publish(a);debugSpeed->publish(a);}
 void diagnostics(const std::string& reason,bool force=false){
  auto t=std::chrono::steady_clock::now();if(!force && reason==lastReason && t-lastDebug<std::chrono::milliseconds(100))return;
  if(reason!=lastReason && reason!="TRACKING" && reason!="WAITING_FOR_START" && reason!="STARTUP")++stops;
  lastReason=reason;lastDebug=t;
  std::ostringstream s;s<<std::fixed<<std::setprecision(6)<<"{\"state\":\""<<reason<<"\",\"stamp\":"<<stamp.seconds()<<",\"sensor_age_s\":"<<age<<",\"arrival_hz\":"<<arrivalHz<<",\"processing_ms\":"<<processMs<<",\"target_speed\":"<<speed<<",\"steering\":"<<steering<<",\"lateral_error\":"<<plan.lateral_error<<",\"heading_error\":"<<plan.heading_error<<",\"curvature\":"<<plan.curvature<<",\"corridor_width\":"<<plan.width<<",\"free_distance\":"<<plan.free_distance<<",\"ground_z\":"<<plan.ground_c<<",\"obstacle_points\":"<<plan.obstacles.size()<<",\"boundary_pairs\":"<<plan.left.size()<<",\"path_mode\":\""<<plan.path_mode<<"\",\"racing_selected\":"<<(plan.racing_selected?"true":"false")<<",\"candidate_count\":"<<plan.candidate_count<<",\"observed_sparse_links\":"<<plan.observed_sparse_links<<",\"effective_lookahead_m\":"<<plan.effective_lookahead<<",\"visibility_buffer_m\":"<<plan.applied_visibility_buffer<<",\"predicted_time_s\":"<<plan.predicted_time<<",\"center_time_s\":"<<plan.center_time<<",\"min_clearance_m\":"<<plan.min_clearance<<",\"max_offset_m\":"<<plan.max_offset<<",\"visibility_speed_limit\":"<<plan.visibility_speed_limit<<",\"limiting_factor\":\""<<plan.limiting_factor<<"\",\"fallback_reason\":\""<<plan.fallback_reason<<"\",\"valid_fraction\":"<<(frames?double(validFrames)/frames:0)<<",\"safety_stops\":"<<stops<<"}";
  std_msgs::msg::String msg;msg.data=s.str();statePub->publish(msg);
 }
 void halt(const std::string& reason){speed=0;steering=0;commands();diagnostics(reason);}
 bool conflict(){return count_publishers(speedTopic)>1 || count_publishers(steerTopic)>1;}
 void watchdog(){
  if(!permitted){halt("WAITING_FOR_START");return;}
  if(conflict()){halt("COMMAND_PUBLISHER_CONFLICT");return;}
  if(!seen || std::chrono::duration<double>(std::chrono::steady_clock::now()-receipt).count()>timeout){halt("LIDAR_TIMEOUT");return;}
  double a=(now()-stamp).seconds();if(a>timeout || a<-.02){halt("LIDAR_STALE");return;}
 }
 void process(const sensor_msgs::msg::PointCloud2& m){
  const auto begin=std::chrono::steady_clock::now();const auto incoming=rclcpp::Time(m.header.stamp,get_clock()->get_clock_type());age=(now()-incoming).seconds();
  if(!std::isfinite(age)||age>timeout||age<-.02){halt("LIDAR_STALE");return;}
  if(seen && incoming<=stamp){if(incoming<stamp){permitted=false;seen=false;traffic.reset();halt("CLOCK_RESET");}return;}
  std::array<int,3> offset{{-1,-1,-1}};for(const auto&f:m.fields)for(size_t i=0;i<3;++i)if(f.name==std::array<std::string,3>{{"x","y","z"}}[i] && f.datatype==7 && f.count==1)offset[i]=f.offset;
  const uint16_t endian=1;const bool big=*reinterpret_cast<const uint8_t*>(&endian)==0;
  bool format=m.header.frame_id=="team04/lidar_link" && m.is_bigendian==big && m.point_step && uint64_t(m.row_step)>=uint64_t(m.width)*m.point_step && m.data.size()>=uint64_t(m.row_step)*m.height && uint64_t(m.width)*m.height<=1000000;
  for(int o:offset)format=format && o>=0 && uint32_t(o+4)<=m.point_step;
  if(!format){seen=false;halt("INVALID_CLOUD_FORMAT");return;}
  const double dt=seen?std::clamp((incoming-stamp).seconds(),.001,.1):.05;
  if(lastReceipt.time_since_epoch().count()){double delta=std::chrono::duration<double>(begin-lastReceipt).count();if(delta>0)arrivalHz=arrivalHz==0?1/delta:.9*arrivalHz+.1/delta;}
  lastReceipt=begin;receipt=begin;stamp=incoming;seen=true;++frames;
  std::vector<team04::Point> points;points.reserve(size_t(m.width)*m.height);
  for(size_t y=0;y<m.height;++y)for(size_t x=0;x<m.width;++x){const auto p=&m.data[y*m.row_step+x*m.point_step];float v[3];for(int i=0;i<3;++i)std::memcpy(&v[i],p+offset[i],4);points.push_back({v[0],v[1],v[2]});}
  cfg.previous_steering=steering;
  try{plan=traffic.update(points,cfg,speed,steering,stamp.seconds(),permitted && trafficMode=="traffic_racing");}catch(const std::exception&){seen=false;halt("PLANNER_EXCEPTION");return;}
  processMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  if(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>timeout || (now()-stamp).seconds()>timeout){halt("PLANNING_DEADLINE_MISSED");return;}
  visualise();trafficDebug();if(plan.valid)++validFrames;
  if(!permitted){halt("WAITING_FOR_START");return;}
  if(conflict()){halt("COMMAND_PUBLISHER_CONFLICT");return;}
  if(!plan.valid){halt(plan.reason);return;}
  speed=std::clamp(plan.speed,speed-cfg.braking*dt,speed+acceleration*dt);steering=std::clamp(plan.steering,steering-steerRate*dt,steering+steerRate*dt);
  commands();diagnostics("TRACKING");
 }
 void trafficDebug(){
  const auto&d=traffic.debug();
  if(frames%2 && d.state==lastTrafficState && d.risk==lastTrafficRisk)return;
  lastTrafficState=d.state;lastTrafficRisk=d.risk;std::ostringstream s;s<<std::fixed<<std::setprecision(6);
  s<<"{\"stamp\":"<<stamp.seconds()<<",\"mode\":\""<<trafficMode<<"\",\"state\":\""<<d.state<<"\",\"risk\":\""<<d.risk<<"\",\"reason\":\""<<d.reason<<"\",\"processing_ms\":"<<d.processing_ms<<",\"gap_m\":"<<d.gap<<",\"ttc_s\":";
  if(std::isfinite(d.ttc))s<<d.ttc;else s<<"null";
  s<<",\"wall_motion\":"<<(d.wall_motion?"true":"false")<<",\"ego_yaw_increment\":"<<d.ego_yaw_increment<<",\"speed_cap\":"<<d.speed_cap<<",\"side\":"<<d.side<<",\"attempts\":"<<d.attempts<<",\"completed\":"<<d.completed<<",\"aborted\":"<<d.aborted<<",\"detections\":[";
  for(size_t i=0;i<d.detections.size();++i){if(i)s<<",";auto&t=d.detections[i];s<<"{\"temporary_id\":"<<i+1<<",\"x\":"<<t.x<<",\"y\":"<<t.y<<",\"range_m\":"<<t.range<<",\"bearing_rad\":"<<t.bearing<<",\"min_x\":"<<t.min_x<<",\"max_x\":"<<t.max_x<<",\"min_y\":"<<t.min_y<<",\"max_y\":"<<t.max_y<<",\"stamp\":"<<t.stamp<<",\"inside\":"<<(t.inside?"true":"false")<<",\"uncertainty\":"<<t.uncertainty<<",\"length\":"<<t.length<<",\"width\":"<<t.width<<",\"points\":"<<t.points<<",\"confidence\":"<<t.confidence<<"}";}
  s<<"],\"tracks\":[";
  visualization_msgs::msg::MarkerArray markers;visualization_msgs::msg::Marker clear;clear.action=visualization_msgs::msg::Marker::DELETEALL;markers.markers.push_back(clear);
  for(size_t i=0;i<d.tracks.size();++i){if(i)s<<",";auto&t=d.tracks[i];s<<"{\"id\":"<<t.id<<",\"x\":"<<t.x<<",\"y\":"<<t.y<<",\"vx\":"<<t.vx<<",\"vy\":"<<t.vy<<",\"closing\":"<<t.closing<<",\"hits\":"<<t.hits<<",\"missed_s\":"<<t.missed<<",\"uncertainty\":"<<t.uncertainty<<",\"length\":"<<t.length<<",\"width\":"<<t.width<<",\"observed\":"<<(t.observed?"true":"false")<<"}";
   visualization_msgs::msg::Marker m;m.header.frame_id="team04/lidar_link";m.header.stamp=stamp;m.ns="traffic_tracks";m.id=t.id;m.type=visualization_msgs::msg::Marker::CUBE;m.action=visualization_msgs::msg::Marker::ADD;m.pose.orientation.w=1;m.pose.position.x=t.x;m.pose.position.y=t.y;m.pose.position.z=-.04;m.scale.x=std::max(.55,t.length);m.scale.y=std::max(.25,t.width);m.scale.z=.165;m.color.r=1;m.color.g=.4;m.color.a=.5;m.lifetime=rclcpp::Duration::from_seconds(.2);markers.markers.push_back(m);
   m.id=1000+t.id;m.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;m.pose.position.z=.25;m.scale.z=.12;m.color.a=1;m.text=std::to_string(t.id)+" / "+std::to_string(std::hypot(t.x,t.y))+" m / "+std::to_string(t.closing)+" m/s";markers.markers.push_back(m);
  }
  // Display the same conservative occupancy used by the collision predictor.
  for(const auto&t:d.tracks)for(int step=0;step<=2;++step){
   const double future=step,unc=t.uncertainty+.025*future+.25*t.missed;
   visualization_msgs::msg::Marker m;m.header.frame_id="team04/lidar_link";m.header.stamp=stamp;m.ns="predicted_occupancy";m.id=int(t.id)*10+step;m.type=visualization_msgs::msg::Marker::CUBE;m.action=visualization_msgs::msg::Marker::ADD;m.pose.orientation.w=1;m.pose.position.x=t.x+t.vx*future;m.pose.position.y=t.y+t.vy*future;m.pose.position.z=-.04;m.scale.x=t.length+2*(.64+unc);m.scale.y=t.width+2*(.64+unc);m.scale.z=.165;m.color.r=1;m.color.g=.15;m.color.a=.12;m.lifetime=rclcpp::Duration::from_seconds(.2);markers.markers.push_back(m);
  }
  visualization_msgs::msg::Marker label;label.header.frame_id="team04/lidar_link";label.header.stamp=stamp;label.ns="traffic_risk";label.id=0;label.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;label.action=visualization_msgs::msg::Marker::ADD;label.pose.orientation.w=1;label.pose.position.x=1;label.pose.position.z=.8;label.scale.z=.14;label.color.r=1;label.color.g=1;label.color.a=1;label.text=d.state+" / "+d.risk+" / TTC "+(std::isfinite(d.ttc)?std::to_string(d.ttc):"inf");label.lifetime=rclcpp::Duration::from_seconds(.2);markers.markers.push_back(label);
  s<<"]}";std_msgs::msg::String msg;msg.data=s.str();trafficPub->publish(msg);objectsPub->publish(markers);
 }
 void visualise(){
  std_msgs::msg::Header h;h.frame_id="team04/lidar_link";h.stamp=stamp;
  nav_msgs::msg::Path msg;msg.header=h;
  for(const auto&p:plan.path){geometry_msgs::msg::PoseStamped pose;pose.header=h;pose.pose.position.x=p.x;pose.pose.position.y=p.y;pose.pose.orientation.w=1;msg.poses.push_back(pose);}pathPub->publish(msg);
  msg.poses.clear();for(const auto&p:plan.center_path){geometry_msgs::msg::PoseStamped pose;pose.header=h;pose.pose.position.x=p.x;pose.pose.position.y=p.y;pose.pose.orientation.w=1;msg.poses.push_back(pose);}centerPub->publish(msg);
  visualization_msgs::msg::MarkerArray candidates;int id=0;for(const auto&path:plan.candidates){visualization_msgs::msg::Marker marker;marker.header=h;marker.ns="racing_candidates";marker.id=id++;marker.type=visualization_msgs::msg::Marker::LINE_STRIP;marker.action=visualization_msgs::msg::Marker::ADD;marker.pose.orientation.w=1;marker.scale.x=.015;marker.color.a=.5;marker.color.g=.8;marker.color.b=.3;marker.lifetime=rclcpp::Duration::from_seconds(.2);for(const auto&p:path){geometry_msgs::msg::Point q;q.x=p.x;q.y=p.y;marker.points.push_back(q);}candidates.markers.push_back(marker);}candidatePub->publish(candidates);
  std::ostringstream profile;profile<<"{\"speed_mps\":[";for(size_t i=0;i<plan.speed_profile.size();++i){if(i)profile<<",";profile<<plan.speed_profile[i];}profile<<"],\"curvature_per_m\":[";for(size_t i=0;i<plan.curvature_profile.size();++i){if(i)profile<<",";profile<<plan.curvature_profile[i];}profile<<"]}";std_msgs::msg::String detail;detail.data=profile.str();profilePub->publish(detail);
  geometry_msgs::msg::PointStamped target;target.header=h;target.point.x=plan.target.x;target.point.y=plan.target.y;targetPub->publish(target);
  visualization_msgs::msg::MarkerArray markers;
  for(int i=0;i<2;++i){visualization_msgs::msg::Marker marker;marker.header=h;marker.ns="team04_observed_boundaries";marker.id=i;marker.type=visualization_msgs::msg::Marker::LINE_STRIP;marker.action=visualization_msgs::msg::Marker::ADD;marker.pose.orientation.w=1;marker.scale.x=.035;marker.color.a=1;marker.color.r=i==0?1:0;marker.color.b=i==1?1:0;marker.lifetime=rclcpp::Duration::from_seconds(.2);
   for(const auto&p:(i==0?plan.left:plan.right)){geometry_msgs::msg::Point v;v.x=p.x;v.y=p.y;marker.points.push_back(v);}markers.markers.push_back(marker);}
  boundaryPub->publish(markers);
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Autonomy>());rclcpp::shutdown();}
