#pragma once
#include "team04_race/lidar_planner.hpp"
#include <limits>
namespace team04 {
struct Detection {
 unsigned id=0,points=0;double x=0,y=0,min_x=0,max_x=0,min_y=0,max_y=0;
 double length=0,width=0,range=0,bearing=0,confidence=0,uncertainty=.08,stamp=0,lateral_padding=.61;
 bool inside=false,vehicle_like=false,heading_supported=false;
};
struct Track:Detection {
 double vx=0,vy=0,closing=0,missed=0;unsigned hits=1,lateral_votes=0;int lateral_sign=0;bool observed=true;
};
struct TrafficDebug {
 bool wall_motion=false;double ego_yaw_increment=0;
 std::string state="RACING",risk="CLEAR",reason="no interior object";
 std::vector<Detection> detections;std::vector<Track> tracks;
 double processing_ms=0,ttc=std::numeric_limits<double>::infinity(),gap=1000,speed_cap=1000;
 unsigned attempts=0,completed=0,aborted=0;int side=0;
};
std::vector<Detection> detectVehicles(const Plan& observed,double stamp);
class VehicleTracker {
 std::vector<Track> objects;unsigned next_id=1;double last=-1;bool overflow=false;
 public:
 const std::vector<Track>& update(const std::vector<Detection>&,double stamp,double command,double steering,double wheelbase,double measured_yaw=std::numeric_limits<double>::quiet_NaN(),double measured_lateral=std::numeric_limits<double>::quiet_NaN());
 void reset();
 bool capacityExceeded()const{return overflow;}
};
struct WallObservation {bool valid=false;double heading=0,offset=0,slope=0,left_slope=0,right_slope=0,left_offset=0,right_offset=0;};
WallObservation observeWalls(const Plan&);
WallObservation refineWallsFromReturns(const Plan&,const WallObservation&);
bool predictedCollision(const std::vector<Point>& path,const std::vector<double>& speeds,const std::vector<Track>& tracks,double command,double max_speed,double margin);
double collisionTtc(const Track&,double command,double margin);
bool rearEnvelopeClear(const Track&,double margin);
class TrafficPlanner {
 VehicleTracker tracker;TrafficDebug debug_;int locked_side=0;unsigned target_id=0;
 double start_time=0,last_time=-1,attempted_travel=0;unsigned clear_frames=0;bool alongside=false,aborted_current=false;
 std::vector<Track> uncertain_cells;
 bool last_allow=false;double allow_since=0;
 bool capacity_fault=false;
 WallObservation previous_walls;
 public:
 Plan update(const std::vector<Point>&,const PlannerConfig&,double command,double steering,double stamp,bool allow_overtake);
 const TrafficDebug& debug()const{return debug_;}
 void reset();
};
}
