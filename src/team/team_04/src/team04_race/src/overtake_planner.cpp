#include "team04_race/traffic.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
namespace team04 {
namespace {
std::vector<Point> maskVehicleReturns(const std::vector<Point>&points,const Plan&raw,const std::vector<Track>&objects,const WallObservation&walls,const PlannerConfig&cfg){
 std::vector<Point> geometry;geometry.reserve(points.size());
 for(auto p:points){double h=p.z-raw.ground_a*p.x-raw.ground_b*p.y-raw.ground_c;bool object=false;
  if(h>=cfg.min_obstacle_height && h<=cfg.max_obstacle_height){
   bool staticWall=false;
   if(walls.valid){double l=std::abs(p.y-walls.left_slope*p.x-walls.left_offset)/std::hypot(1.,walls.left_slope),r=std::abs(p.y-walls.right_slope*p.x-walls.right_offset)/std::hypot(1.,walls.right_slope);staticWall=std::min(l,r)<.065;}
   if(!staticWall)for(const auto&t:objects)if(std::abs(p.x-t.x)<.74+.5*t.length+t.uncertainty && std::abs(p.y-t.y)<.74+.5*t.width+t.uncertainty){object=true;break;}
  }
  if(!object)geometry.push_back(p);
 }
 return geometry;
}
bool clearRays(const Track&t,const Plan&base){
 unsigned count=0;double range=std::hypot(t.x,t.y);if(range<.1)return false;
 for(const auto&p:base.obstacles){double r=std::hypot(p.x,p.y);if(r<range+.9)continue;double projection=(t.x*p.x+t.y*p.y)/(r*r);if(projection<=0 || projection>=1)continue;double x=p.x*projection,y=p.y*projection;
  if(std::hypot(x-t.x,y-t.y)>std::clamp(.5*std::hypot(t.length,t.width)+.08,.18,.40))continue;
  double h=p.z*projection-base.ground_a*x-base.ground_b*y-base.ground_c;
  if(h>.055 && h<.16 && ++count>=8)return true;
 }
 return false;
}
}
void TrafficPlanner::reset(){tracker.reset();debug_=TrafficDebug{};locked_side=0;target_id=0;start_time=0;last_time=-1;clear_frames=0;alongside=false;uncertain_cells.clear();last_allow=false;allow_since=0;aborted_current=false;attempted_travel=0;previous_walls=WallObservation{};capacity_fault=false;}
Plan TrafficPlanner::update(const std::vector<Point>&points,const PlannerConfig&cfg,double command,double steering,double stamp,bool allow){
 if(allow && !last_allow)allow_since=stamp;
 last_allow=allow;
 auto begin=std::chrono::steady_clock::now();Plan base=planLidar(points,cfg,command);Plan perception=base;
 auto classificationWalls=observeWalls(base);
 if(!classificationWalls.valid)classificationWalls=refineWallsFromReturns(base,previous_walls);
 if(!debug_.tracks.empty() && classificationWalls.valid){
  auto separated=maskVehicleReturns(points,base,debug_.tracks,classificationWalls,cfg);PlannerConfig only=cfg;only.boundary_only=true;only.lookahead_gain=0;only.lookahead_min=1;only.lookahead_max=1;
  auto measured=planObservedCorridorLidar(separated,only,command);
  if(measured.left.size()>=3 && measured.right.size()>=3){measured.obstacles=base.obstacles;perception=std::move(measured);}
 }
 auto detections=detectVehicles(perception,stamp);
 if(allow || locked_side || !detections.empty()){
  std::vector<Point> rearPoints;rearPoints.reserve(points.size());for(auto p:points)rearPoints.push_back({-p.x,-p.y,p.z});
  PlannerConfig rearConfig=cfg;rearConfig.boundary_only=true;rearConfig.lookahead_gain=0;rearConfig.lookahead_min=1;rearConfig.lookahead_max=1;
  auto rear=planObservedCorridorLidar(rearPoints,rearConfig,0);
  auto rearWalls=observeWalls(rear);
  if(!rearWalls.valid && classificationWalls.valid){
   auto prior=classificationWalls;std::swap(prior.left_slope,prior.right_slope);double oldLeft=prior.left_offset;prior.left_offset=-prior.right_offset;prior.right_offset=-oldLeft;prior.offset=-prior.offset;
   rearWalls=refineWallsFromReturns(rear,prior);
  }
  if(rearWalls.valid && !debug_.tracks.empty()){
   auto rearTracks=debug_.tracks;for(auto&t:rearTracks){t.x=-t.x;t.y=-t.y;}
   auto separated=maskVehicleReturns(rearPoints,rear,rearTracks,rearWalls,cfg);
   auto measured=planObservedCorridorLidar(separated,rearConfig,0);
   if(measured.left.size()>=3 && measured.right.size()>=3){measured.obstacles=rear.obstacles;rear=std::move(measured);}
  }
  for(auto d:detectVehicles(rear,stamp)){
   double minX=-d.max_x,maxX=-d.min_x,minY=-d.max_y,maxY=-d.min_y;
   if(maxX>-.2)continue;
   d.min_x=minX;d.max_x=maxX;d.min_y=minY;d.max_y=maxY;d.x=minX;d.y=-d.y;d.bearing=std::atan2(d.y,d.x);d.range=std::hypot(d.x,d.y);
   bool duplicate=false;
   for(const auto&f:detections)if(f.min_x-.15<=d.max_x && f.max_x+.15>=d.min_x && f.min_y-.15<=d.max_y && f.max_y+.15>=d.min_y){duplicate=true;break;}
   if(!duplicate)detections.push_back(d);
  }
 }
 auto previous=debug_.tracks;
 if(detections.size()>16)capacity_fault=true;
 if(capacity_fault){debug_.state="EMERGENCY_BRAKING";debug_.risk="SENSOR_UNCERTAIN";debug_.reason="object capacity exceeded; scene revalidation required";base.valid=false;base.speed=0;base.reason="TRAFFIC_CAPACITY_EXCEEDED";return base;}
 const double dt=last_time<0?.05:std::clamp(stamp-last_time,.001,.2);
 auto walls=observeWalls(perception);
 // Use only refitted CURRENT raw returns if the paired path is too short for
 // motion features. Retaining this measured fit also preserves actual walls
 // when masking vehicle returns for the next boundary extraction.
 if(!walls.valid)walls=classificationWalls;
 double yaw=std::numeric_limits<double>::quiet_NaN(),lateral=yaw;
 debug_.wall_motion=false;
 if(walls.valid && previous_walls.valid && last_time>=0 && stamp-last_time<=.1){
  double measured=std::atan2(std::sin(previous_walls.heading-walls.heading),std::cos(previous_walls.heading-walls.heading));
  double predicted=-command*std::tan(steering)/cfg.wheelbase*dt;
  if(std::abs(measured-predicted)<.005){
   yaw=.1*measured+.9*predicted;
   double sensed=previous_walls.offset+previous_walls.slope*command*dt-walls.offset*(std::cos(yaw)+previous_walls.slope*std::sin(yaw));
   double kinematic=std::abs(yaw)<1e-8?0:command*dt/yaw*(1-std::cos(yaw))+cfg.wheelbase*.5*std::sin(yaw);
   if(std::abs(sensed-kinematic)<.005)lateral=.1*sensed+.9*kinematic;
   debug_.wall_motion=true;
  }
 }
 previous_walls=walls;debug_.ego_yaw_increment=std::isfinite(yaw)?yaw:-command*std::tan(steering)/cfg.wheelbase*dt;
 auto objects=tracker.update(detections,stamp,command,steering,cfg.wheelbase,yaw,lateral);
 if(tracker.capacityExceeded()){capacity_fault=true;debug_.state="EMERGENCY_BRAKING";debug_.risk="SENSOR_UNCERTAIN";debug_.reason="track association capacity exceeded; reset required";base.valid=false;base.speed=0;base.reason="TRAFFIC_CAPACITY_EXCEEDED";return base;}
 last_time=stamp;
 if(locked_side)attempted_travel+=std::max(0.,command)*dt;
 for(auto&t:uncertain_cells){double yaw=-command*std::tan(steering)/cfg.wheelbase*dt,cs=std::cos(yaw),sn=std::sin(yaw),x=t.x-command*dt,y=t.y;t.x=cs*x+sn*y;t.y=-sn*x+cs*y;}
 for(const auto&t:previous)if(std::none_of(objects.begin(),objects.end(),[&](const Track&v){return v.id==t.id;}) && !clearRays(t,base)){
  if(uncertain_cells.size()>=16){capacity_fault=true;break;}
  auto cell=t;cell.id=0;uncertain_cells.push_back(cell);
 }
 if(capacity_fault){debug_.state="EMERGENCY_BRAKING";debug_.risk="SENSOR_UNCERTAIN";debug_.reason="unresolved occupancy capacity exceeded; reset required";base.valid=false;base.speed=0;base.reason="TRAFFIC_CAPACITY_EXCEEDED";return base;}
 uncertain_cells.erase(std::remove_if(uncertain_cells.begin(),uncertain_cells.end(),[&](const Track&t){return clearRays(t,base);}),uncertain_cells.end());
 debug_.detections=detections;debug_.tracks=objects;debug_.risk="CLEAR";debug_.gap=1000;debug_.ttc=std::numeric_limits<double>::infinity();debug_.speed_cap=cfg.max_speed;debug_.side=locked_side;
 auto finish=[&](Plan p){debug_.processing_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();return p;};
 bool rearUncertain=false;
 uncertain_cells.erase(std::remove_if(uncertain_cells.begin(),uncertain_cells.end(),[&](const Track&t){return !locked_side && t.x+.5*t.length+.64+t.uncertainty<-.42 && stamp-t.stamp>3.;}),uncertain_cells.end());
 bool frontUncertain=std::any_of(uncertain_cells.begin(),uncertain_cells.end(),[&](const Track&t){return locked_side || t.x+.5*t.length+.64+t.uncertainty>=-.42;});
 rearUncertain=!uncertain_cells.empty() && !frontUncertain;
 if(frontUncertain){
  debug_.state=locked_side?"ABORT_OVERTAKE":"EMERGENCY_BRAKING";debug_.risk="SENSOR_UNCERTAIN";debug_.reason="retired track requires measured rays through its former occupancy";base.valid=false;base.speed=0;base.reason="TRAFFIC_UNOBSERVED_OCCUPANCY";return finish(base);
 }
 if(objects.empty() && !locked_side){if(rearUncertain){base.speed=std::min(base.speed,command);debug_.risk="SENSOR_UNCERTAIN";}debug_.state="RACING";debug_.reason="no interior object; unchanged stage4 plan";return finish(base);}
 Track target;bool have=false;double nearest=1e9;
 for(const auto&t:objects){if(t.x>-.5 && t.x<nearest && t.inside){target=t;nearest=t.x;have=true;}if(t.id==target_id && locked_side){target=t;have=true;break;}}
 if(!have){
  debug_.state="RACING";debug_.reason="side/rear occupancy checked; no front target";if(rearUncertain){base.speed=std::min(base.speed,command);debug_.risk="SENSOR_UNCERTAIN";}
  // Side/rear objects still veto collisions even when there is no front target.
  if(base.valid && predictedCollision(base.path,base.speed_profile,objects,command,cfg.max_speed,cfg.safety_margin)){base.valid=false;base.speed=0;base.reason="TRAFFIC_LATERAL_COLLISION";debug_.risk="CRITICAL";debug_.state="EMERGENCY_BRAKING";}
  return finish(base);
 }
 debug_.gap=target.x-.5*target.length-.64-.26-target.uncertainty;debug_.ttc=collisionTtc(target,command,cfg.safety_margin);
 debug_.state="TARGET_DETECTED";debug_.risk="CAUTION";
 if(target.x>0 && std::abs(target.y)<.45 && debug_.ttc<cfg.reaction_time+std::abs(command)/cfg.braking){debug_.state="EMERGENCY_BRAKING";debug_.risk="CRITICAL";debug_.reason="TTC below reaction plus commanded braking time";base.valid=false;base.speed=0;base.reason="TRAFFIC_CRITICAL_TTC";return finish(base);}
 bool lost=!target.observed || target.hits<3;
 // Remove only measured interior object returns for boundary extraction. Restore
 // ALL raw obstacle points for the existing full-body collision certificate.
 auto geometry=maskVehicleReturns(points,base,objects,walls,cfg);
 PlannerConfig c=cfg;c.path_mode="racing_line";c.boundary_only=true;c.additional_obstacles=&base.obstacles;
 c.traffic_travel=locked_side?attempted_travel:0;
 if(locked_side)c.stop_before_arc=4.;
 c.dynamic_safe=[&](const std::vector<Point>&p,const std::vector<double>&v){return !predictedCollision(p,v,objects,command,c.max_speed,c.safety_margin);};
 // A confirmed ID can cross the side sensor gap between observations. Its
 // observed rear position proves that crossing; no unobserved or different ID
 // can authorize a return. The entire rear envelope still must be clear.
 if(locked_side && target.id==target_id && target.observed && target.x<1.1 && attempted_travel>1.5)alongside=true;
 if(locked_side && alongside && target.id==target_id && attempted_travel>1.5 && rearEnvelopeClear(target,cfg.safety_margin)){
  debug_.state="RETURN_TO_RACING_LINE";debug_.risk="CLEAR";debug_.reason="observed complete object envelope behind ego rear";
  auto result=planLidar(geometry,c,command);
  if(result.valid && std::abs(result.center_path.front().y)<.10){if(++clear_frames>=5){if(!aborted_current)++debug_.completed;locked_side=0;target_id=0;alongside=false;clear_frames=0;debug_.state="RACING";}}else clear_frames=0;
  return finish(result);
 }
 if(locked_side && (aborted_current || target.missed>.15 || target.hits<3 || stamp-start_time>20)){
  if(!aborted_current){++debug_.aborted;aborted_current=true;}
  debug_.state="ABORT_OVERTAKE";debug_.risk="SENSOR_UNCERTAIN";debug_.reason="hold chosen side; object lost or pass timed out";c.traffic_side=locked_side;c.stop_before_arc=4.;c.max_speed=std::min(.3,cfg.max_speed);auto p=planLidar(geometry,c,command);p.speed=std::min(p.speed,.3);return finish(p);
 }
 // Observed low-curvature corridor and three-frame association are prerequisites.
 bool planning=allow && !lost && target.hits<5;
 bool canPass=allow && stamp-allow_since>=.8 && command>=.25 && !lost && target.hits>=5 && target.observed && target.points>=6 && target.vehicle_like && std::max(base.curvature,perception.curvature)<.18 && target.x>2.5 && target.x<7.5;
 if(locked_side || canPass){
  // Receding local manoeuvre: certify every pose over at most four metres.
  // The unchanged stopping-distance/lookahead checks must fit inside this
  // measured interval. No safety check is deferred beyond its endpoint.
  c.stop_before_arc=4.;
  Plan best;std::string rejected;double score=1e100;int side=locked_side;
  for(int direction:{1,-1}){if(locked_side && direction!=locked_side)continue;c.traffic_side=direction;
   auto p=planLidar(geometry,c,command);if(!p.valid)rejected=p.reason+" "+p.fallback_reason;
   // A lead object may be beyond this local interval. Break symmetric path
   // costs toward the side with more observed target separation, while every
   // candidate still requires the complete local collision certificate.
   double separation=p.valid?std::abs(p.path.back().y-target.y)-.5*target.width-.64-target.uncertainty:0;
   double cost=p.predicted_time+.4/std::max(.1,separation);
   if(p.valid && cost<score){score=cost;best=std::move(p);side=direction;}}
  if(best.valid){
   if(!locked_side){locked_side=side;target_id=target.id;start_time=stamp;attempted_travel=0;++debug_.attempts;alongside=false;aborted_current=false;}
   if(std::abs(target.x)<1.1)alongside=true;
   if(!target.observed)best.speed=std::min(best.speed,command);
   debug_.state=alongside?"SIDE_BY_SIDE":locked_side>0?"OVERTAKE_LEFT":"OVERTAKE_RIGHT";debug_.risk=target.observed?"OVERTAKE":"SENSOR_UNCERTAIN";debug_.side=locked_side;debug_.reason="measured walls, full raw footprint and predicted occupancy certified";return finish(best);
  }
  if(locked_side){if(!aborted_current){++debug_.aborted;aborted_current=true;}debug_.state="ABORT_OVERTAKE";debug_.risk="CRITICAL";debug_.reason="chosen side lost collision certificate: "+rejected;best.valid=false;best.speed=0;best.reason="TRAFFIC_OVERTAKE_BLOCKED";return finish(best);}
  debug_.state="OVERTAKE_PLANNING";debug_.risk="OVERTAKE_CANDIDATE";
 }
 // Safe following path terminates BEFORE the inflated lead occupancy. Existing
 // wall, raw obstacle, dynamic footprint and stopping-distance checks all remain.
 c.traffic_side=0;double gap=std::max(0.,debug_.gap-cfg.safety_margin);
 c.stop_before_arc=std::max(0.,gap);double desired=2.0+.8*command+command*command/(2*cfg.braking);
 double lead=std::clamp(command-target.closing,0.,cfg.max_speed);
 double cap=std::clamp(lead+.65*(gap-desired),0.,cfg.max_speed);
 if(lost)cap=std::min(cap,std::max(0.,command-.1));
 c.max_speed=std::max(.05,cap);debug_.speed_cap=cap;
 auto result=planLidar(geometry,c,command);if(result.valid)result.speed=std::min(result.speed,cap);
 debug_.state=!result.valid?"EMERGENCY_BRAKING":target.hits<3?"TARGET_DETECTED":planning?"OVERTAKE_PLANNING":"FOLLOWING";debug_.risk=planning?"OVERTAKE_CANDIDATE":lost?"SENSOR_UNCERTAIN":result.valid?"FOLLOW":"CRITICAL";debug_.reason=result.valid?"following speed and stopping path limited by inflated occupancy":result.reason;return finish(result);
}
}
