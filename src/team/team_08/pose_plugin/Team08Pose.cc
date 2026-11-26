// Official TEAM08 model plugin using original Monaco-tested BattleReplay08 control core.
// The validated legacy/BattleReplay08.cc is preserved unmodified.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/msgs/float.pb.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
namespace urrc {
constexpr double LEN=.53, WID=.25, STRAIGHT_GAP=.50, CORNER_GAP=.70;
constexpr double STRAIGHT_TARGET=1.00, CORNER_TARGET=1.00;
constexpr double EARLY_HORIZON=2.5;
constexpr double CLEARANCE_EPS=1e-4;
constexpr double MILD_CURVE=.08, SHARP_CURVE=.18;
constexpr double SHARP_SPEED=5., MILD_SPEED=5.5;
constexpr double MAX_TEAM_SPEED=8.;
constexpr double PASS_TEAM_SPEED=10.;
constexpr double TEAM_ACCEL=5., TEAM_DECEL=2.;
constexpr double BASE_TEAM_SPEED=5.5;
constexpr double DEFENSE_RANGE=35., DEFENSE_COLLISION_HORIZON=.5;
constexpr double DEFENSE_COLLISION_STEP=.05;
constexpr double DEFENSE_SINGLE_TIME=1.5, DEFENSE_DOUBLE_TIME=2.0, DEFENSE_SETTLE=.4;
constexpr double DEFENSE_REACTION=.9, DEFENSE_REPLAN_INTERVAL=.05;
constexpr double DEFENSE_COOLDOWN=.2;
constexpr double NORMAL_LANE_CHANGE_SPEED=BASE_TEAM_SPEED;
double wrap(double a){return atan2(sin(a),cos(a));}
double ease(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
struct Pt {double s,x,y,z,yaw,width=0.;};
struct Pos {double x,y,z,yaw;};
struct Car {
 std::string name;gz::sim::Entity entity{gz::sim::kNullEntity};
 double s=0,v=0,lat=0,from=0,to=0,changeStart=0,changeDuration=2.5;
 bool changing=false;
 double lateralVelocity=0.; bool speedKnown=false;
};
class Team08Pose_08:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 public:
 void Configure(const gz::sim::Entity& entity,const std::shared_ptr<const sdf::Element>&,
                gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
   team.entity=entity;
   node.Subscribe("/team08/replay_go_08",&Team08Pose_08::Start,this);
   speedPub=node.Advertise<gz::msgs::Float>("/team08/speed_08");
   steerPub=node.Advertise<gz::msgs::Float>("/team08/steering_08");
   try {
     auto share=ament_index_cpp::get_package_share_directory("team08_race");
     for(auto name:{"monza","spa","monaco","cota","hungaroring","baku"}){
       std::ifstream in(share+"/maps/"+name+"/centerline.csv");
       std::string line;std::getline(in,line);std::vector<Pt> pts;
       while(std::getline(in,line)){
         std::replace(line.begin(),line.end(),',',' ');
         Pt p{};std::istringstream ss(line);
         if(ss>>p.s>>p.x>>p.y>>p.z>>p.yaw>>p.width)pts.push_back(p);
       }
       if(pts.size()>100)maps.push_back({name,std::move(pts)});
     }
   }catch(const std::exception&e){std::cerr<<"[TEAM08 ORIGINAL] "<<e.what()<<std::endl;}
 }
  void PreUpdate(const gz::sim::UpdateInfo& info,gz::sim::EntityComponentManager& ecm) override {
  if(info.paused)return;
   const double now=std::chrono::duration<double>(info.simTime).count();
   if(!ok&&!init(ecm,now))return;
   if(!startSignal.load()){
     if(!started)gz::sim::Model(team.entity).SetWorldPoseCmd(ecm,spawnPose);
     last=now;team.v=0.;publishControls(0,0,now);return;
   }
   if(!started){started=true;begin=now;last=now;lastCsv=now;
     joinPending=false;
     std::cout<<"[TEAM08 ORIGINAL] START "<<trackName<<std::endl;
   }
   const double elapsed=now-begin;
  lastElapsed=elapsed;
  // Use actual simulation-time delta. Limiting dt to 50ms dropped distance
  // whenever Gazebo advances in larger steps and caused visible stutter.
  // Run the unchanged Monaco controller at 200 Hz maximum. Gazebo can
  // update physics much faster; running all collision forecasts at every
  // 1 ms tick starves rendering and ROS pose/clock forwarding.
  // Keep actual accumulated simulation dt, never discard elapsed time.
  const double rawDt=now-last;
  if(rawDt<0.005)return;
  const double dt=std::clamp(rawDt,0.,.2);last=now;
  if(dt<=0)return;
  // Preserve previous poses BEFORE lateral and longitudinal motion.
  // Needed to orient each car along its actual diagonal travel tangent.
  const Pos previousTeam=pose(team), previousLead=pose(lead), previousRear=pose(rear);
  const std::array<double,3> previousS{{team.s,lead.s,rear.s}};
  refreshOpponents(ecm,now);
  // Keep race-wide success for reporting, not as a lock on future passes.
  // Track the named opponent even after it moves from lead to rear.
  if(passingActive){
    if(passTargetSeen && team.s-passTarget.s>LEN+STRAIGHT_TARGET){
      passed=true;
       postPassDefense=true;
       mergePending=true;mergeTargetName=passTargetName;
      ++completedPassCount;
      std::cout<<"[BATTLE] OVERTAKE COMPLETED target="<<passTargetName
               <<" count="<<completedPassCount<<std::endl;
      passingActive=false;
    }else if(!passTargetSeen){
      passingActive=false;
    }
  }
   double frontGap=hasLead?gap(team.s,lead.s):1e6,rearGap=hasRear?gap(rear.s,team.s):1e6;
  double frontTTC=team.v>lead.v+.01?std::max(0.,frontGap-LEN)/(team.v-lead.v):1e6;
  // Count only an actual approaching car still behind us; a car that has
  // passed must not wrap around the lap and appear to have TTC near zero.
  const bool rearBehind=hasRear && rearGap>LEN && rearGap<=DEFENSE_RANGE;
  double rearTTC=rearBehind && rear.v>team.v+.01?
    std::max(0.,rearGap-LEN)/(rear.v-team.v):1e6;
  // Search the braking horizon for the FIRST bend, not only the
  // largest curve. A distant hairpin must not force 7m/s immediately.
  // With maximum TEAM08 deceleration of 2m/s^2, v^2 <= v_bend^2+2*a*d.
  const double stoppingLookahead=std::max(16.0,std::max(team.v, PASS_TEAM_SPEED)*std::max(team.v, PASS_TEAM_SPEED)/(2.*TEAM_DECEL)+8.0);
  const double currentCurve=fabs(curve(team.s));
  double upcomingCurve=currentCurve;
  double curveSpeedRaw=PASS_TEAM_SPEED;
  const double sharpTarget=SHARP_SPEED;
  const double mildTarget=MILD_SPEED;
  double sharpDistance=1e6,mildDistance=1e6;
  for(double d=0.;d<=stoppingLookahead;d+=.5){
    const double k=fabs(curve(team.s+d));
    upcomingCurve=std::max(upcomingCurve,k);
    if(k>=SHARP_CURVE){
      sharpDistance=std::min(sharpDistance,d);
      const double available=std::max(0.,d-1.5);
      curveSpeedRaw=std::min(curveSpeedRaw,
        std::sqrt(sharpTarget*sharpTarget+2.*TEAM_DECEL*available));
    }else if(k>=MILD_CURVE){
      mildDistance=std::min(mildDistance,d);
      const double available=std::max(0.,d-1.);
      curveSpeedRaw=std::min(curveSpeedRaw,
        std::sqrt(mildTarget*mildTarget+2.*TEAM_DECEL*available));
    }
  }
  // Hold classification for a short period at the transition to prevent
  // repeated MILD / STRAIGHT switching at near-identical curvature samples.
  int rawMode=sharpDistance<1e5?2:(mildDistance<1e5?1:0);
  if(rawMode>previousCurveMode || previousCurveMode<0){
    previousCurveMode=rawMode;
    curveModeChangedAt=now;
  }else if(rawMode<previousCurveMode &&
           now-curveModeChangedAt>.5){
    previousCurveMode=rawMode;
    curveModeChangedAt=now;
  }
  const bool sharpAhead=sharpDistance<1e5;
  const bool mildAhead=mildDistance<1e5 || sharpAhead;
  const bool corner=currentCurve>=SHARP_CURVE;
  if(raceMode && trackName=="monaco" && corner)
    lastMonacoSharpAt=now;
  const double curvatureSpeedLimit=std::clamp(curveSpeedRaw,3.,PASS_TEAM_SPEED);
  // Starting a 2.5s overtake is unsafe if a sharp bend is within its
  // projected travel range, even if the current speed cap is still high.
  const bool overtakeZone=!sharpAhead &&
    currentCurve<SHARP_CURVE;
  // Proactive, repeatable passing, even when TTC is infinite at equal speed.
  const bool frontPassRequested=hasLead && leadSpeedKnown && overtakeZone &&
    frontGap>LEN && frontGap<=25. &&
    curvatureSpeedLimit>lead.v+.5 &&
    (!raceMode || team.s-teamStart>=5.0);
  if(passingActive && !overtakeZone)passingActive=false;
   const double defenseSpeed=std::min(BASE_TEAM_SPEED,curvatureSpeedLimit);
   // First merge in front of the named overtaken car, then follow nearest rear.
   const Car* defenseCar=hasRear?&rear:nullptr;
   if(mergePending){
     const Car* target=nullptr;
     for(const auto& observed:observedTraffic)
       if(observed.name==mergeTargetName){target=&observed;break;}
     if(target && target->s<team.s && team.s-target->s<=DEFENSE_RANGE)
       defenseCar=target;
     else {mergePending=false;mergeTargetName.clear();}
   }
   const double mergeGap=defenseCar?team.s-defenseCar->s:1e6;
   const double rearIntent=defenseCar?defenseCar->lat+
       std::clamp(defenseCar->lateralVelocity*DEFENSE_REACTION,-.32,.32):team.lat;
   const double nominalLane=std::clamp(std::round(rearIntent/std::max(.01,lateralOffset)),-1.,1.)*lateralOffset;
   const double defenseDuration=std::abs(nominalLane-team.lat)>1.25*lateralOffset
        ?DEFENSE_DOUBLE_TIME:DEFENSE_SINGLE_TIME;
   const double remainingDefense=team.changing && defenseChanging
        ?std::max(.1,team.changeStart+team.changeDuration-now):defenseDuration;
   const double defenseDistance=std::max(team.v,defenseSpeed)*remainingDefense+2.;
   const bool defenseZone=!corner && sharpDistance>defenseDistance;
   const bool defenseEligible=postPassDefense && !passingActive && !joinPending &&
       defenseZone && defenseCar && defenseCar->speedKnown &&
       mergeGap<=DEFENSE_RANGE && mergeGap>=LEN;
   const double defenseRoadLimit=roadOffsetLimitAhead(team.s,defenseDistance);
   const double defenseLane=std::clamp(nominalLane,-defenseRoadLimit,defenseRoadLimit);
   const bool defenseNeedsChange=std::abs(defenseLane-team.lat)>.20*lateralOffset &&
       (mergePending || std::abs(rearIntent-defenseLane)+.06<std::abs(rearIntent-team.lat));
  if(previousCurveMode!=lastLoggedCurveMode){
    lastLoggedCurveMode=previousCurveMode;
    std::cout<<"[BATTLE] CORNER_SPEED mode="
             <<(previousCurveMode==2?"SHARP":(previousCurveMode==1?"MILD":"STRAIGHT"))
             <<" cap="<<curvatureSpeedLimit<<" curve="<<upcomingCurve
             <<" sharp_distance="<<sharpDistance
             <<" speed="<<team.v<<std::endl;
  }
  const double targetGap=corner?CORNER_TARGET:STRAIGHT_TARGET;
  minRearTTC=std::min(minRearTTC,rearTTC);
  if(defenseEligible && rearTTC<=5.){
    if(!defenseOpportunity){defenseStart=now;defenseOutcome="DETECTED";}
    if(!defenseOpportunity)std::cout<<"[BATTLE] REAR_PRESSURE TTC="<<rearTTC
      <<" gap="<<rearGap<<" team_speed="<<team.v<<" rear_speed="<<rear.v<<std::endl;
    defenseOpportunity=true;
  }
  // A change is a committed manoeuvre: keep its goal unless a *predicted
  // footprint collision* is detected. Negative clearance by itself is not
  // a collision and must not create oscillating replan requests.
  if(team.changing && defenseChanging){
     if(!defenseZone || !defenseCar || defenseCar->name!=defenseTargetName ||
        (now-lastDefenseCheck>=DEFENSE_REPLAN_INTERVAL &&
         !defensePathSafe(team.to,elapsed,now,true,defenseSpeed,
                          std::max(.1,team.changeStart+team.changeDuration-now)+DEFENSE_SETTLE,
                          team.changeDuration))){
       change(team.lat,now);team.changing=false;defenseChanging=false;
       changeFinishedAt=now;defenseOutcome="SAFE_ABORT";
       std::cout<<"[BATTLE] DEFENSE ABORT rear="<<defenseTargetName<<" sharp="<<!defenseZone<<std::endl;
     }
     if(now-lastDefenseCheck>=DEFENSE_REPLAN_INTERVAL)lastDefenseCheck=now;
   }else if(team.changing){
     const double risk=forecast(team.to,elapsed,now,true,
      std::min(plannedPassSpeed,curvatureSpeedLimit),2.5);
    if(risk<0.0 && now-lastReplan>1.0){
      lastReplan=now;
      const double stay=forecast(team.lat,elapsed,now,false);
      const double back=forecast(team.from,elapsed,now,false);
      if(back>0.0 && back>stay+0.15){
        change(team.from,now);replanCount++;
      }else if(stay>0.0){
        change(team.lat,now);replanCount++;
      }
      passingActive=false;
      emergencyBraking=true;
      std::cout<<"[BATTLE] HAZARD_REPLAN future="<<risk
               <<" stay="<<stay<<" back="<<back<<std::endl;
    }
  }else if(joinPending){
    // Apply forward-first joining to EVERY race3 grid. If the official
    // position is already in its corridor, joinPending is false.
    const bool initialStraightDone=!raceMode || team.s-teamStart>=5.0;
    if(initialStraightDone &&
       forecast(preferredLane,elapsed,now,false,
                std::min(team.v+1.0,MAX_TEAM_SPEED),2.5,true)>=0.){
      change(preferredLane,now);joinPending=false;
      std::cout<<"[BATTLE] JOIN initial lane="<<initialLane
               <<" distance_from_grid_m="<<(team.s-teamStart)
               <<" from_offset="<<team.lat
               <<" to_offset="<<preferredLane<<std::endl;
    }
  }else if(now-changeFinishedAt<(mergePending?0.:(defenseEligible?DEFENSE_COOLDOWN:.5))){
    // Hysteresis prevents immediate re-selection of the opposite lane.
  }else if(defenseEligible && defenseNeedsChange &&
             now-lastDefenseCheck>=DEFENSE_REPLAN_INTERVAL){
     lastDefenseCheck=now;
     if(defensePathSafe(defenseLane,elapsed,now,false,defenseSpeed,defenseDuration+DEFENSE_SETTLE,defenseDuration)){
       change(defenseLane,now,defenseDuration);defenseChanging=true;defenseTargetName=defenseCar->name;
       if(!defenseOpportunity){defenseOpportunity=true;defenseStart=now;}
       defenseOutcome="HOLDING";
       std::cout<<"[BATTLE] "<<(mergePending?"PASS_MERGE_START":"DEFENSE MATCH_LANE")<<" rear="<<defenseCar->name<<" collision_horizon="<<DEFENSE_COLLISION_HORIZON
                <<" rear_lat="<<defenseCar->lat<<" rear_vlat="<<defenseCar->lateralVelocity
                <<" target="<<defenseLane<<" gap="<<mergeGap
                <<" speed_target="<<defenseSpeed<<" duration="<<defenseDuration
                <<" curve="<<(currentCurve>=MILD_CURVE?"MILD":"STRAIGHT")<<std::endl;
     }else if(now-lastDefenseLog>1.){
       lastDefenseLog=now;defenseOutcome="SAFE_ABORT";
       std::cout<<"[BATTLE] DEFENSE BLOCK_REJECTED rear="<<defenseCar->name<<" gap="<<mergeGap<<std::endl;
     }
   }else if(defenseEligible && !defenseNeedsChange && !(hasLead && frontGap<8.)){
     if(mergePending && defensePathSafe(team.lat,elapsed,now,false,defenseSpeed,DEFENSE_SINGLE_TIME+DEFENSE_SETTLE,DEFENSE_SINGLE_TIME)){
       mergePending=false;std::cout<<"[BATTLE] PASS_MERGE_COMPLETE target="<<mergeTargetName<<std::endl;
       mergeTargetName.clear();
     }
     if(now-lastDefenseLog>2.){
       lastDefenseLog=now;
       std::cout<<"[BATTLE] DEFENSE HOLD_LANE rear="<<rear.name
                <<" target="<<team.lat<<" speed_target="<<defenseSpeed<<std::endl;
     }
   }else if(frontPassRequested && !passingActive && !mergePending && now-lastPassPlan>=.2){
    lastPassPlan=now;
    const double passingLimit=std::min(PASS_TEAM_SPEED,curvatureSpeedLimit);
    double bestScore=-1e9,bestTarget=team.lat,bestSpeed=0.;
    bool found=false;
    // Existing clear corridor is also a valid passing path; otherwise
    // evaluate the left, middle and right lanes before following.
    for(double dst:{team.lat,-lateralOffset,0.,lateralOffset}){
      if(fabs(dst-lead.lat)<WID+.15)continue;
      if(dst!=team.lat && fabs(dst-team.lat)<.2*lateralOffset)continue;
      for(double trial:{10.,9.5,9.,8.5,8.,7.5,7.,6.5,6.,5.5,5.}){
        if(trial<=lead.v+.5 || trial>passingLimit+.01)continue;
        const double score=forecast(dst,elapsed,now,false,trial,3.0,true,
                                    mildAhead?CORNER_GAP:STRAIGHT_GAP);
        const double utility=.25*std::min(score,1.5)-
                             .10*fabs(dst-team.lat)+.50*(trial-lead.v);
        if(score>=0. && utility>bestScore){
          found=true;bestScore=utility;bestTarget=dst;bestSpeed=trial;
        }
      }
    }
    if(found){
      if(fabs(bestTarget-team.lat)>.2*lateralOffset)change(bestTarget,now);
      attempted=true;++attemptCount;
      plannedPassSpeed=bestSpeed;
      passTargetName=lead.name;passTarget=lead;passTargetSeen=true;
      passingActive=true;
      std::cout<<"[BATTLE] OVERTAKE ATTEMPT target="<<passTargetName
               <<" TTC="<<frontTTC<<" speed="<<bestSpeed
               <<" target_offset="<<bestTarget<<" front_gap="<<frontGap
               <<" score="<<bestScore<<std::endl;
    }
  }
  // Only a final lap-end result counts as a verified defense. Holding a
  // corridor for two seconds is preliminary evidence, not a race outcome.
  if(defenseOpportunity){
    if(rear.s>team.s+LEN){
      if(!rearOvertook)std::cout<<"[BATTLE] REAR OVERTAKE at t="<<elapsed
        <<" rear_s="<<rear.s<<" team_s="<<team.s<<std::endl;
      rearOvertook=true;
      defended=false;
      defenseOutcome="OVERTAKEN";
    }else if(defenseOutcome=="HOLDING" &&
             now-defenseStart>=2.0 && !rearOvertook){
      defenseOutcome="HOLD_MONITOR";
      std::cout<<"[BATTLE] DEFENSE HOLD_MONITOR t="<<elapsed
               <<" rear_gap="<<rearGap<<std::endl;
    }
  }
  if(team.changing){
   const double t=(now-team.changeStart)/team.changeDuration;
   team.lat=team.from+(team.to-team.from)*ease(t);
   if(t>=1){
      team.lat=team.to;team.changing=false;changeFinishedAt=now;
      if(defenseChanging && mergePending && defenseTargetName==mergeTargetName){
        mergePending=false;
        std::cout<<"[BATTLE] PASS_MERGE_COMPLETE target="<<mergeTargetName<<std::endl;
        mergeTargetName.clear();
      }
      defenseChanging=false;
    }
  }
  if(raceMode){
    // Restrict lateral travel using the smallest road width over the
    // next 3 seconds. This is geometric lane limiting, not a collision waiver.
    const double safe=roadOffsetLimitAhead(team.s,std::max(8.,team.v*3.));
    const double old=team.lat;
    team.lat=std::clamp(team.lat,-safe,safe);
    if(team.changing)team.to=std::clamp(team.to,-safe,safe);
    if(std::fabs(old-team.lat)>1e-5){
      team.from=team.lat;team.to=team.lat;
      team.changing=false;defenseChanging=false;changeFinishedAt=now;
       passingActive=false;
      if(now-lastWallCorrectionLog>1.){
        lastWallCorrectionLog=now;
        std::cout<<"[BATTLE] ROAD_CORRECTION vehicle=team_08"
                 <<" from="<<old<<" to="<<team.lat
                 <<" safe_limit="<<safe<<std::endl;
      }
    }
  }
  // Front-to-rear car-body gaps; cars in other lanes do not force matching.
  auto cruise=[&](const Car &a,double limit){
    double desired=limit;
    for(const Car*b:{&team,&lead,&rear}){
      if(b==&a||(b==&lead&&!hasLead)||(b==&rear&&!hasRear))continue;
      const double d=gap(a.s,b->s);
      if(d>20. || fabs(a.lat-b->lat)>WID+.09)continue;
      const double bumper=std::max(0.,d-LEN);
      const bool inCorner=fabs(curve(a.s))>.18;
      // TEAM11 follows at a preferred 1.0m body-to-body gap.
      // Practice traffic retains its existing own driving policy.
      const double minimum=(&a==&team)?(inCorner?CORNER_TARGET:STRAIGHT_TARGET):
                          (inCorner?CORNER_GAP:STRAIGHT_GAP);
      const double ttc=a.v>b->v+.05?(bumper-minimum)/(a.v-b->v):1e6;
      if(ttc<=1. || bumper<=minimum){
        desired=std::min(desired,b->v+std::max(0.,bumper-minimum)*1.2);
      }else if(inCorner && bumper<4.){
        desired=std::min(desired,b->v+std::max(0.,bumper-minimum)*.45);
      }else if(ttc<3.){
        desired=std::min(desired,b->v+std::max(0.,bumper-minimum)*.9);
      }
    }
    return std::clamp(desired,0.,limit);
  };
  // Three-lap race: normal pace 5.5m/s, normal cap 8m/s, safe passing may reach 10m/s.
  // A requested average is measured; it is not a hard speed controller.
  const double teamCruise=BASE_TEAM_SPEED;
  double vt=cruise(team,teamCruise),vl=cruise(lead,9),vr=cruise(rear,rearCruise);
  // Merely detecting a front car must not force a lower cruise target
  // if the actual path remains safe at normal speed.
  if(frontPassRequested && !passingActive && !joinPending &&
     forecast(team.changing?team.to:team.lat,elapsed,now,team.changing,
              std::min(BASE_TEAM_SPEED,curvatureSpeedLimit),2.5,true)>=0.)
    vt=std::min(BASE_TEAM_SPEED,curvatureSpeedLimit);
  // Boost is reserved for a collision-checked passing plan.
  double passingSpeedWindow=0.;
   bool mergeClearanceBoost=false;
  // Scripted rear driver anticipates a cornering car's pace well before
  // the physical 0.7m limit. Relative-speed braking at 2-3m was too late
  // on Monaco: 11m/s rear closed into a 9m/s corner-following TEAM11.
  for(const Car*ahead:{&team,&lead}){
     if(!hasRear||(ahead==&lead&&!hasLead))continue;
     const double d=gap(rear.s,ahead->s);
    if(d>24. || d<=0.)continue;
    const bool crossing=ahead==&lead;
    // In bends the body corners can be much closer than the difference
    // between centreline offsets suggests; include diagonal neighbours.
    const bool aligned=fabs(rear.lat-ahead->lat)<
      (d<10.0?WID+CORNER_TARGET+.15:WID+.20);
    if(!crossing && !aligned)continue;
    // Look ahead along both trajectories; the current instantaneous
    // curvature alone becomes true only after a vehicle enters the bend.
    const bool nearCorner=fabs(curve(rear.s))>.18 ||
                           fabs(curve(ahead->s))>.18 ||
                           fabs(curve(rear.s+rear.v*2.0))>.18 ||
                           fabs(curve(ahead->s+ahead->v*2.0))>.18;
    // Add following margin, rather than commanding a zero closing speed
    // only when the measured rear gap is already near the hard limit.
    const double desiredGap=(nearCorner?CORNER_TARGET:STRAIGHT_TARGET)+.55;
    const double free=d-LEN-desiredGap;
    const double closing=std::max(0.,rear.v-ahead->v);
    // Begin matching at speed-dependent range; no abrupt 0m/s command.
    const double anticipation=std::max(7.0,
      closing*3.0+closing*closing/(2.*4.0)+desiredGap);
    if(free<anticipation){
      // Anticipate pace several seconds in advance; the actual acceleration
      // controller below limits deceleration to 5m/s^2.
      const double permitted=ahead->v+
        std::clamp((free-1.4)*.22,-1.0,1.2);
      vr=std::min(vr,std::max(0.,permitted));
      if(now-lastPracticeYield>1.5){
        lastPracticeYield=now;
        std::cout<<"[BATTLE] PRACTICE_YIELD to="<<ahead->name
                 <<" gap="<<d<<" desired="<<vr
                 <<" corner="<<nearCorner
                 <<" target_gap="<<desiredGap<<std::endl;
      }
    }
  }
  // Lead crosses three lanes on a known 2-second S-curve. A comfortable
  // speed change should begin before the hard 0.5m/0.7m boundary.
  // Calculate an early target from predicted occupied footprints.
  if(now-lastComfortForecast>=.1){
    lastComfortForecast=now;
    comfortSpeed=teamCruise;
    bool foundComfort=false;
    for(double trial:{10.,9.5,9.,8.5,8.,7.5,7.,6.5,6.,5.5,5.,4.,3.}){
      if(trial>curvatureSpeedLimit+.01 || trial>vt+.25)continue;
      const double safe=forecast(team.changing?team.to:team.lat,
        elapsed,now,team.changing,trial,EARLY_HORIZON,false,
        corner?CORNER_TARGET:STRAIGHT_TARGET);
      if(safe>=0.){
        comfortSpeed=trial;foundComfort=true;break;
      }
    }
    if(!foundComfort){
      // A five-sample 2.5s forecast can reject all constant-speed
      // candidates when the S-car temporarily shares a lane. Allow a
      // modest, continuously rechecked 1s acceleration window rather
      // than locking both cars to exactly 9m/s forever.
      comfortSpeed=BASE_TEAM_SPEED;
      if(frontGap>LEN+STRAIGHT_TARGET+1.5 &&
         !team.changing &&
         forecast(team.lat,elapsed,now,false,BASE_TEAM_SPEED,1.0,false,
                  corner?CORNER_TARGET:STRAIGHT_TARGET)>=0.)
        comfortSpeed=BASE_TEAM_SPEED;
    }
    // Avoid unnecessary 2.5-second following when the S-car is safely
    // alongside in a different corridor and can be passed on the straight.
    // The one-second emergency forecast remains active regardless.
    if(overtakeZone && fabs(team.lat-lead.lat)>WID+STRAIGHT_TARGET &&
       gap(team.s,lead.s)>LEN+STRAIGHT_TARGET+3.0 &&
       forecast(team.lat,elapsed,now,team.changing,team.v,1.0,false)>=0.){
      comfortSpeed=teamCruise;
    }
  }
  if(!hasLead&&!hasRear)comfortSpeed=std::min(BASE_TEAM_SPEED,curvatureSpeedLimit);
   // Preserve a locally safe acceleration window; the existing 2.5s
  // following forecast otherwise repeatedly resets the limit to 9m/s.
  if(passingSpeedWindow>comfortSpeed)comfortSpeed=passingSpeedWindow;
  // Keep lane choice and speed together throughout a committed overtake.
  // Recheck the actual interpolated lane against both traffic vehicles
  // before lifting the normal leading-car following speed cap.
  if(passingActive && passTargetSeen && overtakeZone){
    const double trial=std::min(plannedPassSpeed,curvatureSpeedLimit);
    const double predicted=forecast(team.changing?team.to:team.lat,
      elapsed,now,team.changing,trial,2.5,true,
      mildAhead?CORNER_GAP:STRAIGHT_GAP);
    if(trial>passTarget.v+.5 && predicted>=0.){
      passingSpeedWindow=trial;
      comfortSpeed=std::max(comfortSpeed,trial);
    }else{
      // Keep safety limits; allow another plan after the current lane change.
      passingActive=false;
    }
  }
  // The speed ceiling for approaching bends remains mandatory.
  comfortSpeed=std::min(comfortSpeed,curvatureSpeedLimit);
  // Record why a safe passing speed is not being selected, instead of
  // assuming that every no-overtake case is a lane-planner failure.
  if(hasLead&&!team.changing&&now-lastOvertakeDiagnostic>=5.0){
    lastOvertakeDiagnostic=now;
    std::cout<<"[BATTLE] PASS_DIAGNOSTIC time="<<elapsed
             <<" team_v="<<team.v<<" lead_v="<<lead.v
             <<" forward_gap="<<frontGap
             <<" lateral_delta="<<fabs(team.lat-lead.lat)
             <<" curve="<<curve(team.s)
             <<" comfort_limit="<<comfortSpeed
             <<" rear_ttc="<<rearTTC
             <<" lane_join_pending="<<joinPending
             <<" pass_window="<<passingSpeedWindow
             <<" speed_target="<<vt<<std::endl;
  }
  // Only a gentle approach for hazards 1-2.5s out, never an instant stop.
  vt=std::min(vt,std::max(team.v-1.5,comfortSpeed));
  // The same-lane cruise clamp is only a proximity heuristic. Lift it
  // through a collision-checked 1s speed window, never unconditionally.
  if(passingSpeedWindow>0. && comfortSpeed>=passingSpeedWindow)
    vt=std::max(vt,passingSpeedWindow);
   // Build enough return clearance before settling to steady defensive pace.
   if(mergePending && !defenseChanging && !team.changing && defenseZone && defenseCar &&
      defenseCar->speedKnown && mergeGap>LEN && mergeGap<DEFENSE_RANGE){
     const double trial=std::min(PASS_TEAM_SPEED,curvatureSpeedLimit);
     if(trial>BASE_TEAM_SPEED && forecast(team.lat,elapsed,now,false,trial,2.5,true)>=0.){
       vt=std::max(vt,trial);mergeClearanceBoost=true;
     }
   }
  vt=std::min(vt,curvatureSpeedLimit);
  // Ordinary lane joins are capped at the base speed. A committed pass
  // may use its planned speed only after validating the actual changing
  // path against BOTH predicted opponent trajectories.
  if(team.changing){
    const bool passClear=passingActive && overtakeZone &&
      forecast(team.to,elapsed,now,true,
        std::min(plannedPassSpeed,curvatureSpeedLimit),1.2,true,
        mildAhead?CORNER_GAP:STRAIGHT_GAP)>=0.;
    if(!passClear)vt=std::min(vt,NORMAL_LANE_CHANGE_SPEED);
    else if(now-lastPassMotionLog>3.){
      lastPassMotionLog=now;
      std::cout<<"[BATTLE] PASS_LANE_SPEED planned="<<plannedPassSpeed
               <<" target="<<vt<<" lat="<<team.lat
               <<" destination="<<team.to<<std::endl;
    }
  }
  if(emergencyBraking && frontTTC<=1.)vt=std::min(vt,5.0);
  // Restrict strong braking to hazards predicted inside one second.
  if(now-lastSpeedForecast>=.1){
    lastSpeedForecast=now;
    // The emergency ceiling is not the normal cruise target:
    // collision-checked passes must still be able to reach 10m/s.
    speedCeiling=PASS_TEAM_SPEED;
    if((hasLead||hasRear)&&forecast(team.changing?team.to:team.lat,elapsed,now,team.changing,team.v,1.,false)<0.){
      speedCeiling=0.;
      for(double trial:{10.,9.5,9.,8.5,8.,7.,6.,5.5,5.,4.,3.,1.,0.}){
        if(trial>vt+.01)continue;
        if(forecast(team.changing?team.to:team.lat,elapsed,now,team.changing,trial,1.,false)>=0.){
          speedCeiling=trial;break;
        }
      }
      if(speedCeiling<vt-1. && now-lastSpeedLog>=2.){
        lastSpeedLog=now;
        std::cout<<"[BATTLE] TTC_1S_BRAKE desired="<<vt<<" limit="<<speedCeiling
                 <<" front_ttc="<<frontTTC<<std::endl;
      }
    }
  }
  vt=std::min(vt,speedCeiling);
  if(hasLead&&now-lastSpeedDecisionLog>=3.0 &&
     frontGap<7. && team.v<lead.v+2.0){
    lastSpeedDecisionLog=now;
    std::cout<<"[BATTLE] SPEED_DECISION t="<<elapsed
             <<" current="<<team.v<<" target="<<vt
             <<" cruise="<<cruise(team,teamCruise)
             <<" comfort="<<comfortSpeed
             <<" one_sec_limit="<<speedCeiling
             <<" pass_window="<<passingSpeedWindow
             <<" team_changing="<<team.changing
             <<" front_gap="<<frontGap<<std::endl;
  }
  // All circuits use the same passing window; no independent Monaco boost.
  vt=std::min(vt,speedCeiling);
  // Braking for a pursuer behind would make a rear-end collision worse,
  // but only when the forward trajectory also remains clear.
  if(!postPassDefense && rearBehind && rearTTC<2. && frontTTC>2. &&
     !team.changing && speedCeiling>=team.v &&
     (!raceMode || forecast(team.lat,elapsed,now,false,
          std::min(curvatureSpeedLimit,team.v+0.5),1.5,true)>=0.))
    vt=std::max(vt,std::min(curvatureSpeedLimit,team.v+0.5));
  vt=std::min(vt,curvatureSpeedLimit);
   // Maintain base pace after passing; collision protection can reduce it.
   if(postPassDefense && !passingActive && !mergeClearanceBoost)vt=std::min(vt,BASE_TEAM_SPEED);
   if(raceMode && now-lastRaceHazardCheck>=.1){
    lastRaceHazardCheck=now;
    // The old emergency check only looked one second ahead and could
    // return speed=0 when the lead car was already beside TEAM11.
    // Check a longer window while braking is still physically possible.
    const double horizon=std::max(2.5,std::max(team.v,vt)/TEAM_DECEL+.5);
    const double dst=team.changing?team.to:team.lat;
    // Forward collision speed planning must not include an overtaken car.
    // Rear and side vehicles remain checked in lane-change forecasts.
    const double risk=forecast(dst,elapsed,now,team.changing,vt,horizon,false);
    if((hasLead||hasRear)&&risk<0.){
      bool found=false;
      for(double candidate:{10.,9.5,9.,8.5,8.,7.,6.,5.5,5.,4.,3.,2.,1.,0.}){
        if(candidate>vt+.01)continue;
        if(forecast(dst,elapsed,now,team.changing,candidate,horizon,false)>=0.){
          raceSafeTarget=candidate;
          found=true;
          break;
        }
      }
      if(!found){
        const bool actualFrontCar=hasLead&&lead.s>=team.s-LEN*.5;
        // A zero-speed target is reserved for a genuine forward hazard.
        // If an S-car has already been passed, do not create a rear impact.
        raceSafeTarget=actualFrontCar?0.:PASS_TEAM_SPEED;
      }
      if(now-lastRaceHazardLog>2.){
        lastRaceHazardLog=now;
        std::cout<<"[BATTLE] RACE3_EARLY_BRAKE risk="<<risk
                 <<" desired="<<vt<<" safe_target="<<raceSafeTarget
                 <<" team_v="<<team.v
                 <<" lead_v="<<lead.v
                 <<" body_gap="<<clearance(pose(team),pose(lead))
                 <<std::endl;
      }
    }else{
      raceSafeTarget=PASS_TEAM_SPEED;
    }
  }
  if(raceMode)vt=std::min(vt,raceSafeTarget);
   // Only a currently collision-checked pass may request the higher ceiling.
   const double activeCap=(mergeClearanceBoost || (passingActive && passTargetSeen && overtakeZone &&
                           passingSpeedWindow>0. && (!team.changing || vt>NORMAL_LANE_CHANGE_SPEED)))
                          ?PASS_TEAM_SPEED:MAX_TEAM_SPEED;
   vt=std::min(vt,std::min(activeCap,speedCeiling));
  if(!team.changing && frontTTC>1. && !joinPending)emergencyBraking=false;
  if((vl<8.9||vr<rearCruise-.1)&&elapsed-lastBrake>1.5){
    lastBrake=elapsed;braked=true;
    std::cout<<"[BATTLE] PRACTICE BRAKING lead="<<vl<<" rear="<<vr<<std::endl;
  }
  auto update=[dt](Car &a,double desired){
    // Smoothed speed transitions: integrate trapezoidally over real dt.
    const double before=a.v;
    a.v+=std::clamp(desired-a.v,-TEAM_DECEL*dt,TEAM_ACCEL*dt);
    a.s+=(before+a.v)*.5*dt;
  };
  update(team,std::clamp(vt,0.,activeCap));
  if(raceMode){
    // Reapply envelope after longitudinal motion, using next-lap geometry.
    team.lat=std::clamp(team.lat,-roadOffsetLimit(team.s),roadOffsetLimit(team.s));
    lead.lat=std::clamp(lead.lat,-roadOffsetLimit(lead.s),roadOffsetLimit(lead.s));
    rear.lat=std::clamp(rear.lat,-roadOffsetLimit(rear.s),roadOffsetLimit(rear.s));
  }
  if(raceMode){
    peakTeam=std::max(peakTeam,team.v);
    peakLead=std::max(peakLead,lead.v);
    peakRear=std::max(peakRear,rear.v);
    for(int i=completedTeamLaps+1;i<=raceLaps;++i){
      if(team.s-teamStart>=lap*i){
        completedTeamLaps=i;
        std::cout<<"[BATTLE] LAP_COMPLETED "<<i<<"/"<<raceLaps
                 <<" time="<<elapsed<<" team_speed="<<team.v<<std::endl;
      }else break;
    }
  }
  Pos a=pose(team),b=pose(lead),c=pose(rear);
  // Look-ahead tangent is less sensitive to microscopic position noise
  // than a one-frame heading; retain straight-road orientation at low speed.
  auto alignToMotion=[](Pos &next,const Pos &previous){
    const double dx=next.x-previous.x,dy=next.y-previous.y;
    if(std::hypot(dx,dy)>0.015){
      const double measured=std::atan2(dy,dx);
      next.yaw=previous.yaw+wrap(measured-previous.yaw)*0.35;
    }else next.yaw=previous.yaw;
  };
  alignToMotion(a,previousTeam);
  // Sample swept oriented rectangles to prevent tunneling.
  auto blend=[](const Pos&old,const Pos&next,double t){
    return Pos{old.x+(next.x-old.x)*t,old.y+(next.y-old.y)*t,
               old.z+(next.z-old.z)*t,
               old.yaw+wrap(next.yaw-old.yaw)*t};
  };
  for(int sweep=0;sweep<=10;sweep++){
    double t=static_cast<double>(sweep)/10.;
    Pos ap=blend(previousTeam,a,t);
    Pos bp=blend(previousLead,b,t);
    Pos cp=blend(previousRear,c,t);
    // Detect physical collision first and name the two vehicles involved.
    const std::array<Pos,3> snapshot{{ap,bp,cp}};
    const std::array<const char*,3> names{{"team_08","team_01","team_02"}};
    if(raceMode){
      const std::array<double,3> currentS{{team.s,lead.s,rear.s}};
      for(int i=0;i<3;++i){
        if((i==1&&!hasLead)||(i==2&&!hasRear))continue;
         const double sampleS=previousS[i]+t*(currentS[i]-previousS[i]);
        const double margin=roadMargin(snapshot[i],sampleS);
        minWallMargin=std::min(minWallMargin,margin);
        if(margin<=0.){
          ++wallWarningCount;
          if(now-lastWallWarningLog>1.){
            lastWallWarningLog=now;
            std::cout<<"[BATTLE] WALL_BOUNDARY_WARNING t="<<elapsed
                     <<" vehicle="<<names[i]<<" margin="<<margin
                     <<" sensor=GEOMETRIC_ROAD_ENVELOPE"
                     <<" physical_contact=NOT_MEASURED"<<std::endl;
          }
          // This is a road-width proxy, not a wall-contact event.
          // Continue and keep the geometric crossing visible in the report.
        }
      }
    }
    for(int i=0;i<3;++i){
      for(int j=i+1;j<3;++j){
       if((i==1&&!hasLead)||(i==2&&!hasRear)||(j==1&&!hasLead)||(j==2&&!hasRear))continue;
       const double dist=hypot(snapshot[i].x-snapshot[j].x,
                                snapshot[i].y-snapshot[j].y);
        minimumCenter=std::min(minimumCenter,dist);
        if(collide(snapshot[i],snapshot[j])){
          const bool scenarioOnly=i==1 && j==2;
          std::cout<<"[BATTLE] COLLISION t="<<elapsed<<" pair="<<names[i]
                   <<"/"<<names[j]<<" positions=("<<snapshot[i].x<<","
                   <<snapshot[i].y<<")/("<<snapshot[j].x<<","
                   <<snapshot[j].y<<")"<<std::endl;
          finish(scenarioOnly?"FAIL_SCENARIO_COLLISION":"FAIL_COLLISION");return;
        }
      }
    }
    const double clearanceLead=hasLead?clearance(ap,bp):1e9;
    const double clearanceRear=hasRear?clearance(ap,cp):1e9;
    minimumClearance=std::min(minimumClearance,
                              std::min(clearanceLead,clearanceRear));
    const double required=fabs(curve(team.s))>.18?CORNER_GAP:STRAIGHT_GAP;
    if(std::min(clearanceLead,clearanceRear)+CLEARANCE_EPS<required){
      const bool withLead=clearanceLead<=clearanceRear;
      if(!raceMode || now-lastCloseWarningLog>1.){
        lastCloseWarningLog=now;
        std::cout<<"[BATTLE] "<<(raceMode?"CLEARANCE_WARNING":"CLEARANCE_BREACH")
                 <<" t="<<elapsed
                 <<" pair=team_08/"<<(withLead?"team_01":"team_02")
                 <<" actual="<<std::min(clearanceLead,clearanceRear)
                 <<" required="<<required<<" team_s="<<team.s
                 <<" team_lat="<<team.lat<<" rear_ttc="<<rearTTC<<std::endl;
      }
      if(raceMode){
        if(sweep==0)++closeWarningCount;
      }else{
        finish("FAIL_CLEARANCE");return;
      }
    }
  }
  minimumClearance=std::min(minimumClearance,
      std::min(hasLead?clearance(a,b):1e9,hasRear?clearance(a,c):1e9));
  gz::sim::Model(team.entity).SetWorldPoseCmd(ecm,
     gz::math::Pose3d(a.x,a.y,a.z+.04,0,0,a.yaw));
   const double rate=wrap(a.yaw-previousTeam.yaw)/std::max(.001,dt);
   publishControls(team.v,std::clamp(std::atan2(rate*.34,std::max(.1,team.v)),-.52,.52),now);
   if(now-lastCsv>=.1){
    report<<std::fixed<<std::setprecision(4)<<elapsed<<','<<team.s<<','<<team.lat<<','<<team.v<<','
          <<lead.s<<','<<lead.lat<<','<<lead.v<<','<<rear.s<<','<<rear.lat<<','<<rear.v<<','
          <<frontTTC<<','<<rearTTC<<','<<minimumCenter<<','<<minimumClearance<<'\n';
    report.flush();lastCsv=now;
  }
  if(false&&raceMode&&team.s-teamStart>=raceLaps*lap){
    const double finishS=teamStart+raceLaps*lap;
    // A car ahead of the common finish line has already completed
    // three laps before TEAM11, so its final placing is known.
    const int rank=1+static_cast<int>(lead.s>=finishS)+
                     static_cast<int>(rear.s>=finishS);
    if(trackName=="monaco"){
      const double leadDiff=lead.s-team.s;
      const double rearDiff=rear.s-team.s;
      const std::string first=lead.s>=rear.s?"team_01":"team_02";
      const double winnerS=std::max(lead.s,rear.s);
      std::cout<<"[BATTLE] MONACO_FINISH_GAP leader="<<
        (winnerS>team.s?first:"team_08")
        <<" team_vs_lead_m="<<leadDiff
        <<" team_vs_rear_m="<<rearDiff
        <<" leader_ahead_m="<<std::max(0.,winnerS-team.s)
        <<std::endl;
    }
    const double average=(team.s-teamStart)/std::max(.001,elapsed);
    std::cout<<"[BATTLE] RACE3_RESULT PASS_3_LAPS"
             <<" rank="<<rank<<"/3 laps="<<raceLaps
             <<" team_peak_mps="<<peakTeam
             <<" team_average_mps="<<average
             <<" lead_peak_mps="<<peakLead
             <<" rear_peak_mps="<<peakRear
             <<" wall_margin_min_m="<<minWallMargin
             <<" wall_check=GEOMETRIC_ROAD_ENVELOPE"
             <<" min_body_clearance_m="<<minimumClearance
             <<" clearance_warning_count="<<closeWarningCount
             <<" wall_warning_count="<<wallWarningCount
             <<" elapsed_s="<<elapsed<<std::endl;
    finish("PASS_3_LAPS");return;
  }
  if(false&&!raceMode&&team.s-teamStart>=lap){
    // Defense is earned only if a true rear threat occurred and no pass
    // happened during the whole lap. Gap is measured between body edges.
    const double finalRearGap=team.s-rear.s-LEN;
    defended=defenseOpportunity && !rearOvertook &&
             finalRearGap>=(fabs(curve(team.s))>.18?CORNER_GAP:STRAIGHT_GAP);
    if(defended){
      defenseOutcome="HELD_TO_FINISH";
      std::cout<<"[BATTLE] DEFENSE VERIFIED_FINISH rear_gap="
               <<finalRearGap<<std::endl;
    }else if(rearOvertook)defenseOutcome="OVERTAKEN";
    else if(defenseOpportunity)defenseOutcome="UNVERIFIED_AT_FINISH";
    std::cout<<"[BATTLE] SUMMARY overtake="<<passed<<" defense="<<defended
             <<" defense_status="<<(defenseOpportunity?defenseOutcome:"NOT_TRIGGERED")
             <<" minimum_rear_ttc="<<minRearTTC<<" attempts="<<attemptCount
             <<" replans="<<replanCount<<" practice_braking="<<braked
             <<" minimum_center_m="<<minimumCenter
             <<" min_body_clearance_m="<<minimumClearance<<std::endl;
    finish(!passed?"FAIL_NO_OVERTAKE":(defended?"PASS_BATTLE":"PARTIAL_DEFENSE_UNVERIFIED"));return;
  }
  // The official operator owns race duration.
  previousRearTTC=rearTTC;
 }
 private:
 void publishControls(double v,double steer,double now){
   // ROS telemetry does not need the 200 Hz pose-controller rate.
   // 40 Hz reduces Gazebo->ROS bridge and monitor message pressure.
   if(now-lastTelemetryPublish<.025)return;
   lastTelemetryPublish=now;
   gz::msgs::Float a;a.set_data(static_cast<float>(v));speedPub.Publish(a);
   gz::msgs::Float b;b.set_data(static_cast<float>(steer));steerPub.Publish(b);
 }
 bool project(const gz::math::Pose3d&p,double&ss,double&lat) const{
   const double x=p.Pos().X(),y=p.Pos().Y();double best=1e30;
   for(size_t i=0;i+1<route.size();++i){
     const auto&a=route[i],&b=route[i+1];
     const double dx=b.x-a.x,dy=b.y-a.y;
     const double t=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/std::max(1e-9,dx*dx+dy*dy),0.,1.);
     const double px=a.x+t*dx,py=a.y+t*dy,d=std::hypot(px-x,py-y);
     if(d<best){best=d;ss=a.s+t*(b.s-a.s);
       lat=(x-px)*(-std::sin(a.yaw))+(y-py)*std::cos(a.yaw);}
   }
   return best<2.;
 }
 bool init(gz::sim::EntityComponentManager&ecm,double now){
   if(team.entity==gz::sim::kNullEntity||maps.empty())return false;
   spawnPose=gz::sim::worldPose(team.entity,ecm);
   const double x=spawnPose.Pos().X(),y=spawnPose.Pos().Y(),yaw=spawnPose.Rot().Yaw();
   // Nearest-XY matching picks Monaco on the official Monza grid 1:
   // that also causes the wrong map's Z elevation to be commanded.
   std::string activeWorld;
   ecm.Each<gz::sim::components::World,gz::sim::components::Name>(
     [&](const gz::sim::Entity& world,
         const gz::sim::components::World*,
         const gz::sim::components::Name* name)->bool {
       activeWorld=name->Data();worldEntity=world;return false;
     });
   if(activeWorld.empty())return false;
   double best=1e30,ss=0,lat=0;int selected=-1;
   for(size_t m=0;m<maps.size();++m) {
     if(maps[m].first!=activeWorld)continue;
     for(size_t i=0;i+1<maps[m].second.size();++i){
     const auto&a=maps[m].second[i],&b=maps[m].second[i+1];
     const double dx=b.x-a.x,dy=b.y-a.y;
     const double t=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/std::max(1e-9,dx*dx+dy*dy),0.,1.);
     const double px=a.x+t*dx,py=a.y+t*dy;
     const double score=std::hypot(px-x,py-y)+(std::abs(wrap(yaw-a.yaw))>.8?100.:0.);
     if(score<best){best=score;selected=static_cast<int>(m);ss=a.s+t*(b.s-a.s);
       lat=(x-px)*(-std::sin(a.yaw))+(y-py)*std::cos(a.yaw);}
   }
     }
   if(selected<0 || best>1.5){
     if(!worldMismatchReported){
       std::cerr<<"[TEAM08] World/track mismatch: world="<<activeWorld
                <<" projection_error_m="<<best
                <<" (fail closed; no route selected)"<<std::endl;
       worldMismatchReported=true;
     }
     return false;
   }
   trackName=maps[selected].first;route=maps[selected].second;
   lap=route.back().s+std::hypot(route.front().x-route.back().x,route.front().y-route.back().y);
   team.name="team_08";team.s=ss;team.lat=lat;teamStart=ss;
   preferredLane=lat;lateralOffset=.465;teamGrid=1;initialLane=2;
   raceMode=true;raceLaps=3;rearCruise=11.;
   lead.name="team_01";rear.name="team_02";
   lead.s=ss-100.;rear.s=ss-100.;last=now;
   report.open("/tmp/team08_official_motion.csv");
   if(!report)return false;
   report<<"time_s,team_s,team_lat,team_speed,lead_s,lead_lat,lead_speed,rear_s,rear_lat,rear_speed,front_ttc,rear_ttc,min_center_m,min_body_clearance_m\n";
   ok=true;
   std::cout<<"[TEAM08 ORIGINAL] BattleReplay08 core active map="<<trackName
            <<" start_s="<<ss<<" lat="<<lat<<std::endl;
   return true;
 }
 void refreshOpponents(gz::sim::EntityComponentManager&ecm,double now){
   // Only actual spawned entrants count. The nearest front and rear car
   // are selected dynamically; the original two-car forecast remains intact.
   hasLead=false;hasRear=false;leadSpeedKnown=false;passTargetSeen=false;
    observedTraffic.clear();
   double closestFront=1e9,closestRear=1e9;
   ecm.Each<gz::sim::components::Model,gz::sim::components::Name,
            gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity&e,const gz::sim::components::Model*,
          const gz::sim::components::Name*n,
          const gz::sim::components::ParentEntity*)->bool{
         const std::string name=n->Data();
         if(e==team.entity || name.size()!=7 || name.substr(0,5)!="team_" ||
            name[5]<'0'||name[5]>'9'||name[6]<'0'||name[6]>'9' ||
            name=="team_08")return true;
         const int number=(name[5]-'0')*10+(name[6]-'0');
         if(number<1||number>12)return true;
         double ss=0,lat=0;
         if(!project(gz::sim::worldPose(e,ecm),ss,lat))return true;
         const double relative=std::remainder(ss-team.s,lap);
         if(std::abs(relative)>90.)return true;
         // Use a multi-tick sample interval. The 200 Hz controller used
         // to overwrite the sample every tick, so dt>.01 rarely held.
         auto it=trafficHistory.find(name);
         if(it==trafficHistory.end()){
           trafficHistory[name]={ss,now};trafficLastLat[name]=lat;
         }else{
           const double dt=now-it->second.second;
           if(dt>=.05 && dt<2.){
             trafficSpeeds[name]=std::clamp(std::remainder(
                ss-it->second.first,lap)/dt,0.,30.);
             trafficLateralSpeeds[name]=std::clamp((lat-trafficLastLat[name])/dt,-2.,2.);
              trafficLastLat[name]=lat;it->second={ss,now};
            }else if(dt>=2. || dt<0.){
             trafficSpeeds.erase(name);trafficLateralSpeeds.erase(name);
              trafficLastLat[name]=lat;it->second={ss,now};
           }
         }
         const auto speedIt=trafficSpeeds.find(name);
         const bool speedKnown=speedIt!=trafficSpeeds.end();
         const double measuredSpeed=speedKnown?speedIt->second:0.;
          const auto latIt=trafficLateralSpeeds.find(name);
          const double lateralVelocity=latIt!=trafficLateralSpeeds.end()?latIt->second:0.;
          Car observed;observed.name=name;observed.entity=e;observed.s=team.s+relative;
          observed.lat=lat;observed.v=measuredSpeed;observed.lateralVelocity=lateralVelocity;observed.speedKnown=speedKnown;
          observedTraffic.push_back(observed);
          if(std::abs(relative)>DEFENSE_RANGE)return true;
         if(passingActive && name==passTargetName){
           passTargetSeen=true;passTarget.name=name;passTarget.entity=e;
           passTarget.s=team.s+relative;passTarget.lat=lat;passTarget.v=measuredSpeed;
         }
         Car* target=nullptr;
         if(relative>=0. && relative<closestFront){
           closestFront=relative;hasLead=true;leadSpeedKnown=speedKnown;target=&lead;
         }else if(relative<0. && -relative<closestRear){
           closestRear=-relative;hasRear=true;target=&rear;
         }
         if(target){
           target->name=name;target->entity=e;
           target->s=team.s+relative;
           target->lat=lat;target->v=measuredSpeed;
            target->lateralVelocity=lateralVelocity;target->speedKnown=speedKnown;
         }
         return true;
      });
 }
 // Conservative outer-lane limit. Reserve physical chassis width and 0.10m
 // yaw/curvature allowance; stronger than center-point-only clipping.
 double roadOffsetLimit(double s)const{
   double t=fmod(s,lap);if(t<0)t+=lap;
   auto it=std::upper_bound(route.begin(),route.end(),t,
      [](double x,const Pt&p){return x<p.s;});
   size_t i=it==route.begin()?0:static_cast<size_t>(it-route.begin()-1);
   size_t j=(i+1)%route.size();
   const double end=j==0?lap:route[j].s;
   const double f=std::clamp((t-route[i].s)/std::max(.001,end-route[i].s),0.,1.);
   const double width=route[i].width+
       (route[j].width-route[i].width)*f;
   return std::max(0.,.5*width-.5*WID-.10);
 }
 double roadOffsetLimitAhead(double s,double horizon)const{
   double safe=roadOffsetLimit(s);
   for(double d=1.;d<=horizon;d+=1.)
     safe=std::min(safe,roadOffsetLimit(s+d));
   return safe;
 }
 // Geometry-based road/wall envelope: not a Gazebo physics contact sensor.
 // Measures projected oriented chassis corners against local half-road-width.
 double roadMargin(const Pos&p,double s)const{
   double t=fmod(s,lap);if(t<0)t+=lap;
   auto it=std::upper_bound(route.begin(),route.end(),t,
     [](double x,const Pt&pt){return x<pt.s;});
   size_t i=it==route.begin()?0:static_cast<size_t>(it-route.begin()-1);
   const double width=2.*(roadOffsetLimit(s)+.5*WID+.10);
   Car center;center.s=s;center.lat=0.;
   const Pos ref=pose(center);
   const double nx=-sin(ref.yaw),ny=cos(ref.yaw);
   const double lateral=(p.x-ref.x)*nx+(p.y-ref.y)*ny;
   const double dyaw=wrap(p.yaw-ref.yaw);
   const double halfProjection=.5*LEN*fabs(sin(dyaw))+
                               .5*WID*fabs(cos(dyaw));
   return .5*width-fabs(lateral)-halfProjection;
 }
 double gap(double from,double to)const {
  double x=fmod(to-from,lap);if(x<0)x+=lap;return x;
 }
 Pos pose(const Car&a)const {
  double t=fmod(a.s,lap);if(t<0)t+=lap;
  auto it=std::upper_bound(route.begin(),route.end(),t,[](double x,const Pt&p){return x<p.s;});
  size_t i=it==route.begin()?0:static_cast<size_t>(it-route.begin()-1);
  size_t j=(i+1)%route.size();const Pt &p=route[i],&q=route[j];
  double end=j==0?lap:q.s;double f=std::clamp((t-p.s)/std::max(1e-6,end-p.s),0.,1.);
  double yaw=p.yaw+wrap(q.yaw-p.yaw)*f;
  return {p.x+(q.x-p.x)*f-a.lat*sin(yaw),
          p.y+(q.y-p.y)*f+a.lat*cos(yaw),
          p.z+(q.z-p.z)*f,yaw};
 }
 double curve(double s)const {
  double t=fmod(s,lap);if(t<0)t+=lap;
  auto it=std::lower_bound(route.begin(),route.end(),t,[](const Pt&p,double x){return p.s<x;});
  size_t i=it==route.end()?0:static_cast<size_t>(it-route.begin());
  const Pt&a=route[(i+route.size()-3)%route.size()],&b=route[(i+3)%route.size()];
  double d=gap(a.s,b.s);return d>.01?wrap(b.yaw-a.yaw)/d:0.;
 }
 bool collide(const Pos&a,const Pos&b)const {
  double dx=b.x-a.x,dy=b.y-a.y;
  for(double yaw:{a.yaw,b.yaw}){
    for(int k=0;k<2;k++){
      double ux=k==0?cos(yaw):-sin(yaw),uy=k==0?sin(yaw):cos(yaw);
      auto r=[&](const Pos&p){double cs=cos(p.yaw),sn=sin(p.yaw);
        return LEN*.5*fabs(ux*cs+uy*sn)+WID*.5*fabs(-ux*sn+uy*cs);};
      // Race3: only overlapping physical chassis footprints count.
      // Preserve the legacy conservative 25mm SAT cushion for one-lap tests.
      if(fabs(dx*ux+dy*uy)>r(a)+r(b)+(raceMode?0.:.025))
        return false;
    }
  }
  return true;
 }
 // Exact Euclidean edge-to-edge distance for two oriented rectangles.
 // A maximum SAT separating-axis gap is only a LOWER BOUND on distance
 // when corners are diagonal. It incorrectly rejected Monaco safe passes.
 double clearance(const Pos&a,const Pos&b)const{
   auto corners=[](const Pos&p){
     std::array<std::array<double,2>,4> out{};
     const double cs=cos(p.yaw),sn=sin(p.yaw);
     const double xs[4]={-LEN/2.,LEN/2.,LEN/2.,-LEN/2.};
     const double ys[4]={-WID/2.,-WID/2.,WID/2.,WID/2.};
     for(size_t i=0;i<4;++i){
       out[i]={p.x+cs*xs[i]-sn*ys[i],p.y+sn*xs[i]+cs*ys[i]};
     }
     return out;
   };
   const auto ca=corners(a),cb=corners(b);
   // Check intersection first; overlapping rectangles have zero distance.
   double dx=b.x-a.x,dy=b.y-a.y;
   bool overlap=true;
   for(double yaw:{a.yaw,b.yaw}){
     for(int axis=0;axis<2;++axis){
       const double ux=axis? -sin(yaw):cos(yaw);
       const double uy=axis? cos(yaw):sin(yaw);
       auto radius=[&](const Pos&p){
         const double cs=cos(p.yaw),sn=sin(p.yaw);
         return LEN*.5*fabs(ux*cs+uy*sn)+
                WID*.5*fabs(-ux*sn+uy*cs);
       };
       if(fabs(dx*ux+dy*uy)>radius(a)+radius(b)){
         overlap=false;break;
       }
     }
     if(!overlap)break;
   }
   if(overlap)return 0.;
   double best=1e9;
   auto measure=[&](const auto&points,const auto&edges){
     for(const auto&p:points){
       for(size_t j=0;j<4;++j){
         const auto&u=edges[j],&v=edges[(j+1)%4];
         const double ex=v[0]-u[0],ey=v[1]-u[1];
         const double denom=ex*ex+ey*ey;
         const double t=std::clamp(((p[0]-u[0])*ex+
                                   (p[1]-u[1])*ey)/denom,0.,1.);
         best=std::min(best,hypot(p[0]-u[0]-t*ex,
                                  p[1]-u[1]-t*ey));
       }
     }
   };
   measure(ca,cb);measure(cb,ca);
   return best;
 }
 // Predict both opponents' occupied footprints at future time points.
 // Nonnegative score means every predicted body-to-body clearance meets
 // its required limit. Never use centre separation as a substitute:
 // safe lateral passes may have short centre distances.
 double forecast(double destination,double elapsed,double now,bool active,double desiredSpeed=-1.,double horizon=5.,bool includeRear=true,double requiredGap=-1.,double plannedDuration=2.5,bool geometryOnly=false)const{
  double minimumMargin=1e6;
  Pos lastP=pose(team);
  double lastHeading=lastP.yaw;
  for(int i=1;i<=static_cast<int>(horizon*10+.5);i++){
    const double t=i*.1;
    Car candidate=team;
    const double vmax=desiredSpeed<0?std::max(0.,team.v):desiredSpeed;
    // Match the real TEAM08 controller: +5 / -2m/s^2. An
    // unrealistically fast predicted brake gives false safety clearance.
    const double acc=vmax<team.v?-TEAM_DECEL:TEAM_ACCEL;
    const double reach=fabs(vmax-team.v)/fabs(acc);
    const double until=std::min(t,reach);
    const double distance=team.v*until+.5*acc*until*until+
                          vmax*std::max(0.,t-until);
    candidate.s+=std::max(0.,distance);
    const double from=active?team.from:team.lat;
    const double start=active?team.changeStart:now;
    const double duration=active?team.changeDuration:plannedDuration;
    candidate.lat=from+(destination-from)*ease((now+t-start)/duration);
    if(raceMode &&
       fabs(candidate.lat)>roadOffsetLimit(candidate.s)+.01)return -3.;
    Pos p=pose(candidate);
    const double ds=hypot(p.x-lastP.x,p.y-lastP.y);
    if(ds>1e-4){
      p.yaw=atan2(p.y-lastP.y,p.x-lastP.x);
      if(i>1){
        const double k=fabs(wrap(p.yaw-lastHeading))/std::max(.001,ds);
        if(atan(.36*k)>30.*3.141592653589793/180. + .001)return -2.;
      }
      lastHeading=p.yaw;
    }
    if(geometryOnly){lastP=p;continue;}
    for(const Car*o:{&lead,&rear}){
     if((o==&lead&&!hasLead)||(o==&rear&&!hasRear))continue;
     // The lane-change predictor checks both front and rear vehicles.
      // Speed/braking prediction checks forward traffic only. An overtaken
      // S-car must not cause an emergency stop from behind.
      if(o==&rear && !includeRear)continue;
      if(raceMode && !includeRear && o->s<team.s-LEN*.5)continue;
      const double normalTravel=o->v*t;
       const bool ahead=o->s>=team.s-LEN*.5;
      // Consider sudden -5m/s^2 braking only for a car ahead.
      // Keep nominal motion and a possible full stop as distinct cases.
      const int scenarios=(raceMode && o==&lead && ahead &&
                           o->s-team.s<30.)?2:1;
      for(int scenario=0;scenario<scenarios;++scenario){
        Car sample=*o;
        if(scenario==0){
          sample.s+=normalTravel;
        }else{
          const double brakingTime=std::min(t,o->v/5.);
          sample.s+=std::max(0.,o->v*brakingTime-
                                2.5*brakingTime*brakingTime);
        }
        // Live competitor's lateral position is observed.
         if(raceMode)
          sample.lat=std::clamp(sample.lat,-roadOffsetLimit(sample.s),
                                             roadOffsetLimit(sample.s));
        const Pos op=pose(sample);
        const double need=fabs(curve(candidate.s))>.18?CORNER_GAP:STRAIGHT_GAP;
        const double threshold=raceMode?0.15:
          (requiredGap>=0?std::max(need,requiredGap):need);
        const double margin=clearance(p,op)-threshold;
        if(margin < -CLEARANCE_EPS)return -1.;
        minimumMargin=std::min(minimumMargin,margin);
      }
    }
    lastP=p;
  }
  return std::max(0.,minimumMargin);
 }
 // Defense forecast covers all nearby entrants and measured lateral motion.
  bool defensePathSafe(double destination,double elapsed,double now,bool active,
                       double desiredSpeed,double horizon,double plannedDuration)const{
   // Keep full-maneuver steering/road validation, without a long traffic veto.
   if(forecast(destination,elapsed,now,active,desiredSpeed,horizon,true,-1.,plannedDuration,true)<0.)return false;
   const double acc=desiredSpeed<team.v?-TEAM_DECEL:TEAM_ACCEL;
   const double reach=std::abs(desiredSpeed-team.v)/std::abs(acc);
   for(double t=0.;t<=horizon+.001;t+=DEFENSE_COLLISION_STEP){
     Car candidate=team;
     const double until=std::min(t,reach);
     candidate.s+=std::max(0.,team.v*until+.5*acc*until*until+desiredSpeed*std::max(0.,t-until));
     const double from=active?team.from:team.lat;
     const double start=active?team.changeStart:now;
     const double duration=active?team.changeDuration:plannedDuration;
     candidate.lat=from+(destination-from)*ease((now+t-start)/duration);
     if(std::abs(curve(candidate.s))>=SHARP_CURVE ||
        std::abs(candidate.lat)>roadOffsetLimit(candidate.s))return false;
     for(const auto&o:observedTraffic){
       // Only near-term collision risk can veto the traffic portion of defense.
       if(t>DEFENSE_COLLISION_HORIZON+1e-8)continue;
       if(!o.speedKnown && std::abs(o.s-team.s)<(30.+std::max(team.v,desiredSpeed))*DEFENSE_COLLISION_HORIZON+LEN)return false;
       if(!o.speedKnown)continue;
       for(int lateral=0;lateral<2;++lateral){
         for(int braking=0;braking<(o.s>=team.s?2:1);++braking){
           Car sample=o;
           const double stop=std::min(t,o.v/5.);
           sample.s+=braking?o.v*stop-2.5*stop*stop:o.v*t;
           if(lateral)sample.lat+=o.lateralVelocity*t;
           const double limit=roadOffsetLimit(sample.s);
           sample.lat=std::clamp(sample.lat,-limit,limit);
           if(clearance(pose(candidate),pose(sample))<=CLEARANCE_EPS)return false;
         }
       }
     }
   }
   return true;
  }
  void change(double dest,double now,double duration=2.5){
  team.from=team.lat;team.to=dest;team.changeStart=now;
  team.changeDuration=duration;team.changing=true;
 }
 void finish(const char*result){
  report.flush();
  if(raceMode && std::string(result)!="PASS_3_LAPS"){
    const double average=(team.s-teamStart)/std::max(.001,lastElapsed);
    std::cout<<"[BATTLE] RACE3_RESULT "<<result
             <<" rank=UNRANKED"
             <<" laps="<<completedTeamLaps<<"/"<<raceLaps
             <<" team_peak_mps="<<peakTeam
             <<" team_average_mps="<<average
             <<" lead_peak_mps="<<peakLead
             <<" rear_peak_mps="<<peakRear
             <<" wall_margin_min_m="<<minWallMargin
             <<" wall_check=GEOMETRIC_ROAD_ENVELOPE"
             <<" min_body_clearance_m="<<minimumClearance
             <<" clearance_warning_count="<<closeWarningCount
             <<" wall_warning_count="<<wallWarningCount
             <<" elapsed_s="<<lastElapsed<<std::endl;
  }
  std::cout<<"[BATTLE] RESULT "<<result<<" overtake="<<passed<<" defense="<<defended
           <<" defense_status="<<(defenseOpportunity?defenseOutcome:"NOT_TRIGGERED")
           <<" rear_ttc_min="<<minRearTTC<<" attempts="<<attemptCount
           <<" replans="<<replanCount<<" braked="<<braked
           <<" min_center="<<minimumCenter
           <<" min_body_clearance="<<minimumClearance<<std::endl;
 }
 void Start(const gz::msgs::Boolean&b){startSignal.store(b.data());}
 gz::sim::Entity worldEntity{gz::sim::kNullEntity};
 gz::transport::Node::Publisher speedPub,steerPub;
 gz::math::Pose3d spawnPose;
 std::vector<std::pair<std::string,std::vector<Pt>>> maps;
 std::unordered_map<std::string,std::pair<double,double>> trafficHistory;
 std::unordered_map<std::string,double> trafficSpeeds,trafficLastLat,trafficLateralSpeeds;
  std::vector<Car> observedTraffic;
  bool postPassDefense=false,defenseChanging=false,mergePending=false;
  std::string mergeTargetName;
  std::string defenseTargetName;
  double lastDefenseCheck=-1e6;
 bool leadSpeedKnown=false,passTargetSeen=false;
 Car passTarget;
 std::string passTargetName;
 double lastPassPlan=-1e6;
 int completedPassCount=0;
 bool hasLead=false,hasRear=false,worldMismatchReported=false;
 double leadObservedAt=0,rearObservedAt=0;
 double lastTelemetryPublish=-1e9;
 gz::transport::Node node;gz::transport::Node::Publisher ready;
 std::atomic<bool> startSignal{false};
 std::vector<Pt>route;Car team,lead,rear;
 std::ofstream report;double lap=0,lateralOffset=0,leadInitialLat=0,teamStart=0,begin=0,last=0,lastCsv=0,lastBrake=-10;
 int raceLaps=1,completedTeamLaps=0;
 bool raceMode=false;
 double peakTeam=0.,peakLead=0.,peakRear=0.,minWallMargin=1e9,lastElapsed=0.;
 double lastCloseWarningLog=-1e6,lastWallCorrectionLog=-1e6,lastWallWarningLog=-1e6;
 double lastRaceHazardCheck=-1e6,lastRaceHazardLog=-1e6,raceSafeTarget=PASS_TEAM_SPEED;
 long long closeWarningCount=0,wallWarningCount=0;
 double previousRearTTC=1e6,minRearTTC=1e6,minimumCenter=1e9,minimumClearance=1e9,preferredLane=0.;
 double lastReplan=-1e6,changeFinishedAt=-1e6,defenseStart=-1e6;
 double lastComfortForecast=-1e6,comfortSpeed=PASS_TEAM_SPEED,lastPracticeYield=-1e6,lastOvertakeDiagnostic=-1e6,lastPassSpeedLog=-1e6,lastSpeedDecisionLog=-1e6,lastPassMotionLog=-1e6;
 double lastSpeedForecast=-1e6,lastSpeedLog=-1e6,lastDefenseLog=-1e6,speedCeiling=PASS_TEAM_SPEED,rearCruise=11.,plannedPassSpeed=PASS_TEAM_SPEED;
 int attemptCount=0,replanCount=0,previousCurveMode=-1,lastLoggedCurveMode=-1;
 double curveModeChangedAt=-1e6;
 std::string defenseOutcome="NOT_TRIGGERED";
 std::string trackName;
 double lastMonacoSharpAt=-1e6,lastMonacoExitLog=-1e6,lastMonacoPassLog=-1e6;
 bool defenseOpportunity=false,rearOvertook=false;
 bool joinPending=false,emergencyBraking=false,passingActive=false;
 int initialLane=1,teamGrid=1;
 bool ok=false,started=false,ended=false,passed=false,defended=false,attempted=false,braked=false;
};
}
GZ_ADD_PLUGIN(urrc::Team08Pose_08,gz::sim::System,urrc::Team08Pose_08::ISystemConfigure,urrc::Team08Pose_08::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::Team08Pose_08,"urrc::Team08Pose_08")
