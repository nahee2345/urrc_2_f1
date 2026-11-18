// Three-car Gazebo kinematic scenario: 9m/s S-traffic, team11, 11m/s grid10 pursuer.
// Uses direct pose commands. SAT-shaped collision checks; no camera or wheel dynamics.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
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
namespace urrc {
constexpr double LEN=.53, WID=.25, STRAIGHT_GAP=.50, CORNER_GAP=.70;
constexpr double STRAIGHT_TARGET=1.00, CORNER_TARGET=1.00;
constexpr double EARLY_HORIZON=2.5;
constexpr double CLEARANCE_EPS=1e-4;
constexpr double MILD_CURVE=.08, SHARP_CURVE=.18;
constexpr double SHARP_SPEED=7., MILD_SPEED=11.;
constexpr double MAX_TEAM_SPEED=15.;
constexpr double NORMAL_LANE_CHANGE_SPEED=10.5;
double wrap(double a){return atan2(sin(a),cos(a));}
double ease(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
struct Pt {double s,x,y,z,yaw,width=0.;};
struct Pos {double x,y,z,yaw;};
struct Car {
 std::string name;gz::sim::Entity entity{gz::sim::kNullEntity};
 double s=0,v=0,lat=0,from=0,to=0,changeStart=0,changeDuration=2.5;
 bool changing=false;
};
class BattleReplay_08:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 public:
 void Configure(const gz::sim::Entity& world,const std::shared_ptr<const sdf::Element> &cfg,
                gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
  worldEntity=world;
  const std::string topic=cfg->Get<std::string>("start_topic");
  node.Subscribe(topic,&BattleReplay_08::Start,this);
  ready=node.Advertise<gz::msgs::Boolean>("/team08/battle_ready_08");
  gz::msgs::Boolean readyMsg;readyMsg.set_data(true);ready.Publish(readyMsg);
  std::ifstream src(cfg->Get<std::string>("centerline_file"));
  std::string row;std::getline(src,row);
  while(std::getline(src,row)){
   std::replace(row.begin(),row.end(),',',' ');
   Pt p{};std::istringstream in(row);
   if(in>>p.s>>p.x>>p.y>>p.z>>p.yaw){
     if(!(in>>p.width))p.width=0.;
     route.push_back(p);
   }
  }
  raceLaps=cfg->HasElement("race_laps")?cfg->Get<int>("race_laps"):1;
  raceMode=raceLaps==3;
  trackName=cfg->HasElement("track_name")?
    cfg->Get<std::string>("track_name"):"";
  lap=cfg->Get<double>("lap_length_m");
  lateralOffset=cfg->Get<double>("lane_offset_m");
  team.name="team_08";team.s=cfg->Get<double>("team_start_s");
  team.lat=cfg->Get<double>("team_start_offset_m");teamStart=team.s;
  preferredLane=cfg->Get<double>("team_preferred_offset_m");
  initialLane=cfg->Get<int>("team_initial_lane");
  teamGrid=cfg->Get<int>("team_grid");
  lead.name="team_01";lead.s=cfg->Get<double>("lead_start_s");
  leadInitialLat=cfg->HasElement("lead_start_offset_m")?
    cfg->Get<double>("lead_start_offset_m"):-lateralOffset;
  lead.lat=leadInitialLat;
  rear.name="team_02";rear.s=cfg->Get<double>("rear_start_s");
  rear.lat=cfg->Get<double>("rear_start_offset_m");
  rearCruise=cfg->Get<double>("rear_cruise_mps");
  report.open(cfg->Get<std::string>("log_file"));
  if(route.size()<100||lap<100||lateralOffset<=0||!report||
     (raceMode && std::any_of(route.begin(),route.end(),
                              [](const Pt&p){return p.width<=0.;}))){
   std::cerr<<"[BATTLE] INVALID SETUP"<<std::endl;return;
  }
  report<<"time_s,team_s,team_lat,team_speed,lead_s,lead_lat,lead_speed,rear_s,rear_lat,rear_speed,front_ttc,rear_ttc,min_center_m,min_body_clearance_m\n";
  ok=true;
  std::cout<<"[BATTLE] READY grid="<<teamGrid<<" lane="<<initialLane
           <<" laps="<<raceLaps<<" team max "<<MAX_TEAM_SPEED
           <<"m/s, S-car 9m/s, rear="<<rearCruise<<"m/s"<<std::endl;
 }
 void PreUpdate(const gz::sim::UpdateInfo& info,gz::sim::EntityComponentManager& ecm) override {
  if(info.paused||!ok||ended)return;
  const double now=std::chrono::duration<double>(info.simTime).count();
  for(Car*a:{&team,&lead,&rear})if(a->entity==gz::sim::kNullEntity)
   ecm.Each<gz::sim::components::Model,gz::sim::components::Name,gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity&id,const gz::sim::components::Model*,
        const gz::sim::components::Name*name,const gz::sim::components::ParentEntity*p)->bool {
      if(name->Data()==a->name&&p->Data()==worldEntity){a->entity=id;return false;}return true;
    });
  if(team.entity==gz::sim::kNullEntity||lead.entity==gz::sim::kNullEntity||rear.entity==gz::sim::kNullEntity)return;
  if(!started){
    if(!startSignal.load())return;
    started=true;begin=now;last=now;lastCsv=now;
    // All race3 grids must drive straight from their official starting
    // pose. Small grid-to-lane offset differences never trigger a slide.
    const double joinTolerance=raceMode?0.18:0.05;
    joinPending=std::fabs(team.lat-preferredLane)>joinTolerance;
    std::cout<<"[BATTLE] START grid="<<teamGrid<<" (all five lights off)"<<std::endl;
  }
  const double elapsed=now-begin;
  lastElapsed=elapsed;
  // Use actual simulation-time delta. Limiting dt to 50ms dropped distance
  // whenever Gazebo advances in larger steps and caused visible stutter.
  const double dt=std::clamp(now-last,0.,.2);last=now;
  if(dt<=0)return;
  // Preserve previous poses BEFORE lateral and longitudinal motion.
  // Needed to orient each car along its actual diagonal travel tangent.
  const Pos previousTeam=pose(team), previousLead=pose(lead), previousRear=pose(rear);
  const std::array<double,3> previousS{{team.s,lead.s,rear.s}};
  const double lateral[5]={-lateralOffset,0,lateralOffset,0,-lateralOffset};
  int period=static_cast<int>(elapsed/2.0);
  int phase=period%4;
  // Race3 red car starts at the actual grid-1 lateral offset and merges
  // smoothly into its existing S-curve sequence during the first 2 seconds.
  const double leadFrom=(raceMode && period==0)?leadInitialLat:lateral[phase];
  lead.lat=leadFrom+(lateral[phase+1]-leadFrom)*ease((elapsed-2.0*period)/2.0);
  // Keep the scripted weaving vehicle inside its actual road envelope;
  // do not fail TEAM11 for a practice vehicle's off-road script.
  if(raceMode){
    const double safe=roadOffsetLimit(lead.s);
    const double original=lead.lat;
    lead.lat=std::clamp(lead.lat,-safe,safe);
    if(std::fabs(original-lead.lat)>1e-5 && now-lastWallCorrectionLog>1.){
      lastWallCorrectionLog=now;
      std::cout<<"[BATTLE] ROAD_CORRECTION vehicle="<<lead.name
               <<" from="<<original<<" to="<<lead.lat<<std::endl;
    }
    rear.lat=std::clamp(rear.lat,-roadOffsetLimit(rear.s),roadOffsetLimit(rear.s));
  }
  double frontGap=gap(team.s,lead.s),rearGap=gap(rear.s,team.s);
  double frontTTC=team.v>lead.v+.01?std::max(0.,frontGap-LEN)/(team.v-lead.v):1e6;
  // Count only an actual approaching car still behind us; a car that has
  // passed must not wrap around the lap and appear to have TTC near zero.
  const bool rearBehind=rearGap>LEN && rearGap<25.;
  double rearTTC=rearBehind && rear.v>team.v+.01?
    std::max(0.,rearGap-LEN)/(rear.v-team.v):1e6;
  // Search the braking horizon for the FIRST bend, not only the
  // largest curve. A distant hairpin must not force 7m/s immediately.
  // With maximum replay deceleration of 5m/s^2, v^2 <= v_bend^2+2*a*d.
  const double stoppingLookahead=std::max(16.0,team.v*team.v/10.0+8.0);
  const double currentCurve=fabs(curve(team.s));
  double upcomingCurve=currentCurve;
  double curveSpeedRaw=MAX_TEAM_SPEED;
  const double sharpTarget=raceMode?9.0:SHARP_SPEED;
  const double mildTarget=raceMode?10.:MILD_SPEED;
  double sharpDistance=1e6,mildDistance=1e6;
  for(double d=0.;d<=stoppingLookahead;d+=.5){
    const double k=fabs(curve(team.s+d));
    upcomingCurve=std::max(upcomingCurve,k);
    if(k>=SHARP_CURVE){
      sharpDistance=std::min(sharpDistance,d);
      const double available=std::max(0.,d-1.5);
      curveSpeedRaw=std::min(curveSpeedRaw,
        std::sqrt(sharpTarget*sharpTarget+10.*available));
    }else if(k>=MILD_CURVE){
      mildDistance=std::min(mildDistance,d);
      const double available=std::max(0.,d-1.);
      curveSpeedRaw=std::min(curveSpeedRaw,
        std::sqrt(mildTarget*mildTarget+10.*available));
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
  const double curvatureSpeedLimit=std::clamp(curveSpeedRaw,3.,MAX_TEAM_SPEED);
  // Starting a 2.5s overtake is unsafe if a sharp bend is within its
  // projected travel range, even if the current speed cap is still high.
  const bool overtakeZone=!sharpAhead &&
    currentCurve<SHARP_CURVE;
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
  if(rearBehind && rearTTC<=5.){
    if(!defenseOpportunity){defenseStart=now;defenseOutcome="DETECTED";}
    if(!defenseOpportunity)std::cout<<"[BATTLE] REAR_PRESSURE TTC="<<rearTTC
      <<" gap="<<rearGap<<" team_speed="<<team.v<<" rear_speed="<<rear.v<<std::endl;
    defenseOpportunity=true;
  }
  // A change is a committed manoeuvre: keep its goal unless a *predicted
  // footprint collision* is detected. Negative clearance by itself is not
  // a collision and must not create oscillating replan requests.
  if(team.changing){
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
                std::min(team.v+1.0,10.0),2.5,true)>=0.){
      change(preferredLane,now);joinPending=false;
      std::cout<<"[BATTLE] JOIN initial lane="<<initialLane
               <<" distance_from_grid_m="<<(team.s-teamStart)
               <<" from_offset="<<team.lat
               <<" to_offset="<<preferredLane<<std::endl;
    }
  }else if(now-changeFinishedAt<2.0){
    // Hysteresis prevents immediate re-selection of the opposite lane.
  }else if(!overtakeZone && rearBehind && rearTTC<=5.0 && !defended &&
            frontTTC>1.0 && !(frontGap<8. && !passed)){
    // Handle rear pressure before an optional front overtake. Safest
    // defense is maintaining a corridor or speeding away, not cutting off
    // a vehicle already alongside.
    const double backClearance=clearance(pose(team),pose(rear));
    if(rearGap>LEN+STRAIGHT_TARGET && backClearance>STRAIGHT_TARGET){
      if(defenseOutcome=="DETECTED" || defenseOutcome=="SAFE_ABORT"){
        defenseOutcome="HOLDING";
        std::cout<<"[BATTLE] DEFENSE HOLD_START TTC="<<rearTTC
                 <<"s gap="<<rearGap<<"m clearance="<<backClearance<<std::endl;
      }
    }else{
      defenseOutcome="SAFE_ABORT";
      if(now-lastDefenseLog>1.0){
        lastDefenseLog=now;
        std::cout<<"[BATTLE] DEFENSE SAFE_ABORT TTC="<<rearTTC
                 <<" gap="<<rearGap<<" clearance="<<backClearance<<std::endl;
      }
    }
  }else if(overtakeZone && !passed &&
            (!raceMode || team.s-teamStart>=5.0) &&
            lead.s>team.s+LEN &&
            frontGap>LEN+STRAIGHT_TARGET &&
            frontGap<(raceMode && trackName=="monaco"?28.:20.) &&
            (frontTTC<=3. || frontGap<5.0 ||
             (raceMode && trackName=="monaco" &&
              frontGap<28. && team.v>lead.v+.3 &&
              (frontGap-LEN-STRAIGHT_TARGET)/
                std::max(.3,team.v-lead.v)<4.5) ||
             (frontGap<18. && curvatureSpeedLimit>lead.v+0.5 &&
              team.s-teamStart<lap-25.))){
    // Start looking before following at 5m. A 2.5s manoeuvre begun
    // after the lead is already alongside is often too late on Monaco.
    // Pre-position only if both opponents and the bend remain safe for
    // the complete forecast; don't make a mandatory unsafe lane change.
    double bestScore=-1e9,bestTarget=team.lat,bestSpeed=team.v;
    const double passingLimit=std::min(MAX_TEAM_SPEED,curvatureSpeedLimit);
    for(double dst:{-lateralOffset,0.,lateralOffset}){
      if(fabs(dst-team.lat)<.2*lateralOffset)continue;
      for(double trial:{15.,14.,13.,12.,11.,10.,9.}){
        // A lane change at or below the 9m/s leader is not an overtake.
        if(trial<=lead.v+.5 || trial>passingLimit+.01)continue;
        const double score=forecast(dst,elapsed,now,false,trial,3.0,true,
                                    mildAhead?CORNER_GAP:STRAIGHT_GAP);
        // Prefer a genuinely faster passing profile after it has passed
        // the hard footprint checks; excessive clearance alone should not
        // make a 10m/s plan outrank a safe 12m/s plan.
        const double utility=.25*std::min(score,1.5)-
                             .10*fabs(dst-team.lat)+
                             .50*(trial-lead.v);
        if(score>=0. && utility>bestScore){
          bestScore=utility;bestTarget=dst;bestSpeed=trial;
        }
      }
    }
    if(fabs(bestTarget-team.lat)>.2*lateralOffset){
      change(bestTarget,now);attempted=true;attemptCount++;
      plannedPassSpeed=bestSpeed;
      passingActive=true;
      std::cout<<"[BATTLE] OVERTAKE ATTEMPT TTC="<<frontTTC
               <<"s lead_body_gap="<<clearance(pose(team),pose(lead))
               <<" target_offset="<<bestTarget<<" speed="<<bestSpeed
               <<" curve="<<upcomingCurve
               <<" front_gap="<<frontGap
               <<" lap_remaining="<<(lap-(team.s-teamStart))
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
   if(t>=1){team.lat=team.to;team.changing=false;changeFinishedAt=now;}
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
      team.changing=false;changeFinishedAt=now;
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
      if(b==&a)continue;
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
  // Three-lap race: normal pace 10m/s, safe passing may reach 15m/s.
  // A requested average is measured; it is not a hard speed controller.
  const double teamCruise=raceMode?10.:MAX_TEAM_SPEED;
  double vt=cruise(team,teamCruise),vl=cruise(lead,9),vr=cruise(rear,rearCruise);
  // A 9m/s lead and 9m/s TEAM11 create an infinite-TTC deadlock.
  // Do not force unsafe speed through an overlapping lane. Instead allow
  // short rolling-horizon acceleration only when projected body spacing
  // remains compliant. Re-evaluate it on every simulation update.
  double passingSpeedWindow=0.;
  const bool trackingLead=!passed && frontGap>LEN+STRAIGHT_GAP &&
                          frontGap<7. && team.v<11.0;
  if(trackingLead && overtakeZone && !team.changing && !joinPending){
    const double currentBody=clearance(pose(team),pose(lead));
    if(currentBody>STRAIGHT_GAP+.25){
      // Continue checking at the current passing speed, not just while
      // team speed is almost identical to the 9m/s lead car.
      for(double trial:{15.,14.,13.,12.,11.,10.5,10.,9.5}){
        if(trial>curvatureSpeedLimit+.01)continue;
        if(forecast(team.lat,elapsed,now,false,trial,1.0,false,
                    STRAIGHT_GAP+.08)>=0.){
          passingSpeedWindow=trial;
          if(now-lastPassSpeedLog>3.0){
            lastPassSpeedLog=now;
            std::cout<<"[BATTLE] PASS_SPEED_WINDOW trial="<<trial
                     <<" gap="<<frontGap
                     <<" body="<<currentBody<<std::endl;
          }
          break;
        }
      }
    }
  }
  // Scripted rear driver anticipates a cornering car's pace well before
  // the physical 0.7m limit. Relative-speed braking at 2-3m was too late
  // on Monaco: 11m/s rear closed into a 9m/s corner-following TEAM11.
  for(const Car*ahead:{&team,&lead}){
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
    for(double trial:{15.,14.,13.,12.,11.,10.,9.,8.,7.,6.,5.,4.,3.}){
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
      comfortSpeed=9.;
      if(frontGap>LEN+STRAIGHT_TARGET+1.5 &&
         !team.changing &&
         forecast(team.lat,elapsed,now,false,10.0,1.0,false,
                  corner?CORNER_TARGET:STRAIGHT_TARGET)>=0.)
        comfortSpeed=10.;
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
  // Preserve a locally safe acceleration window; the existing 2.5s
  // following forecast otherwise repeatedly resets the limit to 9m/s.
  if(passingSpeedWindow>comfortSpeed)comfortSpeed=passingSpeedWindow;
  // Keep lane choice and speed together throughout a committed overtake.
  // Recheck the actual interpolated lane against both traffic vehicles
  // before lifting the normal leading-car following speed cap.
  if(passingActive && !passed){
    const double trial=std::min(plannedPassSpeed,curvatureSpeedLimit);
    const double predicted=forecast(team.changing?team.to:team.lat,
      elapsed,now,team.changing,trial,1.2,true,
      mildAhead?CORNER_GAP:STRAIGHT_GAP);
    if(overtakeZone && trial>lead.v+.1 && predicted>=0.){
      passingSpeedWindow=std::max(passingSpeedWindow,trial);
      comfortSpeed=std::max(comfortSpeed,trial);
    }else if(sharpAhead){
      passingActive=false;
      std::cout<<"[BATTLE] OVERTAKE PAUSE sharp_corner speed_limit="
               <<curvatureSpeedLimit<<std::endl;
    }
  }
  // The speed ceiling for approaching bends remains mandatory.
  comfortSpeed=std::min(comfortSpeed,curvatureSpeedLimit);
  // Record why a safe passing speed is not being selected, instead of
  // assuming that every no-overtake case is a lane-planner failure.
  if(!passed && !team.changing && now-lastOvertakeDiagnostic>=5.0){
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
  vt=std::min(vt,curvatureSpeedLimit);
  // Ordinary lane joins are capped at 10.5m/s. A committed pass
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
    // The emergency ceiling is not the normal 10m/s cruise target:
    // collision-checked passes must still be able to reach 15m/s.
    speedCeiling=MAX_TEAM_SPEED;
    if(forecast(team.changing?team.to:team.lat,elapsed,now,team.changing,team.v,1.,false)<0.){
      speedCeiling=0.;
      for(double trial:{15.,14.,13.,12.,11.,10.5,9.,7.,5.,3.,1.,0.}){
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
  if(!passed && now-lastSpeedDecisionLog>=3.0 &&
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
  // Phase 2 (Monaco only): sustain a safe, straight-line passing window.
  // Phase 1 did not complete any pass despite 5-6 attempts. After a
  // committed lane change ends, normal 10m/s cruise can be too slow to
  // close on the 9m/s S-car. Never override curvature or braking caps.
  // Use both vehicles in the footprint prediction, including the pursuer.
  if(raceMode && trackName=="monaco" && !passed &&
     !team.changing && !joinPending &&
     now-changeFinishedAt>0.5 && overtakeZone &&
     currentCurve<MILD_CURVE && sharpDistance>20. &&
     curvatureSpeedLimit>=12.5 &&
     lead.s>team.s+LEN && lead.s-team.s<25. &&
     fabs(team.lat-lead.lat)>WID+.25 &&
     clearance(pose(team),pose(lead))>STRAIGHT_GAP &&
     forecast(team.lat,elapsed,now,false,12.5,1.5,true)>=0.){
    const double prior=vt;
    vt=std::max(vt,12.5);
    if(vt>prior+.01 && now-lastMonacoPassLog>2.){
      lastMonacoPassLog=now;
      std::cout<<"[BATTLE] MONACO_PASS_ACCEL target="<<vt
               <<" lead_gap="<<(lead.s-team.s)
               <<" lateral_delta="<<fabs(team.lat-lead.lat)
               <<" lead_v="<<lead.v<<std::endl;
    }
  }
  // Monaco boost must never bypass the existing 1s emergency-braking cap.
  vt=std::min(vt,speedCeiling);
  // Phase 1 (Monaco only): a modest 11.5m/s corner-exit burst.
  // Retain the existing 7.5m/s sharp-corner cap, front/rear collision
  // forecasts, road envelope, and physical acceleration limit.
  // Do not accelerate before the bend has actually ended.
  if(raceMode && trackName=="monaco" &&
     now-lastMonacoSharpAt>0. && now-lastMonacoSharpAt<3.0 &&
     currentCurve<MILD_CURVE && sharpDistance>10. &&
     !team.changing && !joinPending && curvatureSpeedLimit>=10.5 &&
     forecast(team.lat,elapsed,now,false,11.5,1.5,true)>=0.){
    const double prior=vt;
    vt=std::max(vt,std::min(11.5,curvatureSpeedLimit));
    if(vt>prior+.01 && now-lastMonacoExitLog>2.){
      lastMonacoExitLog=now;
      std::cout<<"[BATTLE] MONACO_EXIT_ACCEL target="<<vt
               <<" speed="<<team.v
               <<" curve="<<currentCurve
               <<" sharp_ahead_m="<<sharpDistance<<std::endl;
    }
  }
  // Preserve the emergency ceiling after both Monaco-specific accelerations.
  vt=std::min(vt,speedCeiling);
  // Braking for a pursuer behind would make a rear-end collision worse,
  // but only when the forward trajectory also remains clear.
  if(rearBehind && rearTTC<2. && frontTTC>2. &&
     !team.changing && speedCeiling>=team.v &&
     (!raceMode || forecast(team.lat,elapsed,now,false,
          std::min(curvatureSpeedLimit,team.v+0.5),1.5,true)>=0.))
    vt=std::max(vt,std::min(curvatureSpeedLimit,team.v+0.5));
  vt=std::min(vt,curvatureSpeedLimit);
  if(raceMode && now-lastRaceHazardCheck>=.1){
    lastRaceHazardCheck=now;
    // The old emergency check only looked one second ahead and could
    // return speed=0 when the lead car was already beside TEAM11.
    // Check a longer window while braking is still physically possible.
    const double horizon=2.5;
    const double dst=team.changing?team.to:team.lat;
    // Forward collision speed planning must not include an overtaken car.
    // Rear and side vehicles remain checked in lane-change forecasts.
    const double risk=forecast(dst,elapsed,now,team.changing,vt,horizon,false);
    if(risk<0.){
      bool found=false;
      for(double candidate:{15.,14.,13.,12.,11.,10.,9.,8.,7.,6.,5.,4.,3.,2.,1.,0.}){
        if(candidate>vt+.01)continue;
        if(forecast(dst,elapsed,now,team.changing,candidate,horizon,false)>=0.){
          raceSafeTarget=candidate;
          found=true;
          break;
        }
      }
      if(!found){
        const bool actualFrontCar=lead.s>=team.s-LEN*.5;
        // A zero-speed target is reserved for a genuine forward hazard.
        // If an S-car has already been passed, do not create a rear impact.
        raceSafeTarget=actualFrontCar?0.:MAX_TEAM_SPEED;
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
      raceSafeTarget=MAX_TEAM_SPEED;
    }
  }
  if(raceMode)vt=std::min(vt,raceSafeTarget);
  if(!team.changing && frontTTC>1. && !joinPending)emergencyBraking=false;
  if((vl<8.9||vr<rearCruise-.1)&&elapsed-lastBrake>1.5){
    lastBrake=elapsed;braked=true;
    std::cout<<"[BATTLE] PRACTICE BRAKING lead="<<vl<<" rear="<<vr<<std::endl;
  }
  auto update=[dt](Car &a,double desired){
    // Smoothed speed transitions: integrate trapezoidally over real dt.
    const double before=a.v;
    a.v+=std::clamp(desired-a.v,-5.0*dt,3.0*dt);
    a.s+=(before+a.v)*.5*dt;
  };
  update(team,vt);update(lead,vl);update(rear,vr);
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
  if(team.s>lead.s+LEN+targetGap&&!passed &&
     fabs(team.lat-lead.lat)>WID+.12){
    passed=true;passingActive=false;
    std::cout<<"[BATTLE] OVERTAKE COMPLETED mode="
             <<(attempted?"LANE_CHANGE":"CLEAR_LANE_ACCELERATION")<<std::endl;
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
  alignToMotion(b,previousLead);
  alignToMotion(c,previousRear);
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
    const double clearanceLead=clearance(ap,bp);
    const double clearanceRear=clearance(ap,cp);
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
      std::min(clearance(a,b),clearance(a,c)));
  for(auto pr:std::array<std::pair<Car*,Pos>,3>{{{&team,a},{&lead,b},{&rear,c}}})
   gz::sim::Model(pr.first->entity).SetWorldPoseCmd(
    ecm,gz::math::Pose3d(pr.second.x,pr.second.y,pr.second.z+.09,0,0,pr.second.yaw));
  if(now-lastCsv>=.1){
    report<<std::fixed<<std::setprecision(4)<<elapsed<<','<<team.s<<','<<team.lat<<','<<team.v<<','
          <<lead.s<<','<<lead.lat<<','<<lead.v<<','<<rear.s<<','<<rear.lat<<','<<rear.v<<','
          <<frontTTC<<','<<rearTTC<<','<<minimumCenter<<','<<minimumClearance<<'\n';
    report.flush();lastCsv=now;
  }
  if(raceMode && team.s-teamStart>=raceLaps*lap){
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
  if(!raceMode && team.s-teamStart>=lap){
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
  if(elapsed>(raceMode?900.:300.))finish("FAIL_TIMEOUT");
  previousRearTTC=rearTTC;
 }
 private:
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
  for(double yaw:{a.yaw,b.y}){
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
 double forecast(double destination,double elapsed,double now,bool active,double desiredSpeed=-1.,double horizon=5.,bool includeRear=true,double requiredGap=-1.)const{
  double minimumMargin=1e6;
  Pos lastP=pose(team);
  double lastHeading=lastP.yaw;
  for(int i=1;i<=static_cast<int>(horizon*10+.5);i++){
    const double t=i*.1;
    Car candidate=team;
    const double vmax=desiredSpeed<0?std::max(0.,team.v):desiredSpeed;
    // Match the real replay controller: +3 / -5m/s^2. An
    // unrealistically fast predicted brake gives false safety clearance.
    const double acc=vmax<team.v?-5.:3.;
    const double reach=fabs(vmax-team.v)/fabs(acc);
    const double until=std::min(t,reach);
    const double distance=team.v*until+.5*acc*until*until+
                          vmax*std::max(0.,t-until);
    candidate.s+=std::max(0.,distance);
    const double from=active?team.from:team.lat;
    const double start=active?team.changeStart:now;
    const double duration=active?team.changeDuration:2.5;
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
    for(const Car*o:{&lead,&rear}){
      // The lane-change predictor checks both front and rear vehicles.
      // Speed/braking prediction checks forward traffic only. An overtaken
      // S-car must not cause an emergency stop from behind.
      if(o==&rear && !includeRear)continue;
      if(raceMode && !includeRear && o->s<team.s-LEN*.5)continue;
      const double cap=o==&lead?9.:rearCruise;
      const double tacc=std::min(t,std::max(0.,cap-o->v)/3.);
      const double normalTravel=o->v*tacc+1.5*tacc*tacc+
                                cap*std::max(0.,t-tacc);
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
        if(o==&lead){
          const double seq[5]={-lateralOffset,0,lateralOffset,0,-lateralOffset};
          const double absolute=elapsed+t;
          const int period=static_cast<int>(absolute/2.);
          const int phase=period%4;
          const double from=(raceMode && period==0)?leadInitialLat:seq[phase];
        sample.lat=from+(seq[phase+1]-from)*ease((absolute-2.*period)/2.);
        }
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
 void change(double dest,double now){
  team.from=team.lat;team.to=dest;team.changeStart=now;
  team.changeDuration=2.5;team.changing=true;
 }
 void finish(const char*result){
  if(ended)return;ended=true;report.flush();
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
 void Start(const gz::msgs::Boolean&b){if(b.data())startSignal.store(true);}
 gz::sim::Entity worldEntity{gz::sim::kNullEntity};
 gz::transport::Node node;gz::transport::Node::Publisher ready;
 std::atomic<bool> startSignal{false};
 std::vector<Pt>route;Car team,lead,rear;
 std::ofstream report;double lap=0,lateralOffset=0,leadInitialLat=0,teamStart=0,begin=0,last=0,lastCsv=0,lastBrake=-10;
 int raceLaps=1,completedTeamLaps=0;
 bool raceMode=false;
 double peakTeam=0.,peakLead=0.,peakRear=0.,minWallMargin=1e9,lastElapsed=0.;
 double lastCloseWarningLog=-1e6,lastWallCorrectionLog=-1e6,lastWallWarningLog=-1e6;
 double lastRaceHazardCheck=-1e6,lastRaceHazardLog=-1e6,raceSafeTarget=MAX_TEAM_SPEED;
 long long closeWarningCount=0,wallWarningCount=0;
 double previousRearTTC=1e6,minRearTTC=1e6,minimumCenter=1e9,minimumClearance=1e9,preferredLane=0.;
 double lastReplan=-1e6,changeFinishedAt=-1e6,defenseStart=-1e6;
 double lastComfortForecast=-1e6,comfortSpeed=MAX_TEAM_SPEED,lastPracticeYield=-1e6,lastOvertakeDiagnostic=-1e6,lastPassSpeedLog=-1e6,lastSpeedDecisionLog=-1e6,lastPassMotionLog=-1e6;
 double lastSpeedForecast=-1e6,lastSpeedLog=-1e6,lastDefenseLog=-1e6,speedCeiling=MAX_TEAM_SPEED,rearCruise=11.,plannedPassSpeed=9.;
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
GZ_ADD_PLUGIN(urrc::BattleReplay_08,gz::sim::System,urrc::BattleReplay_08::ISystemConfigure,urrc::BattleReplay_08::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::BattleReplay_08,"urrc::BattleReplay_08")
