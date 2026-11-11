// TEAM11 3-lane kinematic practice controller; Gazebo Harmonic gz-sim8.
// Vehicle moves by pose command, not physical steering joints.
// It observes model poses in Gazebo and logs equivalent bicycle steering.
#include "OvertakePlanner.hh"
#include "RearDefense.hh"
#include "RearScan.hh"
#include "PhysicsDrive.hh"
#include "CornerPlanner.hh"
#include "SpeedProfile.hh"
#include "BoundedTracker.hh"
#include "LapVerifier.hh"
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace urrc {
struct Sample {double s,x,y,z,yaw;};
static double clamp(double x,double a,double b){return std::max(a,std::min(b,x));}
static double smooth(double x){x=clamp(x,0,1);return x*x*x*(10+x*(-15+6*x));}
static double unwrap(double a,double b){return a+std::atan2(std::sin(b-a),std::cos(b-a));}
struct Position {double x,y,z;};
enum class ShiftReason {Front,Defense,Corner};
class Team11ThreeLane:public gz::sim::System,
 public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &world,
    const std::shared_ptr<const sdf::Element> &cfg,
    gz::sim::EntityComponentManager &,gz::sim::EventManager &) override {
    this->world=world;
    egoName=cfg->Get<std::string>("ego_name");
    lane=cfg->Get<int>("initial_lane");
    target=lane;
    homeLane=lane;
    fixedLane=cfg->Get<bool>("fixed_lane",false).first;
    physicsContact=cfg->Get<bool>("physics_contact_mode",false).first;
    spacing=cfg->Get<double>("spacing_m");
    wheelbase=cfg->Get<double>("wheelbase_m");
    maxSteer=cfg->Get<double>("maximum_steering_deg",30.).first*M_PI/180.;
    maxYawRate=cfg->Get<double>("maximum_yaw_rate_deg_s",68.754935).first*M_PI/180.;
    if(wheelbase<=0. || maxSteer<=0. || maxSteer>M_PI/6.+1.e-8 ||
       maxYawRate<=0. || maxYawRate>bounded::kMaxYawRate+1.e-5){
      std::cerr<<"[TEAM11 ERROR] Invalid steering / yaw rate safety cap\n";
      return;
    }
    baseSpeed=cfg->Get<double>("base_speed_mps");
    slowThreshold=cfg->Get<double>("steering_slow_threshold_deg")*M_PI/180.;
    slowFactor=cfg->Get<double>("slow_factor");
    changeLength=cfg->Get<double>("lane_change_length_m");
    scanAhead=cfg->Get<double>("scan_ahead_m");
    maxDecel=cfg->Get<double>("max_deceleration_mps2",4.0).first;
    maxAccel=cfg->Get<double>("max_acceleration_mps2",2.0).first;
    if(maxDecel<=0 || maxAccel<=0){std::cerr<<"[TEAM11 ERROR] Invalid acceleration limits\n";return;}
    startMode=cfg->Get<std::string>("start_mode","camera").first;
    cameraTopic=cfg->Get<std::string>("camera_topic","").first;
    cameraReadyFile=cfg->Get<std::string>("camera_ready_file","").first;
    rearTopic=cfg->Get<std::string>("rear_lidar_topic","").first;
    if(!rearTopic.empty() &&
       !node.Subscribe(rearTopic,&Team11ThreeLane::OnRearScan,this)){
      std::cerr<<"[TEAM11 REAR ERROR] Failed rear LiDAR subscription "
               <<rearTopic<<"\n";
      return;
    }
    lampSpacingPx=cfg->Get<double>("lamp_spacing_px",0.).first;
    if(startMode!="immediate" && startMode!="camera"){
      std::cerr<<"[TEAM11 ERROR] Invalid start mode "<<startMode<<"\n";return;
    }
    if(startMode=="camera" && !node.Subscribe(cameraTopic,&Team11ThreeLane::OnImage,this)){
      std::cerr<<"[TEAM11 ERROR] Failed subscribing camera "<<cameraTopic<<"\n";return;
    }
    const auto directory=cfg->Get<std::string>("route_prefix");
    for(int k=0;k<3;k++){
      std::ifstream file(directory+"/lane_"+std::to_string(k)+".csv");
      std::string line; std::getline(file,line);
      while(std::getline(file,line)){
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream in(line);
        Sample p{};
        if(in>>p.s>>p.x>>p.y>>p.z>>p.yaw)routes[k].push_back(p);
      }
      if(routes[k].size()<50){std::cerr<<"[TEAM11] bad lane "<<k<<"\n";return;}
    }
    // Centerline s coordinate is common to all three lanes.
    auto &r=routes[1];
    lap=r.back().s+std::hypot(r.back().x-r.front().x,r.back().y-r.front().y);
    std::vector<lapcheck::Point> actualRoute;
    actualRoute.reserve(r.size());
    for(const auto &p:r) actualRoute.push_back({p.s,p.x,p.y,p.z});
    lapVerifier.Configure(std::move(actualRoute),lap,spacing+.065);
    ready=true;
    std::cout<<"[TEAM11] 3 routes armed, spacing "<<spacing
             <<"m; target_lane="<<(target+1)
             <<" fixed_lane="<<fixedLane
             <<" start_mode="<<startMode<<" camera="<<cameraTopic<<"\n";
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if(info.paused || !ready)return;
    findEntities(ecm);
    if(ego==gz::sim::kNullEntity)return;
    if(startMode=="camera" && !cameraStarted.load()){
      const double dtWait=std::chrono::duration<double>(info.dt).count();
      if(dtWait>0 && dtWait<1.){
        cameraTrace+=dtWait;
        if(cameraTrace>=2.){
          cameraTrace=0.;
          std::cout<<"[TEAM11 CAMERA] images="<<imageFrames.load()
                   <<" red_lamps="<<lastLampCount.load()
                   <<" all_five_seen="<<fiveObserved.load()
                   <<" consecutive_off="<<offFrames.load()<<"\\n";
        }
      }
      if(fiveObserved.load() && !redReported){
        redReported=true;
        std::cout<<"[TEAM11] CAMERA all FIVE red lights confirmed; waiting zero red lamps\n";
      }
      return;
    }
    if(!started){
      const auto pose=gz::sim::worldPose(ego,ecm);
      const double x=pose.Pos().X(),y=pose.Pos().Y();
      double best=1e99;
      const auto &r=routes[1];
      for(const auto &p:r){
        double d=std::hypot(x-p.x,y-p.y);
        if(d<best){best=d;s=p.s;}
      }
      // Preserve the exact grid-01 spawn, then merge gently onto the selected route.
      const auto initial=sample(1,s);
      const std::array<double,3> offsets={spacing,0.,-spacing};
      const double offset=offsets[target];
      initialDx=x-(initial.x-offset*std::sin(initial.yaw));
      initialDy=y-(initial.y+offset*std::cos(initial.yaw));
      initialS=s;
      traveledFromStart=0.;
      started=true;
      std::cout<<"[TEAM11] "<<(startMode=="camera"?"CAMERA GREEN":"IMMEDIATE")
               <<" start s="<<s<<" lane="<<(lane+1)<<" home="<<(homeLane+1)<<" changing="<<changing
               <<" fixed="<<fixedLane
               <<" images="<<imageFrames.load()<<" lamps="<<lastLampCount.load()<<"\n";
    }
    double dt=std::chrono::duration<double>(info.dt).count();
    if(dt<=0 || dt>0.2)return;

    // The baseline camera / static route control is unchanged.
    // Overtaking is based on Gazebo model poses (not LiDAR / vision).
    const std::array<double,3> offsets{spacing,0.,-spacing};
    std::vector<avoid::Car> traffic;
    const auto &middle=routes[1];
    const double now=std::chrono::duration<double>(info.simTime).count();
    for(size_t j=0;j<practice.size();j++){
      const auto other=practice[j];
      if(other==gz::sim::kNullEntity || other==ego)continue;
      const auto pose=gz::sim::worldPose(other,ecm);
      const double ox=pose.Pos().X(),oy=pose.Pos().Y();
      double best=1e99,obsS=0.;
      size_t closest=0;
      // Local search tracks the moving cars without an O(N) full-lap scan
      // on each Gazebo physics tick.
      if(!observed[j]){
        for(size_t i=0;i<middle.size();i++){
          const double dx=ox-middle[i].x,dy=oy-middle[i].y;
          const double d=dx*dx+dy*dy;
          if(d<best){best=d;obsS=middle[i].s;closest=i;}
        }
        observed[j]=true;
        measuredSpeed[j]=j==0?10.:20.;
        lastObsTime[j]=now;
      }else{
        const int n=static_cast<int>(middle.size());
        for(int k=-40;k<=40;k++){
          const size_t i=(lastNearest[j]+n+k)%n;
          const double dx=ox-middle[i].x,dy=oy-middle[i].y;
          const double d=dx*dx+dy*dy;
          if(d<best){best=d;obsS=middle[i].s;closest=i;}
        }
        const double sampleDt=now-lastObsTime[j];
        if(sampleDt>=.10 && sampleDt<2.){
          const double delta=avoid::signedGap(lastObsS[j],obsS,lap);
          const double observedVelocity=delta/sampleDt;
          if(observedVelocity>=-.5 && observedVelocity<=30.){
            const double speedEstimate=std::max(0.,observedVelocity);
            measuredSpeed[j]=.45*speedEstimate+.55*measuredSpeed[j];
          }
          lastObsTime[j]=now;
          lastObsS[j]=obsS;
        }
      }
      if(lastObsTime[j]==now)lastObsS[j]=obsS;
      lastNearest[j]=closest;
      const auto &p=middle[closest];
      const double lateral=-(ox-p.x)*std::sin(p.yaw)
                            +(oy-p.y)*std::cos(p.yaw);
      traffic.push_back({static_cast<int>(j),obsS,lateral,measuredSpeed[j]});
    }
    // Only confirmed, fresh rear LiDAR returns become defensive targets.
    // The demo retains the pre-existing Gazebo pose speed / lane tracker
    // solely for associating a rear scan with known pace cars.
    rear_scan::Scan rearImage;
    bool freshRear=false;
    {
      std::lock_guard<std::mutex> guard(rearMutex);
      const auto age=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-rearArrival).count();
      freshRear=(rearFrames.load()>0 && age<.85);
      if(freshRear)rearImage=lastRearImage;
    }
    std::vector<defense::RearTarget> rearTargets;
    if(freshRear){
      const auto egoPose=gz::sim::worldPose(ego,ecm);
      const double yaw=egoPose.Rot().Yaw();
      for(const auto &car:traffic){
        const auto other=practice[static_cast<size_t>(car.id)];
        if(other==gz::sim::kNullEntity)continue;
        const auto otherPose=gz::sim::worldPose(other,ecm);
        const double dx=otherPose.Pos().X()-egoPose.Pos().X();
        const double dy=otherPose.Pos().Y()-egoPose.Pos().Y();
        const double localX=dx*std::cos(yaw)+dy*std::sin(yaw);
        const double localY=-dx*std::sin(yaw)+dy*std::cos(yaw);
        const double behind=-avoid::signedGap(s,car.s,lap);
        if(localX>=-.4 || behind<=0. || behind>defense::kRearDetectM)
          continue;
        const bool seen=rear_scan::verifiedReturn(
          rearImage,localX,localY);
        if(!seen){
          if(now-lastRearSample[static_cast<size_t>(car.id)]>.7)
            rearEvidence[static_cast<size_t>(car.id)]=0;
          continue;
        }
        // Same rear target is tracked across multiple simulation frames.
        const size_t idx=static_cast<size_t>(car.id);
        // A single sideways deviation is NOT proof of S-shaped weaving.
        // Require repeated lateral direction reversals at meaningful speed.
        const double age=now-lastLateralUpdate[idx];
        if(lastLateralUpdate[idx]<0. || age>=.18){
          if(lastLateralUpdate[idx]>=0. && age<1.){
            const double rate=(car.lateral-lastLateral[idx])/
                              std::max(.01,age);
            if(std::abs(rate)>.55 &&
               std::abs(lastLateralRate[idx])>.55 &&
               rate*lastLateralRate[idx]<0.)
              weaveEvidence[idx]=std::min(6,weaveEvidence[idx]+1);
            else if(std::abs(rate)<.25)
              weaveEvidence[idx]=std::max(0,weaveEvidence[idx]-1);
            lastLateralRate[idx]=rate;
          }else{
            weaveEvidence[idx]=0;
            lastLateralRate[idx]=0.;
          }
          lastLateralUpdate[idx]=now;
          lastLateral[idx]=car.lateral;
        }
        const bool weaving=weaveEvidence[idx]>=2;
        // Do not count many Gazebo physics steps from the same scan as
        // independent evidence. Require >=3 spaced observations.
        const double closing=car.speed-speed;
        const double ttc=closing>0.1?behind/closing:1e9;
        // Only accept independent LiDAR frames, not repeated physics
        // updates that inspect one cached scan many times.
        const unsigned int frameId=rearFrames.load();
        if(now-lastRearSample[idx]>=.15 &&
           frameId!=lastRearFrameId[idx]){
          if(now-lastRearSample[idx]>1.0)rearEvidence[idx]=0;
          lastRearSample[idx]=now;
          lastRearFrameId[idx]=frameId;
          const bool urgent=behind<=defense::kRearDetectM &&
             closing>=defense::kClosingMinMps &&
             ttc<=defense::kThreatTtcS;
          rearDeclining[idx]=urgent && rearEvidence[idx]>=1 &&
              ttc<=lastRearTtc[idx]-.10;
          rearEvidence[idx]=urgent?
              std::min(12,rearEvidence[idx]+1):0;
          lastRearTtc[idx]=ttc;
        }
        // Three urgent observations OR two with demonstrably falling TTC.
        const bool confirmed=(
          rearEvidence[idx]>=3 ||
          (rearEvidence[idx]>=2 && rearDeclining[idx])) &&
          now-lastRearSample[idx]<.5;
        rearTargets.push_back({car.id,behind,car.speed,
                               car.lateral,seen,weaving,
                               confirmed,rearDeclining[idx]});
      }
    }
    // Classify the road before deciding whether a safe passing gap exists.
    // This is independent of temporary target lane selection.
    // Inspect original road bends ahead of steering decisions; lateral
    // lane-shift geometry is not a new road corner.
    corner::Preview preview;
    for(double look=0.;look<=corner::kPreviewM;look+=2.){
      const auto a=sample(1,s+look);
      const auto b=sample(1,s+look+2.);
      const auto c=sample(1,s+look+4.);
      const double ya=std::atan2(b.y-a.y,b.x-a.x);
      const double yb=std::atan2(c.y-b.y,c.x-b.x);
      const double delta=std::atan2(std::sin(yb-ya),std::cos(yb-ya));
      const double k=std::abs(delta)/
                      std::max(.4,std::hypot(c.x-b.x,c.y-b.y));
      preview.maxCurvature=std::max(preview.maxCurvature,k);
      if(k>=corner::kEnterCurvature &&
         !std::isfinite(preview.firstBendDistance))
        preview.firstBendDistance=look;
    }
    const double roadCurve=preview.maxCurvature;
    if(!cornerMode && roadCurve>=corner::kEnterCurvature)cornerMode=true;
    else if(cornerMode && roadCurve<=corner::kExitCurvature)cornerMode=false;
    preview.bendAhead=cornerMode;

    if(changing && avoid::signedGap(changeStart,s,lap)>=changeLength-1e-5){
      changing=false;lane=target;
      const char *prefix=changeReason==ShiftReason::Corner ?
        "[TEAM11 CORNER]" : changeReason==ShiftReason::Defense ?
        "[TEAM11 DEFENSE]" : "[TEAM11 AVOID]";
      std::cout<<prefix<<" finished lane="<<(lane+1)<<std::endl;
    }
    double followingSpeed=std::numeric_limits<double>::infinity();
    double cornerApproachCap=std::numeric_limits<double>::infinity();
    if(!fixedLane){
      avoid::State state;
      state.s=s;state.lap=lap;state.speed=speed;
      state.roadSpeed=avoid::curvatureSpeed(roadCurve);
      state.spacing=spacing;state.curve=roadCurve;
      state.travelled=traveledFromStart;
      state.sinceChange=traveledFromStart-lastChangeTravel;
      state.lane=changing?lane:target;
      state.targetLane=target;
      state.homeLane=homeLane;state.avoidId=avoidIndex;
      state.changing=changing;
      const auto decision=avoid::evaluate(state,traffic);
      followingSpeed=decision.followSpeed;
      // Front passing / obstacle avoidance ALWAYS takes priority over rear.
      defense::State defenseState;
      defenseState.car=state;
      defenseState.frontOvertakeActive=(avoidIndex>=0);
      defenseState.frontOvertakeRequested=(
        decision.nextLane>=0 || decision.blocked);
      defenseState.sinceDefenseChangeM=traveledFromStart-lastDefenseTravel;
      defenseState.zigzagEndpoint=zigzagEndpoint;
      const auto rearDecision=defense::evaluate(
        defenseState,rearTargets,traffic);
      // Front overtaking priority > corner merge > rear defense.
      const auto cornerDecision=corner::evaluate(
        state,preview,traffic,decision.nextLane>=0 || decision.blocked,
        avoidIndex>=0,fixedLane,maxDecel);
      cornerApproachCap=cornerDecision.speedCap;
      if(decision.nextLane>=0 && !changing){
        changeReason=ShiftReason::Front;
        defending=false;
        const int oldLane=target;
        lane=target;target=decision.nextLane;
        changing=true;changeStart=s;
        changeLength=decision.changeLength;
        lastChangeTravel=traveledFromStart;
        avoidIndex=decision.returning?-1:decision.avoidId;
        double leadGap=std::numeric_limits<double>::infinity();
        double leadTtc=std::numeric_limits<double>::infinity();
        if(!decision.returning){
          for(const auto &car:traffic){
            if(car.id!=decision.avoidId)continue;
            leadGap=avoid::signedGap(s,car.s,lap);
            const double closing=speed-car.speed;
            if(closing>avoid::kMinClosingMps)
              leadTtc=std::max(0.,leadGap-avoid::kVehicleOverlapM)/closing;
            break;
          }
        }
        std::cout<<"[TEAM11 AVOID] "
                 <<(decision.returning?"return ":"pass ")
                 <<(oldLane+1)<<" -> "<<(target+1)
                 <<" length_m="<<changeLength
                 <<" current_mps="<<speed
                 <<" front_gap_m="<<leadGap
                 <<" collision_ttc_s="<<leadTtc
                 <<" road_curve="<<roadCurve<<std::endl;
      }else if(!changing && cornerDecision.nextLane>=0 &&
               !decision.blocked){
        changeReason=ShiftReason::Corner;
        const int previous=target;
        lane=target;target=cornerDecision.nextLane;
        changing=true;changeStart=s;
        changeLength=cornerDecision.length;
        lastChangeTravel=traveledFromStart;
        avoidIndex=-1;
        homeLane=corner::kCenterLane;
        defending=false;
        std::cout<<"[TEAM11 CORNER] slow merge "
                 <<(previous+1)<<" -> "<<(target+1)
                 <<" length_m="<<changeLength
                 <<" speed_cap_mps="<<cornerDecision.speedCap
                 <<" road_curve="<<roadCurve
                 <<" first_bend_m="<<preview.firstBendDistance
                 <<std::endl;
      }else if(!changing && rearDecision.nextLane>=0 &&
               !decision.blocked && !cornerMode){
        changeReason=ShiftReason::Defense;
        const int oldLane=target;
        lane=target;
        target=rearDecision.nextLane;
        changeStart=s;
        changeLength=rearDecision.length;
        changing=true;
        lastChangeTravel=traveledFromStart;
        lastDefenseTravel=traveledFromStart;
        defending=true;
        if(rearDecision.zigzag && target==zigzagEndpoint)
          zigzagEndpoint=(zigzagEndpoint==0?2:0);
        std::cout<<"[TEAM11 DEFENSE] shift "<<(oldLane+1)
                 <<" -> "<<(target+1)
                 <<" rear_id="<<rearDecision.rearId
                 <<" rear_ttc_s="<<rearDecision.ttc
                 <<" zigzag="<<rearDecision.zigzag
                 <<" rear_lidar_frames="<<rearFrames.load()
                 <<std::endl;
      // A successfully defended route is the new normal route.
      // No automatic 2 -> 3 return when the rear contact disappears.
      }else if(decision.blocked && !changing &&
               trackedAt<now-.8){
        trackedAt=now;
        std::cout<<"[TEAM11 AVOID] FOLLOW lane="<<(target+1)
                 <<" follow_target_mps="<<followingSpeed<<std::endl;
      }
    }
    // Quintic interpolation over traveled distance, including lap wrap.
    double changeProgress=std::fmod(s-changeStart+lap,lap)/changeLength;
    if(changing && changeProgress>=1.0){changing=false;lane=target;}
    auto lateral=[&](double distance){
      if(!changing)return offsets[target];
      return offsets[lane]+(offsets[target]-offsets[lane])*
             smooth((std::fmod(distance-changeStart+lap,lap))/changeLength);
    };
    auto poseAt=[&](double distance){
      auto mid=sample(1,distance);
      double o=lateral(distance);
      Position p{mid.x-o*std::sin(mid.yaw),mid.y+o*std::cos(mid.yaw),mid.z};
      // Quintic grid-01 -> route merge, 12m travel, no teleport on spawn.
      if(traveledFromStart<12.){
        const double ahead=std::fmod(distance-s+lap,lap);
        const double blend=1.-smooth((traveledFromStart+ahead)/12.);
        p.x+=initialDx*blend;
        p.y+=initialDy*blend;
      }
      return p;
    };
    const auto p0=poseAt(s),p1=poseAt(s+.4),p2=poseAt(s+.8);
    const double yaw0=std::atan2(p1.y-p0.y,p1.x-p0.x);
    const double yaw1=std::atan2(p2.y-p1.y,p2.x-p1.x);
    double yawChange=std::atan2(std::sin(yaw1-yaw0),std::cos(yaw1-yaw0));
    const double curvature=yawChange/std::max(.05,std::hypot(p1.x-p0.x,p1.y-p0.y));
    const double steer=clamp(std::atan(wheelbase*curvature),
                             -maxSteer,maxSteer);
    // Preview 36m of ACTUAL XY trajectory geometry, not waypoint yaw.
    // Earlier raw yaw preview reported spurious ~1.5rad/m on straights.
    // Braking from 15 to 3m/s at 4m/s2 needs 27m.
    double futureCurvature=std::abs(curvature);
    // Avoid applying apex curvature to CURRENT speed when the apex is
    // still 20-36m away. Each point gives its own braking envelope.
    double profileSpeedLimit=speed_profile::allowedNow(
        std::abs(curvature),0.,maxDecel,wheelbase,maxSteer,maxYawRate);
    for(double look=0.;look<=36.;look+=3.){
      const auto a=poseAt(s+look);
      const auto b=poseAt(s+look+2.);
      const auto c=poseAt(s+look+4.);
      const double first=std::atan2(b.y-a.y,b.x-a.x);
      const double second=std::atan2(c.y-b.y,c.x-b.x);
      const double turn=std::atan2(std::sin(second-first),
                                   std::cos(second-first));
      const double distance=std::max(.4,std::hypot(c.x-b.x,c.y-b.y));
      const double k=std::abs(turn)/distance;
      futureCurvature=std::max(futureCurvature,k);
      profileSpeedLimit=std::min(profileSpeedLimit,
        speed_profile::allowedNow(
          k,look+2.,maxDecel,wheelbase,maxSteer,maxYawRate));
    }
    // Straight and gentle curves use the same 10-15 m/s policy.
    // Ordinary and sharp curves both use a 5-7 m/s target.
    // Hysteresis prevents repeated acceleration/braking at the threshold.
    double targetSpeed=std::min(baseSpeed,profileSpeedLimit);
    // The corner flags and 52m lookahead are for safe merging. Braking
    // is tied to remaining DISTANCE, not the worst peak in a long window.
    targetSpeed=std::min(targetSpeed,cornerApproachCap);
    // Excessive commanded steering still reduces speed even if a noisy
    // preview momentarily classifies the road as straight.
    if(std::abs(steer)>slowThreshold)targetSpeed=std::min(targetSpeed,7.);
    if(std::abs(steer)>2.*slowThreshold)targetSpeed=std::min(targetSpeed,5.);
    if(changing){
      // Quintic lane change lateral acceleration <= 2.5 m/s^2.
      const double width=std::abs(offsets[target]-offsets[lane]);
      if(width>1e-4){
        const double laneSpeed=changeLength*std::sqrt(
              avoid::kMaxLatAccelMps2/(5.774*width));
        targetSpeed=std::min(targetSpeed,laneSpeed);
      }
    }
    if(std::isfinite(followingSpeed))
      targetSpeed=std::min(targetSpeed,followingSpeed);
    // Curvature/yaw/lateral caps are enforced at the point where the
    // curvature occurs (SpeedProfile.hh), then again in BoundedTracker.
    targetSpeed=clamp(targetSpeed,0.,baseSpeed);
    if(lapVerifier.invalid())targetSpeed=0.; // hard safety hold
    // Acceleration/deceleration slew limits in simulation time.
    // speed starts at 0 at spawn, ramps up smoothly, and never jumps
    // instantly from 10 to 3 at a corner.
    speed=clamp(targetSpeed,speed-maxDecel*dt,speed+maxAccel*dt);
    // The selected CSV lane is a lookahead reference, not an
    // instantaneous pose command. Bicycle integration bounds actual
    // turning curvature and yaw rate, including sudden map heading bends.
    const double lookahead=clamp(2.5+.35*speed,3.,7.5);
    const auto goal=poseAt(s+lookahead);
    const auto onTrack=poseAt(s);
    const auto actual=gz::sim::worldPose(ego,ecm);
    const bounded::Pose current{actual.Pos().X(),actual.Pos().Y(),
                                actual.Rot().Yaw()};
    const auto command=bounded::follow(
      current,goal.x,goal.y,speed,dt,wheelbase,maxSteer,maxYawRate,
      lastCommandSteer,1.57);
    lastCommandSteer=command.steer;
    const double trackError=std::hypot(current.x-onTrack.x,
                                       current.y-onTrack.y);
    if(physicsContact){
      const auto body=gz::sim::Model(ego).LinkByName(ecm,"base_link");
      if(body!=gz::sim::kNullEntity){
        const auto desired=gz::math::Pose3d(
          command.pose.x,command.pose.y,onTrack.z+.003,
          0.,0.,command.pose.yaw);
        physics::guide(body,ecm,actual,desired,speed,50.,dt,
                       egoPhysicalDrive);
      }
    }else{
      gz::sim::Model(ego).SetWorldPoseCmd(ecm,gz::math::Pose3d(
          command.pose.x,command.pose.y,onTrack.z+.003,
          0.,0.,command.pose.yaw));
    }
    s=std::fmod(s+speed*dt,lap);
    traveledFromStart+=speed*dt;
    const int estimatedLaps=static_cast<int>(traveledFromStart/std::max(1.,lap));
    if(estimatedLaps>estimatedLapCount){
      estimatedLapCount=estimatedLaps;
      std::cout<<"[TEAM11 ESTIMATED LAP] completed="<<estimatedLapCount
               <<" commanded_distance_m="<<traveledFromStart<<std::endl;
    }
    // Real base_link position, not commanded odometry, owns official practice
    // completion reporting. Reject off-track, backward or teleported runs.
    lapObserveClock+=dt;
    if(lapObserveClock>=.05){
      const auto valid=lapVerifier.Observe(current.x,current.y,actual.Pos().Z(),
                                           lapObserveClock,speed);
      lapObserveClock=0.;
      if(valid.invalid && !invalidReported){
        invalidReported=true;
        std::cout<<"[TEAM11 QA] INVALID reason="<<valid.reason
                 <<" actual_progress_m="<<valid.progress<<std::endl;
      }
      if(valid.lap){
        lapCount=valid.completed;
        std::cout<<"[TEAM11 LAP] completed="<<lapCount
                 <<" actual_progress_m="<<valid.progress
                 <<" route="<<(target+1)
                 <<" off_track=0"<<std::endl;
      }
    }
    trace+=dt;
    if(trace>=1.){
      trace=0.;
      std::cout<<"[TEAM11] lane="<<(target+1)
               <<" fixed="<<fixedLane
               <<" merge_progress_m="<<std::min(12.,traveledFromStart)
               <<" speed="<<speed
               <<"m/s steer="<<command.steer*180./M_PI
               <<"deg yaw_rate_deg_s="<<command.yawRate*180./M_PI
               <<" max_steer_deg="<<maxSteer*180./M_PI
               <<" track_error_m="<<trackError
               <<" speed_profile_cap="<<profileSpeedLimit
               <<" corner_approach_cap="<<cornerApproachCap
               <<" corner_mode="<<cornerMode<<" preview_curvature="
               <<futureCurvature<<" target="<<targetSpeed
               <<"m/s max_decel="<<maxDecel<<"m/s2 s="<<s<<"\n";
    }
  }
 private:
  // Camera is calibrated per grid to point at the five-lamp housing.
  // Test each individual expected lens, instead of requiring five clean
  // connected components; emissive glow can merge neighboring red blobs.
  int CountRedLamps(const gz::msgs::Image &image)const{
    const int w=static_cast<int>(image.width());
    const int h=static_cast<int>(image.height());
    const size_t stride=static_cast<size_t>(image.step());
    const auto &bytes=image.data();
    if(w<200 || h<100 || lampSpacingPx<2. ||
       stride<3u*static_cast<size_t>(w) ||
       bytes.size()<stride*static_cast<size_t>(h))return -1;
    const double dx=lampSpacingPx*w/1280.;
    if(dx<3. || 4.*dx>w*.70)return -1; // The five lamps cannot be resolved.
    const int centerX=w/2,centerY=h/2;
    const int radiusX=std::max(2,static_cast<int>(std::round(dx*.32)));
    const int radiusY=std::max(5,static_cast<int>(std::round(dx*.65)));
    int active=0;
    for(int lamp=0;lamp<5;lamp++){
      const int cx=static_cast<int>(std::round(centerX+(lamp-2)*dx));
      int redPixels=0;
      const int x0=std::max(0,cx-radiusX),x1=std::min(w-1,cx+radiusX);
      const int y0=std::max(0,centerY-radiusY),y1=std::min(h-1,centerY+radiusY);
      for(int y=y0;y<=y1;y++){
        for(int x=x0;x<=x1;x++){
          const size_t i=static_cast<size_t>(y)*stride+3u*x;
          const int r=static_cast<unsigned char>(bytes[i]);
          const int g=static_cast<unsigned char>(bytes[i+1]);
          const int b=static_cast<unsigned char>(bytes[i+2]);
          if(r>=100 && r>g*1.6 && r>b*1.6 &&
             r-g>=55 && r-b>=55)redPixels++;
        }
      }
      if(redPixels>=std::max(3,static_cast<int>((x1-x0+1)*(y1-y0+1)*.004)))
        active++;
    }
    return active;
  }
  void OnRearScan(const gz::msgs::LaserScan &laser){
    rear_scan::Scan next;
    next.angleMin=laser.angle_min();
    next.angleStep=laser.angle_step();
    next.rangeMin=laser.range_min();
    next.rangeMax=laser.range_max();
    next.ranges.reserve(static_cast<size_t>(laser.ranges_size()));
    for(int i=0;i<laser.ranges_size();i++)
      next.ranges.push_back(static_cast<float>(laser.ranges(i)));
    next.valid=!next.ranges.empty() && next.angleStep>0.;
    if(!next.valid)return;
    {
      std::lock_guard<std::mutex> guard(rearMutex);
      lastRearImage=std::move(next);
      rearArrival=std::chrono::steady_clock::now();
    }
    if(rearFrames.fetch_add(1)==0)
      std::cout<<"[TEAM11 REAR] first real LiDAR scan "
               <<laser.ranges_size()<<" beams topic="<<rearTopic
               <<std::endl;
  }
  void OnImage(const gz::msgs::Image &image){
    const int lamps=CountRedLamps(image);
    if(lamps<0)return; // Invalid or incomplete image is NOT lights-off.
    const auto count=imageFrames.fetch_add(1)+1;
    if(count==1 && !cameraReadyFile.empty()){
      std::ofstream readyStream(cameraReadyFile,std::ios::trunc);
      if(readyStream){
        readyStream<<"image_received=1 width="<<image.width()
                   <<" height="<<image.height()<<"\\n";
        readyStream.flush();
        std::cout<<"[TEAM11 CAMERA] first valid image received"
                 <<" width="<<image.width()<<" height="<<image.height()
                 <<std::endl;
      }else{
        std::cerr<<"[TEAM11 CAMERA ERROR] Cannot write camera readiness "
                 <<cameraReadyFile<<std::endl;
      }
    }
    lastLampCount.store(lamps);
    if(cameraStarted.load())return;
    if(lamps==5){
      offFrames.store(0);
      if(redFrames.fetch_add(1)+1>=2)fiveObserved.store(true);
    }else{
      redFrames.store(0);
      // Require zero visible red lamps, not merely <5.
      if(fiveObserved.load() && lamps==0){
        if(offFrames.fetch_add(1)+1>=4)cameraStarted.store(true);
      }else{
        offFrames.store(0);
      }
    }
  }
  Sample sample(int k,double distance)const{
    const auto &r=routes[k];
    distance=std::fmod(distance+lap,lap);
    auto it=std::upper_bound(r.begin(),r.end(),distance,
      [](double val,const Sample &p){return val<p.s;});
    if(it==r.begin())return r.front();
    const Sample &a=*(it-1);
    Sample b=it==r.end()?r.front():*it;
    double bs=it==r.end()?lap:b.s;
    const double t=clamp((distance-a.s)/std::max(1e-6,bs-a.s),0,1);
    b.yaw=unwrap(a.yaw,b.yaw);
    return {distance,a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,
            a.z+(b.z-a.z)*t,a.yaw+(b.yaw-a.yaw)*t};
  }
  void findEntities(gz::sim::EntityComponentManager &ecm){
    if(ego!=gz::sim::kNullEntity &&
       practice[1]!=gz::sim::kNullEntity)return;
    ecm.Each<gz::sim::components::Model,gz::sim::components::Name,
             gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &ent,const gz::sim::components::Model *,
          const gz::sim::components::Name *name,
          const gz::sim::components::ParentEntity *parent)->bool{
        if(parent->Data()!=world)return true;
        if(name->Data()==egoName)ego=ent;
        if(name->Data()=="urrc_pace_grid_02")practice[0]=ent;
        if(name->Data()=="urrc_pace_grid_09")practice[1]=ent;
        return true;
      });
  }
  gz::sim::Entity world{gz::sim::kNullEntity},ego{gz::sim::kNullEntity};
  std::array<gz::sim::Entity,2> practice{gz::sim::kNullEntity,gz::sim::kNullEntity};
  std::array<std::vector<Sample>,3> routes;
  std::string egoName,startMode,cameraTopic,cameraReadyFile,rearTopic;
  std::mutex rearMutex;
  rear_scan::Scan lastRearImage;
  std::chrono::steady_clock::time_point rearArrival{};
  std::atomic<unsigned int> rearFrames{0};
  gz::transport::Node node;
  std::atomic<bool> cameraStarted{false},fiveObserved{false};
  std::atomic<unsigned int> imageFrames{0},redFrames{0},offFrames{0};
  std::atomic<int> lastLampCount{-1};
  bool redReported{false};
  bool ready{false},started{false},changing{false},fixedLane{false},cornerMode{false};
  bool physicsContact{false};
  ShiftReason changeReason{ShiftReason::Front};
  physics::DriveState egoPhysicalDrive;
  lapcheck::Verifier lapVerifier;
  bool invalidReported{false};
  int estimatedLapCount{0};
  double lapObserveClock{0.};
  double lastCommandSteer{0.};
  int lane{1},target{1},homeLane{1},lapCount{0};
  int avoidIndex{-1};
  bool defending{false};
  int zigzagEndpoint{0};
  double lastDefenseTravel{-100.};
  std::array<double,2> lastLateral{0.,0.};
  std::array<double,2> lastLateralUpdate{-1.,-1.};
  std::array<double,2> lastLateralRate{0.,0.};
  std::array<int,2> weaveEvidence{0,0};
  std::array<int,2> rearEvidence{0,0};
  std::array<double,2> lastRearSample{-100.,-100.};
  std::array<double,2> lastRearTtc{1e9,1e9};
  std::array<bool,2> rearDeclining{false,false};
  std::array<unsigned int,2> lastRearFrameId{0,0};
  std::array<bool,2> observed{false,false};
  std::array<size_t,2> lastNearest{0,0};
  std::array<double,2> lastObsS{0.,0.},lastObsTime{-1.,-1.};
  std::array<double,2> measuredSpeed{10.,20.};
  double trackedAt{-100.};
  double lastChangeTravel{-100.};
  double lap{1},s{0},speed{0},spacing{.55},wheelbase{.33};
  double maxSteer{bounded::kSteer30},maxYawRate{bounded::kMaxYawRate};
  double baseSpeed{10},slowThreshold{10*M_PI/180.},slowFactor{.8};
  double changeLength{9},scanAhead{35},changeStart{0},trace{0};
  double maxDecel{4.},maxAccel{2.},cameraTrace{0.},lampSpacingPx{0.};
  double initialDx{0.},initialDy{0.},initialS{0.},traveledFromStart{0.};
};
}
GZ_ADD_PLUGIN(urrc::Team11ThreeLane,gz::sim::System,
  urrc::Team11ThreeLane::ISystemConfigure,
  urrc::Team11ThreeLane::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::Team11ThreeLane,"urrc::Team11ThreeLane")
