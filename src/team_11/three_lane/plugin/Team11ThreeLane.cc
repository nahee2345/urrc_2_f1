// TEAM11 3-lane kinematic practice controller; Gazebo Harmonic gz-sim8.
// Vehicle moves by pose command, not physical steering joints.
// It observes model poses in Gazebo and logs equivalent bicycle steering.
#include "OvertakePlanner.hh"
#include "RearDefense.hh"
#include "RearScan.hh"
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
    spacing=cfg->Get<double>("spacing_m");
    wheelbase=cfg->Get<double>("wheelbase_m");
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
        if(!seen)continue;
        // Same rear target is tracked across multiple simulation frames.
        const size_t idx=static_cast<size_t>(car.id);
        const bool betweenLanes=std::none_of(
          offsets.begin(),offsets.end(),[&](double o){
            return std::abs(o-car.lateral)<.30;
          });
        bool weaving=betweenLanes;
        const double age=now-lastLateralUpdate[idx];
        if(lastLateralUpdate[idx]<0. || age>=.18){
          if(lastLateralUpdate[idx]>=0. && age<2.){
            const double rate=std::abs(
              car.lateral-lastLateral[idx])/std::max(.01,age);
            if(rate>.55)weaving=true;
          }
          lastLateralUpdate[idx]=now;
          lastLateral[idx]=car.lateral;
        }
        rearTargets.push_back({car.id,behind,car.speed,
                               car.lateral,seen,weaving});
      }
    }
    // Classify the road before deciding whether a safe passing gap exists.
    // This is independent of temporary target lane selection.
    double roadCurve=0.;
    for(double look=0.;look<=36.;look+=3.){
      const auto a=sample(1,s+look);
      const auto b=sample(1,s+look+2.);
      const auto c=sample(1,s+look+4.);
      const double ya=std::atan2(b.y-a.y,b.x-a.x);
      const double yb=std::atan2(c.y-b.y,c.x-b.x);
      const double delta=std::atan2(std::sin(yb-ya),std::cos(yb-ya));
      roadCurve=std::max(roadCurve,std::abs(delta)/
                            std::max(.4,std::hypot(c.x-b.x,c.y-b.y)));
    }
    if(changing && avoid::signedGap(changeStart,s,lap)>=changeLength-1e-5){
      changing=false;lane=target;
      std::cout<<"[TEAM11 AVOID] finished lane="<<(lane+1)<<std::endl;
    }
    double followingSpeed=std::numeric_limits<double>::infinity();
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
      if(decision.nextLane>=0 && !changing){
        defending=false;
        const int oldLane=target;
        lane=target;target=decision.nextLane;
        changing=true;changeStart=s;
        changeLength=decision.changeLength;
        lastChangeTravel=traveledFromStart;
        avoidIndex=decision.returning?-1:decision.avoidId;
        std::cout<<"[TEAM11 AVOID] "
                 <<(decision.returning?"return ":"pass ")
                 <<(oldLane+1)<<" -> "<<(target+1)
                 <<" length_m="<<changeLength
                 <<" current_mps="<<speed
                 <<" road_curve="<<roadCurve<<std::endl;
      }else if(!changing && rearDecision.nextLane>=0 &&
               !decision.blocked){
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
      }else if(!changing && defending &&
               !rearDecision.active && target!=homeLane &&
               traveledFromStart-lastDefenseTravel>=18. &&
               !decision.blocked){
        const int restore=(target<homeLane?target+1:target-1);
        const double length=avoid::distanceToChange(
          std::max(speed,state.roadSpeed),spacing);
        if(avoid::trajectoryClear(state,restore,length,traffic)){
          lane=target;target=restore;
          changing=true;changeStart=s;changeLength=length;
          lastChangeTravel=traveledFromStart;
          lastDefenseTravel=traveledFromStart;
          if(target==homeLane)defending=false;
          std::cout<<"[TEAM11 DEFENSE] clear -> return "
                   <<(lane+1)<<" -> "<<(target+1)<<std::endl;
        }
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
    const double steer=std::atan(wheelbase*curvature);
    // Preview 36m of ACTUAL XY trajectory geometry, not waypoint yaw.
    // Earlier raw yaw preview reported spurious ~1.5rad/m on straights.
    // Braking from 15 to 3m/s at 4m/s2 needs 27m.
    double futureCurvature=std::abs(curvature);
    for(double look=0.;look<=36.;look+=3.){
      const auto a=poseAt(s+look);
      const auto b=poseAt(s+look+2.);
      const auto c=poseAt(s+look+4.);
      const double first=std::atan2(b.y-a.y,b.x-a.x);
      const double second=std::atan2(c.y-b.y,c.x-b.x);
      const double turn=std::atan2(std::sin(second-first),
                                   std::cos(second-first));
      const double distance=std::max(.4,std::hypot(c.x-b.x,c.y-b.y));
      futureCurvature=std::max(futureCurvature,std::abs(turn)/distance);
    }
    // Straight and gentle curves use the same 10-15 m/s policy.
    // Ordinary and sharp curves both use a 5-7 m/s target.
    // Hysteresis prevents repeated acceleration/braking at the threshold.
    constexpr double kCornerEnter=0.030;
    constexpr double kCornerExit=0.022;
    constexpr double kSharp=0.080;
    if(!cornerMode && futureCurvature>=kCornerEnter)cornerMode=true;
    else if(cornerMode && futureCurvature<=kCornerExit)cornerMode=false;
    double targetSpeed=baseSpeed;
    if(cornerMode){
      const double blend=clamp((futureCurvature-kCornerEnter)/
                               (kSharp-kCornerEnter),0.,1.);
      targetSpeed=7.-2.*blend; // 7 m/s ordinary; 5 m/s sharp
    }
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
    targetSpeed=clamp(targetSpeed,0.,baseSpeed);
    // Acceleration/deceleration slew limits in simulation time.
    // speed starts at 0 at spawn, ramps up smoothly, and never jumps
    // instantly from 10 to 3 at a corner.
    speed=clamp(targetSpeed,speed-maxDecel*dt,speed+maxAccel*dt);
    const auto pos=poseAt(s);
    gz::sim::Model(ego).SetWorldPoseCmd(ecm,gz::math::Pose3d(
        pos.x,pos.y,pos.z+.003,0,0,yaw0));
    s=std::fmod(s+speed*dt,lap);
    traveledFromStart+=speed*dt;
    const int completedLaps=static_cast<int>(traveledFromStart/std::max(1.,lap));
    if(completedLaps>lapCount){
      lapCount=completedLaps;
      std::cout<<"[TEAM11 LAP] completed="<<lapCount
               <<" distance_m="<<traveledFromStart
               <<" route="<<(target+1)<<std::endl;
    }
    trace+=dt;
    if(trace>=1.){
      trace=0.;
      std::cout<<"[TEAM11] lane="<<(target+1)
               <<" fixed="<<fixedLane
               <<" merge_progress_m="<<std::min(12.,traveledFromStart)
               <<" speed="<<speed
               <<"m/s steer="<<steer*180./M_PI
               <<"deg corner_mode="<<cornerMode<<" preview_curvature="
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
  int lane{1},target{1},homeLane{1},lapCount{0};
  int avoidIndex{-1};
  bool defending{false};
  int zigzagEndpoint{0};
  double lastDefenseTravel{-100.};
  std::array<double,2> lastLateral{0.,0.};
  std::array<double,2> lastLateralUpdate{-1.,-1.};
  std::array<bool,2> observed{false,false};
  std::array<size_t,2> lastNearest{0,0};
  std::array<double,2> lastObsS{0.,0.},lastObsTime{-1.,-1.};
  std::array<double,2> measuredSpeed{10.,20.};
  double trackedAt{-100.};
  double lastChangeTravel{-100.};
  double lap{1},s{0},speed{0},spacing{.55},wheelbase{.33};
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
