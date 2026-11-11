// TEAM11 3-lane kinematic practice controller; Gazebo Harmonic gz-sim8.
// Vehicle moves by pose command, not physical steering joints.
// It observes model poses in Gazebo and logs equivalent bicycle steering.
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
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
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

    // Gazebo world-pose obstacle tracking for the two practice cars.
    // This is not a camera / LiDAR obstacle detector.
    const std::array<double,3> offsets{spacing,0.,-spacing};
    struct Traffic {gz::sim::Entity id; double s, lateral, nominalSpeed;};
    std::vector<Traffic> traffic;
    const auto &middle=routes[1];
    for(size_t j=0;j<practice.size();j++){
      const auto other=practice[j];
      if(other==gz::sim::kNullEntity || other==ego)continue;
      const auto pose=gz::sim::worldPose(other,ecm);
      const double ox=pose.Pos().X(),oy=pose.Pos().Y();
      double best=1e99,obsS=0.;
      size_t closest=0;
      for(size_t i=0;i<middle.size();i++){
        const double d=std::hypot(ox-middle[i].x,oy-middle[i].y);
        if(d<best){best=d;obsS=middle[i].s;closest=i;}
      }
      const auto &p=middle[closest];
      const double lateral=-(ox-p.x)*std::sin(p.yaw)
                           +(oy-p.y)*std::cos(p.yaw);
      traffic.push_back({other,obsS,lateral,j==0?10.:20.});
    }
    auto forwardDistance=[&](double from,double to){
      return std::fmod(to-from+lap,lap);
    };
    auto laneFree=[&](int candidate){
      if(candidate<0 || candidate>2)return false;
      for(const auto &car:traffic){
        if(std::abs(car.lateral-offsets[candidate])>=.40)continue;
        const double ahead=forwardDistance(s,car.s);
        const double behind=forwardDistance(car.s,s);
        const double rearWindow=std::max(7.,(car.nominalSpeed-baseSpeed)*2.+7.);
        if(ahead<20. || behind<rearWindow)return false;
      }
      return true;
    };
    if(changing && forwardDistance(changeStart,s)>=changeLength-1e-5){
      changing=false;
      lane=target;
      std::cout<<"[TEAM11 AVOID] finished lane="<<(target+1)<<std::endl;
    }
    bool brakeForTraffic=false;
    if(!fixedLane && !changing && traveledFromStart>=12.){
      bool passed=false;
      for(const auto &car:traffic)if(car.id==avoidEntity){
        const double behind=forwardDistance(car.s,s);
        passed=behind>=6. && behind<lap/2.;
      }
      if(target!=homeLane && avoidEntity!=gz::sim::kNullEntity &&
         passed && laneFree(homeLane) &&
         traveledFromStart-lastChangeTravel>=changeLength+6.){
        lane=target;target=homeLane;changeStart=s;changing=true;
        lastChangeTravel=traveledFromStart;
        avoidEntity=gz::sim::kNullEntity;
        std::cout<<"[TEAM11 AVOID] return "<<(lane+1)<<" -> "
                 <<(target+1)<<" after passing"<<std::endl;
      }else if(traveledFromStart-lastChangeTravel>=changeLength+4.){
        double risk=1e99;
        gz::sim::Entity blocker=gz::sim::kNullEntity;
        for(const auto &car:traffic){
          if(std::abs(car.lateral-offsets[target])>=.40)continue;
          const double ahead=forwardDistance(s,car.s);
          const double closing=std::max(0.,baseSpeed-car.nominalSpeed);
          const double trigger=clamp(8.+closing*3.,10.,30.);
          if(closing>.5 && ahead>=2. && ahead<trigger && ahead<risk){
            risk=ahead;blocker=car.id;
          }
        }
        if(blocker!=gz::sim::kNullEntity){
          int next=-1;
          for(int candidate:{target-1,target+1}){
            if(laneFree(candidate)){next=candidate;break;}
          }
          if(next>=0){
            lane=target;target=next;changeStart=s;changing=true;
            lastChangeTravel=traveledFromStart;
            avoidEntity=blocker;
            std::cout<<"[TEAM11 AVOID] pass "<<(lane+1)<<" -> "
                     <<(target+1)<<" obstacle_distance="<<risk<<"m"
                     <<std::endl;
          }else{
            brakeForTraffic=true;
            std::cout<<"[TEAM11 AVOID] no adjacent free lane, braking"
                     <<std::endl;
          }
        }
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
    // Preview 20m of curvature; braking 10 -> 3m/s at 4m/s^2
    // needs about 11.4m, so start decelerating before corner entry.
    double futureCurvature=std::abs(curvature);
    for(double look=0.;look<=20.;look+=2.){
      const auto a=sample(1,s+look);
      const auto b=sample(1,s+look+2.);
      const double delta=std::atan2(std::sin(b.yaw-a.y),std::cos(b.yaw-a.y));
      futureCurvature=std::max(futureCurvature,std::abs(delta)/2.);
    }
    double targetSpeed=baseSpeed;
    if(futureCurvature<.012 && std::abs(steer)<=slowThreshold){
      targetSpeed=baseSpeed;
    }else if(futureCurvature<.025){
      targetSpeed=7.;
    }else if(futureCurvature<.055){
      targetSpeed=7.-2.*(futureCurvature-.025)/.030;
    }else if(futureCurvature<.10){
      targetSpeed=5.-2.*(futureCurvature-.055)/.045;
    }else{
      targetSpeed=3.;
    }
    if(std::abs(steer)>slowThreshold)targetSpeed=std::min(targetSpeed,7.);
    if(std::abs(steer)>2.*slowThreshold)targetSpeed=std::min(targetSpeed,3.);
    if(changing)targetSpeed=std::min(targetSpeed,7.);
    if(brakeForTraffic)targetSpeed=0.;
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
    trace+=dt;
    if(trace>=1.){
      trace=0.;
      std::cout<<"[TEAM11] lane="<<(target+1)
               <<" fixed="<<fixedLane
               <<" merge_progress_m="<<std::min(12.,traveledFromStart)
               <<" speed="<<speed
               <<"m/s steer="<<steer*180./M_PI<<"deg preview_curvature="
               <<futureCurvature<<" target="<<targetSpeed
               <<"m/s max_decel="<<maxDecel<<"m/s2 s="<<s<<"\n";
    }
  }
 private:
  // The gantry has FIVE horizontally separated red lamps.
  // A single red-pixel count cannot establish that all five were on.
  // Count coherent red columns and reject unrelated red objects.
  static int CountRedLamps(const gz::msgs::Image &image) {
    const int w=static_cast<int>(image.width());
    const int h=static_cast<int>(image.height());
    const size_t stride=static_cast<size_t>(image.step());
    const auto &bytes=image.data();
    if(w<100 || h<80 || stride<3u*static_cast<size_t>(w) ||
       bytes.size()<stride*static_cast<size_t>(h))return -1;
    const int x0=w/5,x1=w*4/5,y0=h/12,y1=h*2/3;
    std::vector<int> column(w,0);
    std::vector<double> rowSum(w,0.);
    for(int y=y0;y<y1;y++) {
      for(int x=x0;x<x1;x++) {
        const size_t i=static_cast<size_t>(y)*stride+3u*x;
        const int r=static_cast<unsigned char>(bytes[i]);
        const int g=static_cast<unsigned char>(bytes[i+1]);
        const int b=static_cast<unsigned char>(bytes[i+2]);
        if(r>=135 && r>g*2 && r>b*2 && g<95 && b<95){
          column[x]++;
          rowSum[x]+=y;
        }
      }
    }
    struct Blob {double x,y;int pixels,width;};
    std::vector<Blob> blobs;
    int left=-1,pixels=0;double totalY=0.;
    for(int x=x0;x<=x1;x++){
      const bool active=x<x1 && column[x]>=2;
      if(active){
        if(left<0){left=x;pixels=0;totalY=0.;}
        pixels+=column[x];totalY+=rowSum[x];
      }else if(left>=0){
        const int width=x-left;
        if(width>=2 && width<=w/12 && pixels>=8){
          blobs.push_back({(left+x-1)*.5,totalY/pixels,pixels,width});
        }
        left=-1;
      }
    }
    // Lamps share one row and have approximately even horizontal spacing.
    // Noise may create extra blobs; look for a five-lamp subset.
    if(blobs.size()<5)return static_cast<int>(std::min<size_t>(blobs.size(),4));
    for(size_t a=0;a+4<blobs.size();a++){
      for(size_t b=a+4;b<blobs.size();b++){
        const double gap=(blobs[b].x-blobs[a].x)/4.;
        if(gap<4. || gap>w/10.)continue;
        int matched=0;
        for(int j=0;j<5;j++){
          const double center=blobs[a].x+j*gap;
          bool found=false;
          for(const auto &candidate:blobs){
            if(std::abs(candidate.x-center)<=std::max(2.,gap*.22) &&
               std::abs(candidate.y-blobs[a].y)<std::max(6.,h*.025)){
              found=true;break;
            }
          }
          if(found)matched++;
        }
        if(matched==5)return 5;
      }
    }
    return 0;
  }
  void OnImage(const gz::msgs::Image &image){
    const int lamps=CountRedLamps(image);
    if(lamps<0)return; // Invalid or incomplete image is NOT lights-off.
    imageFrames.fetch_add(1);
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
  std::string egoName,startMode,cameraTopic;
  gz::transport::Node node;
  std::atomic<bool> cameraStarted{false},fiveObserved{false};
  std::atomic<unsigned int> imageFrames{0},redFrames{0},offFrames{0};
  std::atomic<int> lastLampCount{-1};
  bool redReported{false};
  bool ready{false},started{false},changing{false},fixedLane{false};
  int lane{1},target{1},homeLane{1};
  gz::sim::Entity avoidEntity{gz::sim::kNullEntity};
  double lastChangeTravel{-100.};
  double lap{1},s{0},speed{0},spacing{.55},wheelbase{.33};
  double baseSpeed{10},slowThreshold{10*M_PI/180.},slowFactor{.8};
  double changeLength{9},scanAhead{35},changeStart{0},trace{0};
  double maxDecel{4.},maxAccel{2.},cameraTrace{0.};
  double initialDx{0.},initialDy{0.},initialS{0.},traveledFromStart{0.};
};
}
GZ_ADD_PLUGIN(urrc::Team11ThreeLane,gz::sim::System,
  urrc::Team11ThreeLane::ISystemConfigure,
  urrc::Team11ThreeLane::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::Team11ThreeLane,"urrc::Team11ThreeLane")
