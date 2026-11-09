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
    obstacleName=cfg->Get<std::string>("obstacle_name");
    lane=cfg->Get<int>("initial_lane");
    target=lane;
    spacing=cfg->Get<double>("spacing_m");
    wheelbase=cfg->Get<double>("wheelbase_m");
    baseSpeed=cfg->Get<double>("base_speed_mps");
    slowThreshold=cfg->Get<double>("steering_slow_threshold_deg")*M_PI/180.;
    slowFactor=cfg->Get<double>("slow_factor");
    changeLength=cfg->Get<double>("lane_change_length_m");
    scanAhead=cfg->Get<double>("scan_ahead_m");
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
    const std::string topic=cfg->Get<std::string>("start_topic");
    node.Subscribe(topic,&Team11ThreeLane::OnStart,this);
    ready=true;
    std::cout<<"[TEAM11] 3 routes armed, spacing "<<spacing<<"m, obstacle "
             <<obstacleName<<", waiting lights OFF\n";
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if(info.paused || !ready)return;
    findEntities(ecm);
    if(ego==gz::sim::kNullEntity || obstacle==gz::sim::kNullEntity)return;
    if(!startRequested.load())return;
    if(!started){
      const auto pose=gz::sim::worldPose(ego,ecm);
      const double x=pose.Pos().X(),y=pose.Pos().Y();
      double best=1e99;
      const auto &r=routes[1];
      for(const auto &p:r){
        double d=std::hypot(x-p.x,y-p.y);
        if(d<best){best=d;s=p.s;}
      }
      started=true;
      std::cout<<"[TEAM11] GREEN start s="<<s<<" lane="<<lane<<"\n";
    }
    double dt=std::chrono::duration<double>(info.dt).count();
    if(dt<=0 || dt>0.2)return;
    // Find stationary demo blocker in actual Gazebo world coordinates.
    auto obstaclePose=gz::sim::worldPose(obstacle,ecm);
    const double ox=obstaclePose.Pos().X(),oy=obstaclePose.Pos().Y();
    const auto &middle=routes[1];
    double dmin=1e99,obsS=0;
    int obsIndex=0;
    for(size_t i=0;i<middle.size();i++){
      const double d=std::hypot(ox-middle[i].x,oy-middle[i].y);
      if(d<dmin){dmin=d;obsS=middle[i].s;obsIndex=static_cast<int>(i);}
    }
    const double dx=ox-middle[obsIndex].x,dy=oy-middle[obsIndex].y;
    const double normal=-dx*std::sin(middle[obsIndex].yaw)
                        +dy*std::cos(middle[obsIndex].yaw);
    const double ahead=std::fmod(obsS-s+lap,lap);
    auto offsets=std::array<double,3>{spacing,0.,-spacing};
    bool blocked[3]={false,false,false};
    if(ahead>1.0 && ahead<scanAhead){
      for(int k=0;k<3;k++){
        // .25 car width, .25 blocker width, 0.12 extra clearance.
        blocked[k]=std::abs(normal-offsets[k]) < .37;
      }
    }
    if(!changing && blocked[target]){
      int next=-1;
      // Only adjacent paths. No jumping over a blocked lane.
      for(int j: {target-1,target+1}){
        if(j>=0 && j<3 && !blocked[j]){next=j;break;}
      }
      if(next>=0){
        lane=target;target=next;
        changeStart=s;changing=true;
        std::cout<<"[TEAM11] BLOCKER "<<ahead<<"m: "
                 <<lane<<" -> "<<target<<" (keep new lane)\n";
      } else {
        speed=0;
        std::cout<<"[TEAM11] all adjacent lanes blocked; stop\n";
        return;
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
      return Position{mid.x-o*std::sin(mid.yaw),mid.y+o*std::cos(mid.yaw),mid.z};
    };
    const auto p0=poseAt(s),p1=poseAt(s+.4),p2=poseAt(s+.8);
    const double yaw0=std::atan2(p1.y-p0.y,p1.x-p0.x);
    const double yaw1=std::atan2(p2.y-p1.y,p2.x-p1.x);
    double yawChange=std::atan2(std::sin(yaw1-yaw0),std::cos(yaw1-yaw0));
    const double curvature=yawChange/std::max(.05,std::hypot(p1.x-p0.x,p1.y-p0.y));
    const double steer=std::atan(wheelbase*curvature);
    speed=baseSpeed*(std::abs(steer)>slowThreshold?slowFactor:1.);
    const auto pos=poseAt(s);
    gz::sim::Model(ego).SetWorldPoseCmd(ecm,gz::math::Pose3d(
        pos.x,pos.y,pos.z+.003,0,0,yaw0));
    s=std::fmod(s+speed*dt,lap);
    trace+=dt;
    if(trace>=1.){
      trace=0.;
      std::cout<<"[TEAM11] lane="<<target<<" speed="<<speed
               <<"m/s steer="<<steer*180./M_PI<<"deg s="<<s<<"\n";
    }
  }
 private:
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
    if(ego!=gz::sim::kNullEntity && obstacle!=gz::sim::kNullEntity)return;
    ecm.Each<gz::sim::components::Model,gz::sim::components::Name,
             gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &ent,const gz::sim::components::Model *,
          const gz::sim::components::Name *name,
          const gz::sim::components::ParentEntity *parent)->bool{
        if(parent->Data()!=world)return true;
        if(name->Data()==egoName)ego=ent;
        if(name->Data()==obstacleName)obstacle=ent;
        return true;
      });
  }
  void OnStart(const gz::msgs::Boolean &msg){if(msg.data())startRequested.store(true);}
  gz::sim::Entity world{gz::sim::kNullEntity},ego{gz::sim::kNullEntity},
                   obstacle{gz::sim::kNullEntity};
  gz::transport::Node node;
  std::atomic<bool> startRequested{false};
  std::array<std::vector<Sample>,3> routes;
  std::string egoName,obstacleName;
  bool ready{false},started{false},changing{false};
  int lane{1},target{1};
  double lap{1},s{0},speed{0},spacing{.55},wheelbase{.33};
  double baseSpeed{10},slowThreshold{10*M_PI/180.},slowFactor{.8};
  double changeLength{9},scanAhead{35},changeStart{0},trace{0};
};
}
GZ_ADD_PLUGIN(urrc::Team11ThreeLane,gz::sim::System,
  urrc::Team11ThreeLane::ISystemConfigure,
  urrc::Team11ThreeLane::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::Team11ThreeLane,"urrc::Team11ThreeLane")
