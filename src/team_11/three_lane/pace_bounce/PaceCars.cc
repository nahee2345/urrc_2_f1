// Exhibition pace cars: kinematic offset-lane tracking, not participant AI.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
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
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

namespace urrc {
struct Point {double s,x,y,z,yaw;};
struct Car {
  std::string name;
  std::vector<Point> points;
  gz::sim::Entity entity{gz::sim::kNullEntity};
  double speed{0}, zOffset{0.003};
  double lastHit{-100.};
  double hitX{0},hitY{0};
};
class PaceCars : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &world,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override {
    this->worldEntity=world;
    egoName=sdf->Get<std::string>("team11_ego_name","").first;
    egoMass=sdf->Get<double>("team11_mass_kg",50.).first;
    otherMass=sdf->Get<double>("pace_mass_kg",1.5).first;
    const std::string topic = sdf->Get<std::string>("start_topic", "/urrc/pace/start").first;
    if (!this->node.Subscribe(topic, &PaceCars::OnStart, this))
      std::cerr << "[PACE] Cannot subscribe to " << topic << std::endl;
    this->readyPublisher = this->node.Advertise<gz::msgs::Boolean>(topic + "/ready");
    if (sdf->HasElement("car")) {
      auto element=sdf->FindElement("car");
      while (element) {
        Car car;
        car.name=element->Get<std::string>("name");
        car.speed=element->Get<double>("speed_mps");
        car.zOffset=element->Get<double>("z_offset_m");
        const auto path=element->Get<std::string>("route_file");
        std::ifstream file(path);
        if (!file.is_open()) {
          std::cerr << "[PACE] Missing route " << path << std::endl;
        } else {
          std::string line;
          std::getline(file,line); // CSV header
          while(std::getline(file,line)){
            std::replace(line.begin(),line.end(),',',' ');
            std::istringstream in(line);
            Point p{};
            if(in>>p.s>>p.x>>p.y>>p.z>>p.yaw)car.points.push_back(p);
          }
        }
        if(car.points.size()<2 || car.speed<=0 || car.points.back().s<=0)
          std::cerr << "[PACE] Invalid car/route " << car.name << std::endl;
        else this->cars.push_back(std::move(car));
        element=element->GetNextElement("car");
      }
    }
    std::cout << "[PACE] Armed " << cars.size() << " cars, waiting for all five lights OFF" << std::endl;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused)return;
    if(egoEntity==gz::sim::kNullEntity && !egoName.empty()){
      ecm.Each<gz::sim::components::Model,gz::sim::components::Name,
               gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity &ent,const gz::sim::components::Model *,
            const gz::sim::components::Name *name,
            const gz::sim::components::ParentEntity *parent)->bool{
          if(name->Data()==egoName && parent->Data()==worldEntity){
            egoEntity=ent;return false;
          }
          return true;
        });
    }
    if (this->startRequested.load() && !this->started){
      this->started=true;
      this->startTime=std::chrono::duration<double>(info.simTime).count();
      std::cout << "[PACE] ALL 5 LIGHTS OFF: all pace cars started" << std::endl;
    }
    if (!this->started)return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    const double elapsed=std::max(0.0,now-this->startTime);
    const bool hasEgo=egoEntity!=gz::sim::kNullEntity;
    const auto egoPose=hasEgo ? gz::sim::worldPose(egoEntity,ecm) : gz::math::Pose3d();
    for(auto &car : cars) {
      if(car.entity==gz::sim::kNullEntity){
        ecm.Each<gz::sim::components::Model,
                 gz::sim::components::Name,
                 gz::sim::components::ParentEntity>(
          [&](const gz::sim::Entity &ent,
              const gz::sim::components::Model *,
              const gz::sim::components::Name *name,
              const gz::sim::components::ParentEntity *parent)->bool{
            if(name->Data()==car.name && parent->Data()==this->worldEntity){
              car.entity=ent;return false;
            }
            return true;
          });
      }
      if(car.entity==gz::sim::kNullEntity)continue;
      const auto &path=car.points;
      const double lap=path.back().s;
      const double dist=std::fmod(elapsed*car.speed,lap);
      auto it=std::upper_bound(path.begin(),path.end(),dist,
          [](double s,const Point &p){return s<p.s;});
      if(it==path.begin())++it;
      if(it==path.end())it=path.end()-1;
      const auto &p1=*(it-1),&p2=*it;
      const double t=std::clamp((dist-p1.s)/std::max(1e-9,p2.s-p1.s),0.0,1.0);
      const double dyaw=std::atan2(std::sin(p2.yaw-p1.yaw),std::cos(p2.yaw-p1.yaw));
      gz::math::Pose3d pose(p1.x+(p2.x-p1.x)*t,p1.y+(p2.y-p1.y)*t,
                           p1.z+(p2.z-p1.z)*t+car.zOffset,
                           0,0,p1.yaw+dyaw*t);
      if(hasEgo) {
        const double dx=pose.Pos().X()-egoPose.Pos().X();
        const double dy=pose.Pos().Y()-egoPose.Pos().Y();
        const double d=std::hypot(dx,dy);
        // Scripted asymmetric impact: ego receives no recoil.
        if(d<0.39 && now-car.lastHit>2.5) {
          const double norm=std::max(d,1e-4);
          car.hitX=(d>1e-4 ? dx/norm : -std::sin(egoPose.Rot().Yaw()));
          car.hitY=(d>1e-4 ? dy/norm : std::cos(egoPose.Rot().Yaw()));
          car.lastHit=now;
          std::cout<<"[PACE BOUNCE] "<<car.name
                   <<" hit TEAM11, mass_kg="<<egoMass<<":"<<otherMass
                   <<", scripted no-ego-recoil"<<std::endl;
        }
        const double t=now-car.lastHit;
        if(t>=0. && t<2.0) {
          const double weight=std::pow(1.-t/2.,2.);
          const double lateral=2.2*(1.-std::exp(-5.*t))*weight;
          const double height=.65*std::sin(M_PI*t/2.)*weight;
          pose.Pos().X()+=car.hitX*lateral;
          pose.Pos().Y()+=car.hitY*lateral;
          pose.Pos().Z()+=std::max(0.,height);
          pose.Rot().Yaw(pose.Rot().Yaw()+.9*std::sin(M_PI*t/2.)*weight);
        }
      }
      gz::sim::Model(car.entity).SetWorldPoseCmd(ecm,pose);
    }
  }
 private:
  void OnStart(const gz::msgs::Boolean &msg){
    if(msg.data())this->startRequested.store(true);
  }
  gz::sim::Entity worldEntity{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher readyPublisher;
  std::vector<Car> cars;
  std::atomic<bool> startRequested{false};
  bool started{false};
  double startTime{0};
  std::string egoName;
  gz::sim::Entity egoEntity{gz::sim::kNullEntity};
  double egoMass{50.},otherMass{1.5};
};
} // namespace urrc
GZ_ADD_PLUGIN(urrc::PaceCars, gz::sim::System,
              urrc::PaceCars::ISystemConfigure,
              urrc::PaceCars::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::PaceCars, "urrc::PaceCars")
