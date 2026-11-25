#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/model.pb.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
namespace team04 {
// Internal Twist: linear.x = m/s, angular.z = virtual steering radians (+left).
// Only joint torques are applied. No pose/velocity reset, no world modification.
class VehicleSystem: public gz::sim::System, public gz::sim::ISystemConfigure,
 public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
 std::array<gz::sim::Joint,4> joints;
 const std::array<std::string,4> names{{"team04_rear_left_spin","team04_rear_right_spin","team04_front_left_steer","team04_front_right_steer"}};
 gz::transport::Node node; gz::transport::Node::Publisher pub;
 std::array<double,2> integral{{0,0}},parkPosition{{0,0}};
 std::array<bool,2> parked{{false,false}};
 std::mutex mutex; double speed=0, steer=0; bool received=false;
 std::chrono::steady_clock::time_point receipt;
 std::chrono::steady_clock::duration lastSim{};
 public: void Configure(const gz::sim::Entity &e, const std::shared_ptr<const sdf::Element>&,
 gz::sim::EntityComponentManager &ecm,gz::sim::EventManager&) override {
  gz::sim::Model model(e);
  for(size_t i=0;i<4;++i){joints[i]=gz::sim::Joint(model.JointByName(ecm,names[i]));joints[i].EnablePositionCheck(ecm);joints[i].EnableVelocityCheck(ecm);}
  pub=node.Advertise<gz::msgs::Model>("/team04/gz/joint_states");
  node.Subscribe("/team04/gz/drive",&VehicleSystem::Command,this);
 }
 void Command(const gz::msgs::Twist &msg){
  std::lock_guard<std::mutex> lock(mutex);
  speed=std::isfinite(msg.linear().x())?std::clamp(msg.linear().x(),-2.0,5.0):0;
  steer=std::isfinite(msg.angular().z())?std::clamp(msg.angular().z(),-.6108652382,.6108652382):0;
  receipt=std::chrono::steady_clock::now();received=true;
 }
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
  if(info.paused)return;
  double v=0,d=0;
  {std::lock_guard<std::mutex> lock(mutex);
   if(received && std::chrono::steady_clock::now()-receipt<std::chrono::milliseconds(300)){v=speed;d=steer;}}
  // Limit virtual angle so the INSIDE front wheel never exceeds 35 degrees.
  constexpr double L=.32,T=.184,R=.035,limit=.6108652382;
  const double maxVirtual=std::atan(L/(L/std::tan(limit)+T/2));d=std::clamp(d,-maxVirtual,maxVirtual);
  const double curvature=std::tan(d)/L;
  const std::array<double,4> target{{v*(1-curvature*T/2)/R,v*(1+curvature*T/2)/R,
    std::atan(L*curvature/(1-curvature*T/2)),std::atan(L*curvature/(1+curvature*T/2))}};
  for(size_t i=0;i<4;++i){
   const auto vel=joints[i].Velocity(ecm); const auto pos=joints[i].Position(ecm);
   if(!vel || vel->empty() || !pos || pos->empty())continue;
   double torque=.04*(target[i]-vel->at(0));
   if(i<2){
    if(std::abs(v)>1e-6)parked[i]=false;
    else if(!parked[i] && std::abs(vel->at(0))<.2){parkPosition[i]=pos->at(0);parked[i]=true;}
    if(parked[i])torque=1.0*(parkPosition[i]-pos->at(0))-.04*vel->at(0);
   }else{
    const double error=target[i]-pos->at(0);
    const double dt=std::clamp(std::chrono::duration<double>(info.dt).count(),0.0,.005);
    integral[i-2]=std::clamp(integral[i-2]+.8*error*dt,-.2,.2);
    torque=.6*error-.006*vel->at(0)+integral[i-2];
   }
   joints[i].SetForce(ecm,{std::clamp(torque,i<2?-.6:-.8,i<2?.6:.8)});
  }
 }
 void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
  if(info.paused)return;
  if(info.simTime<lastSim)lastSim=info.simTime;
  if(info.simTime-lastSim<std::chrono::milliseconds(10))return;
  lastSim=info.simTime;
  gz::msgs::Model out;out.set_name("team04");
  const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
  out.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);out.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
  for(size_t i=0;i<4;++i){auto p=joints[i].Position(ecm),v=joints[i].Velocity(ecm);if(!p || !v || p->empty() || v->empty())continue;
   auto j=out.add_joint();j->set_name(names[i]);j->mutable_axis1()->set_position(p->at(0));j->mutable_axis1()->set_velocity(v->at(0));}
  pub.Publish(out);
 }
};
}
GZ_ADD_PLUGIN(team04::VehicleSystem,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
