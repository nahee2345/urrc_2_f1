#pragma once
// Dynamic (non-teleport) contact mode only: physics engine integrates contact.
// No SetWorldPoseCmd in dynamic mode; forces have an explicit magnitude cap.
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <algorithm>
#include <cmath>

namespace urrc::physics {
struct DriveState {
  bool initialized=false;
  gz::math::Vector3d previous;
  double yaw=0.;
};
inline double clip(double v,double lo,double hi){
  return std::max(lo,std::min(hi,v));
}
inline double wrapped(double target,double current){
  return std::atan2(std::sin(target-current),std::cos(target-current));
}
inline void guide(gz::sim::Entity linkEntity,
                  gz::sim::EntityComponentManager &ecm,
                  const gz::math::Pose3d &current,
                  const gz::math::Pose3d &desired,
                  double targetSpeed,double mass,double dt,
                  DriveState &state) {
  if(linkEntity==gz::sim::kNullEntity || dt<=0.||dt>.2)return;
  const auto xyz=current.Pos();
  if(!state.initialized){state.previous=xyz;state.yaw=current.Rot().Yaw();
                         state.initialized=true;}
  const double vx=(xyz.X()-state.previous.X())/dt;
  const double vy=(xyz.Y()-state.previous.Y())/dt;
  const double yawRate=wrapped(current.Rot().Yaw(),state.yaw)/dt;
  const double yaw=desired.Rot().Yaw();
  const double vxRef=std::max(0.,targetSpeed)*std::cos(yaw);
  const double vyRef=std::max(0.,targetSpeed)*std::sin(yaw);
  // Proportional/damped path following in Newtons, bounded to avoid
  // infinite stiffness and allow Gazebo solver to resolve contact.
  const double fx=clip(mass*(6.*(desired.Pos().X()-xyz.X())+
                                4.*(vxRef-vx)),-mass*25.,mass*25.);
  const double fy=clip(mass*(6.*(desired.Pos().Y()-xyz.Y())+
                                4.*(vyRef-vy)),-mass*25.,mass*25.);
  const double moment=clip(mass*(.60*wrapped(yaw,current.Rot().Yaw())-
                          .11*yawRate),-mass*1.8,mass*1.8);
  // Z is controlled by gravity / ground contact, NOT a lifting force.
  gz::sim::Link(linkEntity).AddWorldWrench(
    ecm,gz::math::Vector3d(fx,fy,0.),gz::math::Vector3d(0.,0.,moment));
  state.previous=xyz;
  state.yaw=current.Rot().Yaw();
}
} // namespace urrc::physics
