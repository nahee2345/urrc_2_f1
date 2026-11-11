#pragma once
// Physically resolved contact: follow a 3-route trajectory by FORCE and
// YAW TORQUE on a dynamic Gazebo body (never SetWorldPoseCmd).
// Not a detailed tire/steering-joint model; Gazebo resolves contact impulses
// between the actual 50kg ego and lighter 2.5kg pace bodies.
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <algorithm>
#include <cmath>

namespace urrc::physical {
inline double wrapAngle(double x) {return std::atan2(std::sin(x),std::cos(x));}
inline double clamp(double x,double a,double b){
  return std::max(a,std::min(b,x));
}
struct Target {double x,y,yaw,speed;};
inline void apply(const gz::sim::Link &link,gz::sim::EntityComponentManager &ecm,
                  const gz::math::Pose3d &pose,const Target &goal,
                  double mass,double yawInertia,double maxAccel) {
  if(!link.Valid(ecm))return;
  link.EnableVelocityChecks(ecm);
  const auto v=link.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
  const auto omega=link.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
  const double px=clamp(goal.x-pose.Pos().X(),-3.,3.);
  const double py=clamp(goal.y-pose.Pos().Y(),-3.,3.);
  // Stable position+feedforward velocity PD, limiting the applied FORCE.
  double ax=5.*(goal.speed*std::cos(goal.yaw)+1.15*px-v.X());
  double ay=5.*(goal.speed*std::sin(goal.yaw)+1.15*py-v.Y());
  const double a=std::hypot(ax,ay);
  if(a>maxAccel){ax*=maxAccel/a;ay*=maxAccel/a;}
  const double yawError=wrapAngle(goal.yaw-pose.Rot().Yaw());
  const double yawAcc=clamp(18.*yawError-5.0*omega.Z(),-9.,9.);
  link.AddWorldWrench(ecm,
      gz::math::Vector3d(mass*ax,mass*ay,0.),
      gz::math::Vector3d(0.,0.,yawInertia*yawAcc));
}
} // namespace urrc::physical
