#pragma once
// Single-track kinematic bicycle follower. Unlike SetWorldPoseCmd along a
// CSV curve, this integrates only reachable turns: +/-30deg steer, bounded
// path curvature and bounded yaw rate. The original CSV is a reference,
// not a source of instantaneous heading / position teleports.
#include <algorithm>
#include <cmath>
#include <limits>

namespace urrc::bounded {
constexpr double kPi=3.1415926535897932384626433832795;
constexpr double kSteer30=kPi/6.;
constexpr double kMaxYawRate=1.20; // radians/second; ~68.75deg/s
constexpr double kMaxLateralAcceleration=2.5; // m/s^2
inline double clamp(double x,double lo,double hi){
  return std::max(lo,std::min(hi,x));
}
inline double wrap(double delta){
  return std::atan2(std::sin(delta),std::cos(delta));
}
inline double curvatureLimit(double wheelbase,double maxSteer=kSteer30){
  return std::tan(maxSteer)/wheelbase;
}
inline double speedForCurvature(double curve,double maxYawRate=kMaxYawRate){
  const double absCurve=std::abs(curve);
  if(absCurve<=1.e-8)return std::numeric_limits<double>::infinity();
  return std::min(std::sqrt(kMaxLateralAcceleration/absCurve),
                  maxYawRate/absCurve);
}
struct Pose {
  double x{0.},y{0.},yaw{0.};
};
struct Step {
  Pose pose;
  double steer{0.};      // radians; never > +/-30deg
  double curvature{0.};  // actual signed 1/m
  double yawRate{0.};    // rad/s; never > maxYawRate
  double targetError{0.};
};
inline Step follow(const Pose &current,double targetX,double targetY,
                   double speed,double dt,double wheelbase,
                   double maxSteer=kSteer30,double maxYawRate=kMaxYawRate) {
  Step result;
  result.pose=current;
  const double dx=targetX-current.x,dy=targetY-current.y;
  const double distance=std::hypot(dx,dy);
  result.targetError=distance;
  if(dt<=0. || dt>.2 || wheelbase<=0. || !std::isfinite(distance) ||
     !std::isfinite(speed) || !std::isfinite(current.yaw))
    return result;
  // Pure-pursuit curvature from the current heading to the selected
  // lane's future reference. The signed yaw is an actual reachable command.
  const double angle=wrap(std::atan2(dy,dx)-current.yaw);
  const double rawCurvature=(distance>0.3)?
     2.*std::sin(angle)/distance:0.;
  const double limited=clamp(rawCurvature,
                  -curvatureLimit(wheelbase,maxSteer),
                   curvatureLimit(wheelbase,maxSteer));
  const double velocity=std::max(0.,speed);
  const double limitedRate=clamp(velocity*limited,-maxYawRate,maxYawRate);
  result.yawRate=limitedRate;
  result.curvature=velocity>1.e-5 ? limitedRate/velocity : 0.;
  result.steer=std::atan(wheelbase*result.curvature);
  const double dYaw=limitedRate*dt;
  const double halfway=current.yaw+.5*dYaw;
  result.pose.x+=velocity*dt*std::cos(halfway);
  result.pose.y+=velocity*dt*std::sin(halfway);
  result.pose.yaw=wrap(current.yaw+dYaw);
  return result;
}
} // namespace urrc::bounded
