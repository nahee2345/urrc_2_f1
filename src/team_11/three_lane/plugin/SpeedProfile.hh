#pragma once
// Apply curvature-related deceleration WHEN the bend is reached, not as
// soon as an apex first appears anywhere inside a long preview window.
#include "BoundedTracker.hh"
#include <algorithm>
#include <cmath>
#include <limits>

namespace urrc::speed_profile {
constexpr double kBrakeBufferM=4.0; // margin for imperfect curvature preview
inline double curveTarget(double curvature,double wheelbase,
                          double maxSteer,double maxYawRate){
  const double k=std::abs(curvature);
  if(k<1.e-8)return 15.;
  const double road=k<.030?15.:
       std::max(5.,std::min(7.,7.-2.*(k-.030)/.050));
  // Impossible local curve: stop instead of steering beyond +/-30deg.
  if(k>bounded::curvatureLimit(wheelbase,maxSteer)+1.e-6)return 0.;
  return std::min(road,bounded::speedForCurvature(k,maxYawRate));
}
inline double allowedNow(double curvature,double distanceAhead,
                         double maxDecel,double wheelbase,
                         double maxSteer,double maxYawRate,
                         double margin=kBrakeBufferM){
  const double atCurve=curveTarget(curvature,wheelbase,maxSteer,maxYawRate);
  const double room=std::max(0.,distanceAhead-margin);
  return std::sqrt(atCurve*atCurve+
                   2.*std::max(0.,maxDecel)*room);
}
inline double cornerApproach(double desiredAtBend,
                             double distanceToBend,double maxDecel,
                             double margin=kBrakeBufferM){
  const double room=std::max(0.,distanceToBend-margin);
  return std::sqrt(desiredAtBend*desiredAtBend+
                   2.*std::max(0.,maxDecel)*room);
}
} // namespace urrc::speed_profile
