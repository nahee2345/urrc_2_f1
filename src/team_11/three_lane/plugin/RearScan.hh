#pragma once
// Rear-facing LiDAR raw beam verification in the existing scripted Gazebo demo.
// Actual range returns are mandatory; model pose is used only to associate
// the return with the two existing scripted practice vehicles.
#include <cmath>
#include <limits>
#include <vector>

namespace urrc::rear_scan {
struct Scan {
  std::vector<float> ranges;
  double angleMin{-2.2},angleStep{0.0064};
  double rangeMin{.12},rangeMax{35.};
  bool valid{false};
};
// The rear sensor is mounted -0.20m longitudinally and rotated by pi.
// egoX: other vehicle ahead (+), behind (-); egoY: left (+), right (-).
inline bool verifiedReturn(const Scan &scan, double egoX, double egoY){
  if(!scan.valid || scan.ranges.empty())return false;
  const double sensorX=-(egoX+.20),sensorY=-egoY;
  if(sensorX<=.25)return false; // not actually behind sensor
  const double expected=std::hypot(sensorX,sensorY);
  if(expected>.99*scan.rangeMax || expected<scan.rangeMin)return false;
  const double bearing=std::atan2(sensorY,sensorX);
  // Body occupies roughly 0.25m across, allow a small angle window.
  const double angular=std::max(.025,.36/expected);
  for(size_t i=0;i<scan.ranges.size();i++){
    const double a=scan.angleMin+scan.angleStep*i;
    if(std::abs(a-bearing)>angular)continue;
    const double distance=scan.ranges[i];
    if(!std::isfinite(distance) || distance<scan.rangeMin ||
       distance>=scan.rangeMax)continue;
    if(std::abs(distance-expected)<=.9)return true;
  }
  return false;
}
} // namespace urrc::rear_scan
