#pragma once
// TEAM11 corner preparation. Keeps the existing front passing and rear
// defense modules intact; only selects the center when it is safe to enter.
#include "OvertakePlanner.hh"
#include "SpeedProfile.hh"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace urrc::corner {
constexpr double kEnterCurvature=0.030;
constexpr double kExitCurvature=0.022;
constexpr double kSharpCurvature=0.080;
constexpr double kPreviewM=52.;
constexpr int kCenterLane=1;  // zero-based; officially route 2

struct Preview {
  double maxCurvature{0.};
  double firstBendDistance{std::numeric_limits<double>::infinity()};
  bool bendAhead{false};
};

struct Decision {
  int nextLane{-1};
  double length{18.};
  double speedCap{std::numeric_limits<double>::infinity()};
  bool approach{false};
  bool unsafe{false};
};

inline Decision evaluate(const avoid::State &car,const Preview &preview,
                         const std::vector<avoid::Car> &traffic,
                         bool frontPassingRequested,bool frontPassingActive,
                         bool fixedLane,double maxDecel=4.){
  Decision decision;
  if(fixedLane || !preview.bendAhead || car.lane==kCenterLane)
    return decision;
  decision.approach=true;
  // Prepare to join route 2 while still in route 1 or 3. The target speed
  // applies before the merge, not just once the steering change begins.
  const double atBend=preview.maxCurvature>=kSharpCurvature?5.:7.;
  // Distance-aware braking: lane 2 can be entered at higher speed while
  // ample space remains. Apex target is reached only near the actual bend.
  decision.speedCap=std::min(15.,speed_profile::cornerApproach(
      atBend,preview.firstBendDistance,maxDecel));
  if(car.changing || frontPassingRequested || frontPassingActive ||
     car.travelled<12.)
    return decision;
  const double length=avoid::distanceToChange(
    std::max(car.speed,7.),car.spacing);
  // A real path crossing must have room for both vehicles. No wall or
  // existing passing logic is altered to force the center crossing.
  if(!avoid::trajectoryClear(car,kCenterLane,length,traffic)){
    decision.unsafe=true;
    decision.speedCap=std::min(decision.speedCap,
      speed_profile::cornerApproach(
          5.,preview.firstBendDistance,maxDecel));
    return decision;
  }
  decision.nextLane=kCenterLane;
  decision.length=length;
  return decision;
}
} // namespace urrc::corner
