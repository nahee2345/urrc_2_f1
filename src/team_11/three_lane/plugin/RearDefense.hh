#pragma once
// Independent, deterministic rear-defense decision module for TEAM11.
// Front passing, path tracking, camera start and pace collisions stay separate.
#include "OvertakePlanner.hh"
#include <cmath>
#include <limits>
#include <vector>

namespace urrc::defense {

constexpr double kRearDetectM=60.;
constexpr double kClosingMinMps=2.;
constexpr double kThreatTtcS=5.;
constexpr double kLaneTolerance=.40;
constexpr double kShiftEndGapM=7.;
constexpr double kDefenseCooldownM=28.;

struct RearTarget {
  int id{-1};
  double behindM{0.};
  double speed{0.};
  double lateral{0.};
  bool seenByRearLidar{false};
  bool uncertain{false};
};

struct State {
  avoid::State car;
  bool frontOvertakeActive{false};
  bool frontOvertakeRequested{false};
  double sinceDefenseChangeM{1e9};
  int zigzagEndpoint{0};  // 0=left, 2=right; step via middle lane
};

struct Decision {
  int nextLane{-1};
  int rearId{-1};
  double length{18.};
  bool active{false};
  bool zigzag{false};
  // A completed defense stays on its chosen lane: no home-lane return.
  double ttc{std::numeric_limits<double>::infinity()};
  const char* reason{"NO_THREAT"};
};

inline bool safeEntry(const avoid::State &state, int candidate,
                      double length, const std::vector<avoid::Car> &traffic,
                      int deliberateRearId) {
  if(candidate<0 || candidate>2 || std::abs(candidate-state.lane)!=1 ||
     state.changing)return false;
  const double from=avoid::laneOffset(state.lane,state.spacing);
  const double to=avoid::laneOffset(candidate,state.spacing);
  const double changeSpeed=std::max(3.,state.speed);
  const double duration=length/changeSpeed;
  // Never steer through another car; unlike the normal passing planner,
  // the targeted faster rear car may approach after a completed defense
  // merge. This is specific to the scripted collision practice environment.
  for(double t=0.;t<=duration+.001;t+=.15){
    const double ratio=avoid::smooth(t/duration);
    const double lateral=from+(to-from)*ratio;
    const double ourS=state.s+changeSpeed*t;
    for(const auto &car:traffic){
      const double gap=avoid::signedGap(
          ourS,car.s+car.speed*t,state.lap);
      if(std::abs(car.lateral-lateral)>.45)continue;
      if(std::abs(gap)<avoid::kVehicleOverlapM+1.)return false;
      if(t+0.16>=duration){
        if(car.id==deliberateRearId){
          if(gap> -kShiftEndGapM)return false;
        }else if(gap>-avoid::kRearGapM && gap<avoid::kFrontGapM)
          return false;
      }
    }
  }
  return true;
}

inline Decision evaluate(const State &s,
                         const std::vector<RearTarget> &rearTargets,
                         const std::vector<avoid::Car> &traffic) {
  Decision d;
  // Existing FRONT overtake or rejoin has priority over ALL defense.
  if(s.car.changing || s.frontOvertakeActive || s.frontOvertakeRequested ||
     s.car.travelled<12. ||
     s.sinceDefenseChangeM<kDefenseCooldownM){
    d.reason="FRONT_OR_CHANGE_PRIORITY";
    return d;
  }
  // Do not initiate blocking in medium or sharp corners.
  if(s.car.curve>=.030){
    d.reason="CORNER";
    return d;
  }
  const RearTarget* threat=nullptr;
  double soonest=std::numeric_limits<double>::infinity();
  for(const auto &car:rearTargets){
    if(!car.seenByRearLidar || car.behindM<=0. ||
       car.behindM>kRearDetectM)continue;
    const double closing=car.speed-s.car.speed;
    if(closing<kClosingMinMps)continue;
    const double ttc=car.behindM/closing;
    if(ttc<=kThreatTtcS && ttc<soonest){
      soonest=ttc;threat=&car;
    }
  }
  if(!threat)return d;
  d.active=true;
  d.ttc=soonest;
  d.rearId=threat->id;
  d.zigzag=threat->uncertain;
  d.reason=threat->uncertain?"ZIGZAG_CONFIRMED":"REAR_PASS_IMMINENT";

  int desired=-1;
  if(threat->uncertain) {
    // Zigzag endpoint alternates left/right, but crosses the middle lane
    // one safe adjacent step at a time. No direct 1<->3 teleport.
    desired=s.car.lane==1 ?
      s.zigzagEndpoint :
      1;
  } else {
    double best=1e9;
    for(int i=0;i<3;i++){
      if(i==s.car.lane)continue;
      const double error=std::abs(
          threat->lateral-avoid::laneOffset(i,s.car.spacing));
      if(error<kLaneTolerance && error<best){
        best=error;desired=i;
      }
    }
  }
  // If the faster approaching car has no OTHER lane we can intercept,
  // the only available defensive pattern is the user-requested stepped S.
  // It is still subject to the same continuous TTC and safe-entry gate.
  if(desired<0 || desired==s.car.lane){
    d.zigzag=true;
    d.reason="FAST_REAR_NO_BLOCKABLE_ROUTE";
    desired=(s.car.lane==1 ? s.zigzagEndpoint : 1);
  }
  if(desired==s.car.lane || desired<0 || desired>2)return d;
  // Only move to an adjacent candidate on the way to the desired target.
  const int candidate=s.car.lane+(desired>s.car.lane?1:-1);
  const double length=avoid::distanceToChange(
      std::max(s.car.speed,s.car.roadSpeed),s.car.spacing);
  if(!safeEntry(s.car,candidate,length,traffic,threat->id)){
    d.reason="UNSAFE_SHIFT";
    return d;
  }
  d.nextLane=candidate;
  d.length=length;
  return d;
}
} // namespace urrc::defense
