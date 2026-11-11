#pragma once
// TEAM11 three-lane overtaking planner: deterministic, dependency-free logic.
// Inputs are measured Gazebo track coordinates (s, lateral) and estimated speed.
// This planner does NOT claim camera/LiDAR obstacle perception.
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace urrc::avoid {
constexpr double kDetectM=50.;
constexpr double kTriggerM=35.;
constexpr double kTtcS=6.;
constexpr double kMinClosingMps=2.;
constexpr double kFrontGapM=10.;
constexpr double kRearGapM=10.;
constexpr double kMinChangeM=18.;
constexpr double kMaxLatAccelMps2=2.5;
constexpr double kPreviewS=4.;
constexpr double kVehicleOverlapM=2.0;

inline double bound(double v,double lo,double hi) {
  return std::max(lo,std::min(hi,v));
}
inline double smooth(double progress) {
  double p=bound(progress,0.,1.);
  return p*p*p*(10.+p*(-15.+6.*p));
}
inline double signedGap(double source,double dest,double lap) {
  if(lap<=0.)return dest-source;
  double gap=std::fmod(dest-source,lap);
  if(gap>lap/2.)gap-=lap;
  if(gap< -lap/2.)gap+=lap;
  return gap;
}
inline double curvatureSpeed(double curve) {
  if(curve<.030)return 15.;
  return bound(7.-2.*(curve-.030)/.050,5.,7.);
}
struct Car {
  int id;
  double s;
  double lateral;
  double speed;
};
struct State {
  double s=0., lap=1., speed=0., roadSpeed=15., spacing=.765;
  double curve=0., travelled=0., sinceChange=1e9;
  int lane=2, homeLane=2, avoidId=-1;
  bool changing=false;
};
struct Decision {
  int nextLane=-1; // -1 means retain current target
  int avoidId=-1;
  double changeLength=kMinChangeM;
  double followSpeed=std::numeric_limits<double>::infinity();
  bool returning=false;
  bool blocked=false;
};
inline double laneOffset(int lane,double spacing) {
  return (1-lane)*spacing; // lane 0: left, lane 2: right
}
inline double distanceToChange(double speed,double widthM) {
  // Maximum of quintic d²(shift)/dt² is ~5.774*width/duration².
  const double byAcceleration=std::max(0.,speed) *
      std::sqrt(5.774*widthM/kMaxLatAccelMps2);
  return std::max(kMinChangeM,byAcceleration);
}
inline bool trajectoryClear(const State &state,int candidate,
                            double length,const std::vector<Car> &traffic) {
  if(candidate<0 || candidate>2 || std::abs(candidate-state.lane)!=1)
    return false; // no teleport / two-lane jump
  const double lateral0=laneOffset(state.lane,state.spacing);
  const double lateral1=laneOffset(candidate,state.spacing);
  const double travelSpeed=std::max(3.,state.speed);
  const double duration=length/travelSpeed;
  // Evaluate the crossing and at least four seconds into the future.
  const double horizon=std::max(kPreviewS,duration+1.0);
  for(double time=0.;time<=horizon+.001;time+=.20){
    const double progress=smooth(time/duration);
    const double ourLateral=lateral0+(lateral1-lateral0)*progress;
    const double ourS=state.s+travelSpeed*time;
    for(const auto &car:traffic) {
      const double gap=signedGap(ourS,car.s+car.speed*time,state.lap);
      const double diff=std::abs(ourLateral-car.lateral);
      if(diff>=.45)continue;
      // When cars physically overlap, the transition is invalid.
      if(std::abs(gap)<kVehicleOverlapM)return false;
      // Once in the candidate lane, enforce front and rear headways.
      if(time>=duration && gap>-kRearGapM && gap<kFrontGapM)
        return false;
    }
  }
  return true;
}
inline Decision evaluate(const State &state,const std::vector<Car> &cars) {
  Decision decision;
  if(state.changing)return decision; // never reverse mid-maneuver
  const double preferred=std::max(0.,std::min(state.roadSpeed,15.));
  const Car *front=nullptr,*avoided=nullptr;
  double nearest=kDetectM+.001;
  for(const auto &car:cars){
    const double gap=signedGap(state.s,car.s,state.lap);
    if(car.id==state.avoidId)avoided=&car;
    if(std::abs(car.lateral-laneOffset(state.lane,state.spacing))>.42)
      continue;
    if(gap>0. && gap<=kDetectM && gap<nearest){
      front=&car;
      nearest=gap;
    }
  }
  if(front){
    const double followingSpeed=std::sqrt(std::max(0.,
      front->speed*front->speed+2.*4.*std::max(0.,nearest-kFrontGapM)));
    decision.followSpeed=std::min(preferred,followingSpeed);
    decision.blocked=nearest<kTriggerM;
  }
  if(state.sinceChange<kMinChangeM+8. || state.travelled<12.)
    return decision;

  // Rejoin the original grid-based lane only after securely passing the car.
  if(state.lane!=state.homeLane && avoided) {
    const double behind=-signedGap(state.s,avoided->s,state.lap);
    const double width=state.spacing*std::abs(state.lane-state.homeLane);
    const double distance=distanceToChange(state.speed,width);
    if(behind>=10. && behind<state.lap/2. &&
       trajectoryClear(state,state.homeLane,distance,cars)){
      decision.nextLane=state.homeLane;
      decision.changeLength=distance;
      decision.returning=true;
      decision.followSpeed=std::numeric_limits<double>::infinity();
      return decision;
    }
  }
  if(!front)return decision;
  const double closing=preferred-front->speed;
  if(closing<kMinClosingMps)return decision;
  const double ttc=(nearest-5.)/closing;
  if(nearest>kTriggerM || ttc>kTtcS)return decision;

  // At sharp corners, only permit an emergency low-speed bypass of a
  // virtually stationary car. Moving-vehicle passing is deferred.
  if(state.curve>=.080 &&
      (front->speed>1. || state.speed>7.))return decision;
  if(state.curve>=.030 && front->speed>1. &&
     curvatureSpeed(state.curve)-front->speed<kMinClosingMps)
    return decision;

  // Nearest SAFE adjacent lane. From middle lane, tie breaks LEFT.
  for(int next=0;next<3;next++){
    if(std::abs(next-state.lane)!=1)continue;
    const double width=state.spacing;
    const double distance=distanceToChange(state.speed,width);
    if(!trajectoryClear(state,next,distance,cars))continue;
    decision.nextLane=next;
    decision.avoidId=front->id;
    decision.changeLength=distance;
    decision.followSpeed=std::numeric_limits<double>::infinity();
    return decision;
  }
  return decision; // no lane is safe: controlled follow / braking
}
} // namespace urrc::avoid
