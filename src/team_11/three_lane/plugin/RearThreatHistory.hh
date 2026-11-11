#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace urrc::defense {
// State is updated ONLY when an actual NEW rear LiDAR scan was received.
// Fast Gazebo physics ticks must not count the same scan repeatedly.
struct ThreatHistory {
  int id{-1};
  int consecutiveImminent{0};
  int fallingCount{0};
  int weaveCount{0};
  double lastTtc{std::numeric_limits<double>::infinity()};
  double lastSeenTime{-1.};
  double lastLateral{0.};
  double lastLateralTime{-1.};

  void reset() {*this=ThreatHistory{};}
  bool update(int targetId,double time,double ttc,double lateral,
              bool rawWeave) {
    if(id!=targetId || (lastSeenTime>=0. && time-lastSeenTime>0.75))
      reset();
    id=targetId;
    if(time<=lastSeenTime+0.001)return false;
    if(ttc<=5.0 && ttc>0.)++consecutiveImminent;
    else consecutiveImminent=0;
    if(ttc<=5.0 && std::isfinite(lastTtc) &&
       ttc < lastTtc-0.03)
      ++fallingCount;
    else fallingCount=0;
    // Uncertainty requires repeated evidence; one off-route return
    // can never start an S-maneuver by itself.
    bool fastWeave=false;
    if(lastLateralTime>=0. && time-lastLateralTime>=.08 &&
       time-lastLateralTime<=.8){
      const double rate=std::abs(lateral-lastLateral) /
                        (time-lastLateralTime);
      fastWeave=rate>.55;
    }
    if(rawWeave || fastWeave)weaveCount=std::min(20,weaveCount+1);
    else weaveCount=std::max(0,weaveCount-1);
    lastTtc=ttc;
    lastSeenTime=time;
    if(lastLateralTime<0. || time-lastLateralTime>=.08){
      lastLateralTime=time;
      lastLateral=lateral;
    }
    return consecutiveImminent>=3 ||
           (consecutiveImminent>=2 && fallingCount>=1);
  }
  bool weaving()const {return weaveCount>=3;}
};
} // namespace urrc::defense
