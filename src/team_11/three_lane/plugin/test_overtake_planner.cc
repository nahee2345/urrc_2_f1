#include "OvertakePlanner.hh"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using urrc::avoid::Car;
using urrc::avoid::State;
using urrc::avoid::evaluate;
using urrc::avoid::laneOffset;
static State car(int lane=2) {
  State s;
  s.s=100.; s.lap=1000.; s.speed=15.; s.roadSpeed=15.;
  s.lane=lane;s.targetLane=lane;s.homeLane=lane;
  s.spacing=.765;s.curve=.002;s.travelled=30.;s.sinceChange=50.;
  return s;
}
static void close(double actual,double expected) {
  assert(std::abs(actual-expected)<1e-6);
}
int main() {
  close(urrc::avoid::signedGap(990.,15.,1000.),25.);
  close(urrc::avoid::signedGap(15.,990.,1000.),-25.);
  close(urrc::avoid::curvatureSpeed(.02),15.);
  close(urrc::avoid::curvatureSpeed(.03),7.);
  close(urrc::avoid::curvatureSpeed(.055),6.);
  close(urrc::avoid::curvatureSpeed(.12),5.);
  assert(urrc::avoid::distanceToChange(15.,.765)>18.);
  {
    // From lane 3 prefer nearest free lane 2 over also-free lane 1.
    const auto result=evaluate(car(),{{1,130.,laneOffset(2,.765),10.}});
    assert(result.nextLane==1 && result.avoidId==1);
    assert(result.changeLength>=18.);
  }
  {
    // From central lane, equal-cost free targets -> left lane 1.
    const auto result=evaluate(car(1),{{1,130.,0.,10.}});
    assert(result.nextLane==0);
  }
  {
    // Rear car 20 m/s approaching left from 25m behind blocks lane 1.
    const auto result=evaluate(car(1),{
      {1,130.,0.,10.},
      {2,75.,laneOffset(0,.765),20.}});
    assert(result.nextLane==2);
  }
  {
    // No jumping over an occupied intermediate lane.
    const auto result=evaluate(car(2),{
      {1,125.,laneOffset(2,.765),10.},
      {2,105.,0.,0.}});
    assert(result.nextLane==-1);
  }
  {
    // A faster leading car (20m/s) should not be overtaken.
    const auto result=evaluate(car(),{{1,125.,laneOffset(2,.765),20.}});
    assert(result.nextLane==-1);
  }
  {
    // Match slower leader instead of stop, with no safe alternate.
    const auto result=evaluate(car(2),{
      {1,120.,laneOffset(2,.765),10.},
      {2,108.,0.,0.}});
    assert(result.nextLane==-1);
    assert(result.followSpeed<15. && result.followSpeed>10.);
  }
  {
    // Sharp corner: no new moving-car overtake.
    auto state=car();state.curve=.09;state.roadSpeed=5.;state.speed=5.;
    const auto result=evaluate(state,{{1,115.,laneOffset(2,.765),3.}});
    assert(result.nextLane==-1);
  }
  {
    // Sharp corner: stopped obstacle may be passed at <=7m/s.
    auto state=car();state.curve=.09;state.roadSpeed=5.;state.speed=5.;
    const auto result=evaluate(state,{{1,120.,laneOffset(2,.765),0.}});
    assert(result.nextLane==1);
  }
  {
    // Return only once 10m ahead, and the old lane is also safe.
    auto state=car(1);state.homeLane=2;state.avoidId=1;
    const auto result=evaluate(state,{{1,88.,laneOffset(2,.765),10.}});
    assert(result.returning && result.nextLane==2);
    const auto tooSoon=evaluate(state,{{1,93.,laneOffset(2,.765),10.}});
    assert(tooSoon.nextLane==-1);
  }
  {
    // Never reverse an active maneuver. Reduce speed if traffic appears.
    auto state=car(1);state.changing=true;state.targetLane=0;
    const auto result=evaluate(state,{{1,115.,laneOffset(0,.765),10.}});
    assert(result.nextLane==-1);
    assert(result.followSpeed<15.);
  }
  {
    // Initial route merge and recent change inhibit oscillation.
    auto state=car();state.travelled=11.;
    assert(evaluate(state,{{1,125.,laneOffset(2,.765),10.}}).nextLane==-1);
    state.travelled=80.;state.sinceChange=15.;
    assert(evaluate(state,{{1,125.,laneOffset(2,.765),10.}}).nextLane==-1);
  }
  std::cout<<"PASS 3-lane prediction: TTC, gap, speed, tie-left, rejoin, corner, hysteresis\n";
}
