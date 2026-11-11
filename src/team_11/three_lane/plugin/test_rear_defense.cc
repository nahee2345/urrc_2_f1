#include "RearDefense.hh"
#include "RearScan.hh"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using urrc::avoid::Car;
using urrc::avoid::laneOffset;
using urrc::defense::RearTarget;
static urrc::defense::State ego(int lane=2,double speed=15.){
  urrc::defense::State s;
  s.car.s=150.;s.car.lap=1100.;s.car.speed=speed;
  s.car.roadSpeed=15.;s.car.spacing=.75;s.car.curve=.001;
  s.car.travelled=100.;s.car.sinceChange=80.;
  s.car.lane=lane;s.car.targetLane=lane;s.car.homeLane=lane;
  s.sinceDefenseChangeM=80.;s.zigzagEndpoint=0;
  return s;
}
int main(){
  using urrc::defense::evaluate;
  using urrc::rear_scan::verifiedReturn;
  {
    auto s=ego();
    // 20m/s rear vehicle in lane2, 24m behind our lane3.
    std::vector<Car> traffic{{9,126.,0.,20.}};
    std::vector<RearTarget> rear{{9,24.,20.,0.,true,false}};
    auto d=evaluate(s,rear,traffic);
    assert(d.active && d.nextLane==1 && !d.zigzag);
    assert(d.ttc>4.7 && d.ttc<4.9);
  }
  {
    auto s=ego();
    std::vector<RearTarget> rear{{9,24.,15.,0.,true,false}};
    assert(!evaluate(s,rear,{}).active); // no relative approach
    rear[0].speed=20.;rear[0].behindM=30.;
    assert(!evaluate(s,rear,{}).active); // TTC > 5
    rear[0].behindM=24.;rear[0].seenByRearLidar=false;
    assert(!evaluate(s,rear,{}).active); // no actual scan, no defense
  }
  {
    auto s=ego();
    s.frontOvertakeRequested=true;
    assert(evaluate(s,{{9,24.,20.,0.,true,false}},{}).nextLane==-1);
    s.frontOvertakeRequested=false;s.frontOvertakeActive=true;
    assert(evaluate(s,{{9,24.,20.,0.,true,false}},{}).nextLane==-1);
    s.frontOvertakeActive=false;s.car.changing=true;
    assert(evaluate(s,{{9,24.,20.,0.,true,false}},{}).nextLane==-1);
  }
  {
    auto s=ego();
    s.car.curve=.05;
    assert(evaluate(s,{{9,24.,20.,0.,true,false}},{}).nextLane==-1);
    s.car.curve=0.;s.sinceDefenseChangeM=2.;
    assert(evaluate(s,{{9,24.,20.,0.,true,false}},{}).nextLane==-1);
  }
  {
    auto s=ego(2);
    std::vector<Car> traffic{{9,126.,0.,20.}};
    // Unknown rear path: begin from lane3 toward lane1, via lane2.
    auto first=evaluate(s,{{9,24.,20.,.35,true,true}},traffic);
    assert(first.nextLane==1 && first.zigzag);
    s.car.lane=1;s.car.targetLane=1;s.car.s=180.;
    s.sinceDefenseChangeM=35.;
    traffic[0].s=156.;
    auto second=evaluate(s,{{9,24.,20.,.35,true,true}},traffic);
    assert(second.nextLane==0 && second.zigzag);
    // Once on lane1 and endpoint flips, path returns via lane2 to lane3.
    s.car.lane=0;s.car.targetLane=0;s.zigzagEndpoint=2;
    auto third=evaluate(s,{{9,24.,20.,.35,true,true}},traffic);
    assert(third.nextLane==1);
  }
  {
    auto s=ego(1);
    // Imminent faster car already on our lane: no separate route to block.
    // User requested a stepped S path in this specific case.
    auto d=evaluate(s,{{9,24.,20.,0.,true,false}},{{9,126.,0.,20.}});
    assert(d.active && d.zigzag && d.nextLane==0);
  }
  {
    auto s=ego();
    // Sudden blocking with rear too close to complete safe shift is denied.
    auto d=evaluate(s,{{9,7.,20.,0.,true,false}},{{9,143.,0.,20.}});
    assert(d.active && d.nextLane==-1);
    assert(std::string(d.reason)=="UNSAFE_SHIFT");
  }
  {
    urrc::rear_scan::Scan scan;
    scan.ranges.assign(721,INFINITY);
    scan.angleMin=-2.25;scan.angleStep=4.5/720.;
    scan.rangeMin=.12;scan.rangeMax=70.;scan.valid=true;
    assert(!verifiedReturn(scan,-25.,-.75));
    const double expected=std::hypot(24.8,.75);
    const double bearing=std::atan2(.75,24.8);
    const size_t idx=static_cast<size_t>(std::lround(
       (bearing-scan.angleMin)/scan.angleStep));
    assert(idx<scan.ranges.size());
    scan.ranges[idx]=static_cast<float>(expected-.28);
    assert(verifiedReturn(scan,-25.,-.75));
    assert(!verifiedReturn(scan,20.,0.));
    assert(!verifiedReturn(scan,-25.,3.)); // wrong angular cluster
  }
  std::cout<<"PASS rear LiDAR range, TTC threat, front priority, closest lane,"
           <<" alternating S path, no S on predictable target, unsafe late shift\n";
}
