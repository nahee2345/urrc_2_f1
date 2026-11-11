#include "CornerPlanner.hh"
#include "BoundedTracker.hh"
#include "RearDefense.hh"
#include "OvertakePlanner.hh"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

static urrc::avoid::State road(int lane=2){
  urrc::avoid::State s;
  s.s=200.;s.lap=1100.;s.speed=15.;s.roadSpeed=7.;
  s.spacing=.765;s.curve=.05;s.travelled=70.;
  s.sinceChange=55.;s.lane=lane;s.targetLane=lane;
  s.homeLane=lane;s.changing=false;
  return s;
}
int main(){
  using namespace urrc;
  corner::Preview bend;
  bend.maxCurvature=.06;
  bend.firstBendDistance=31.;
  bend.bendAhead=true;
  for(int lane:{0,2}){
    auto s=road(lane);
    const auto move=corner::evaluate(s,bend,{},false,false,false);
    assert(move.approach && move.nextLane==1);
    assert(move.speedCap==7.);
    assert(move.length>=18.);
    assert(std::abs(lane-move.nextLane)==1);
    // The next route becomes 2 and no automatic return is produced.
    s.lane=1;s.targetLane=1;s.homeLane=1;s.avoidId=-1;
    assert(corner::evaluate(s,bend,{},false,false,false).nextLane==-1);
  }
  {
    auto s=road(2);
    bend.maxCurvature=.10;
    assert(corner::evaluate(s,bend,{},false,false,false).speedCap==5.);
    assert(corner::evaluate(s,bend,{},true,false,false).nextLane==-1);
    assert(corner::evaluate(s,bend,{},false,true,false).nextLane==-1);
    assert(corner::evaluate(s,bend,{},false,false,true).nextLane==-1);
    s.changing=true;
    assert(corner::evaluate(s,bend,{},false,false,false).nextLane==-1);
    s.changing=false;
    // Any adjacent front or rear collision path blocks the merge.
    std::vector<avoid::Car> traffic{{99,s.s+7.,0.,4.}};
    const auto blocked=corner::evaluate(s,bend,traffic,false,false,false);
    assert(blocked.nextLane==-1 && blocked.unsafe);
    assert(blocked.speedCap==5.);
  }
  {
    auto s=road(0);
    bend.bendAhead=false;
    assert(!corner::evaluate(s,bend,{},false,false,false).approach);
  }
  {
    const double wheelbase=.33;
    const double curvature=bounded::curvatureLimit(wheelbase);
    assert(std::abs(curvature-std::tan(bounded::kPi/6.)/.33)<1e-10);
    bounded::Pose pose{};
    // Turn as hard as possible at LOW speed. This must clamp at
    // precisely +/-30 degrees, preserving radius >= .572m.
    const auto left=bounded::follow(pose,0.,.8,.5,.1,wheelbase);
    assert(left.steer<=bounded::kSteer30+1e-12);
    assert(left.steer>=bounded::kSteer30-1e-12);
    assert(std::abs(left.curvature)<=curvature+1e-12);
    const auto right=bounded::follow(pose,0.,-.8,.5,.1,wheelbase);
    assert(right.steer>=-bounded::kSteer30-1e-12);
    assert(right.steer<=-bounded::kSteer30+1e-12);
    const auto faster=bounded::follow(pose,0.,2.,15.,.02,wheelbase);
    assert(std::abs(faster.yawRate)<=bounded::kMaxYawRate+1e-10);
    assert(std::abs(bounded::wrap(faster.pose.yaw-pose.yaw)) <=
           bounded::kMaxYawRate*.02+1e-10);
    assert(std::hypot(faster.pose.x-pose.x,faster.pose.y-pose.y)
           <=15.*.02+1e-9);
    // Sustained sharp turn cannot violate a hard yaw rate.
    for(int i=0;i<500;i++){
      const auto next=bounded::follow(pose,pose.x-4.,pose.y+4.,
                                      12.,.02,wheelbase);
      assert(std::abs(next.steer)<=bounded::kSteer30+1e-9);
      assert(std::abs(next.curvature)<=curvature+1e-9);
      assert(std::abs(next.yawRate)<=bounded::kMaxYawRate+1e-9);
      pose=next.pose;
    }
    assert(bounded::speedForCurvature(.08)<=
           std::sqrt(2.5/.08)+1e-10);
    assert(bounded::speedForCurvature(.3)<=
           bounded::kMaxYawRate/.3+1e-10);
    assert(std::isinf(bounded::speedForCurvature(0.)));
  }
  std::cout<<"PASS outer routes 1/3 -> center route 2 on corner, front priority,"
             " collision-safe merge, +/-30 steer, curvature and yaw limits\n";
}
