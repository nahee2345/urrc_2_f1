#include "team04_race/traffic.hpp"
#include <cmath>
#include <algorithm>
namespace team04 {
WallObservation observeWalls(const Plan&plan){
 struct Line{bool valid=false;double slope=0,offset=0;};
 auto fit=[](const std::vector<Point>&p){Line l;double xx=0,x=0,y=0,xy=0,first=1e9,last=-1e9,n=0;
  for(auto q:p)if(q.x>.2 && q.x<3){++n;x+=q.x;y+=q.y;xx+=q.x*q.x;xy+=q.x*q.y;first=std::min(first,q.x);last=std::max(last,q.x);}
  if(n<8 || last-first<1.5 || n*xx-x*x<.01)return l;
  l.slope=(n*xy-x*y)/(n*xx-x*x);l.offset=(y-l.slope*x)/n;double error=0;
  for(auto q:p)if(q.x>.2 && q.x<3)error+=std::pow(q.y-l.slope*q.x-l.offset,2);
  l.valid=std::sqrt(error/n)<.008 && std::abs(l.slope)<.5;return l;
 };
 auto left=fit(plan.left),right=fit(plan.right);WallObservation out;
 if(left.valid && right.valid && std::abs(left.slope-right.slope)<.015){out.valid=true;out.slope=(left.slope+right.slope)/2;out.heading=std::atan(out.slope);out.offset=(left.offset+right.offset)/2;out.left_slope=left.slope;out.right_slope=right.slope;out.left_offset=left.offset;out.right_offset=right.offset;}
 return out;
}
WallObservation refineWallsFromReturns(const Plan&raw,const WallObservation&prior){
 if(!prior.valid)return {};
 Plan current;
 for(auto p:raw.obstacles){
  double l=std::abs(p.y-prior.left_slope*p.x-prior.left_offset)/std::hypot(1.,prior.left_slope);
  double r=std::abs(p.y-prior.right_slope*p.x-prior.right_offset)/std::hypot(1.,prior.right_slope);
  if(l<.12)current.left.push_back(p);
  if(r<.12)current.right.push_back(p);
 }
 // Refit and validate CURRENT measured returns. Prior lines are association
 // hints only; no predicted wall points are ever passed to a path planner.
 return observeWalls(current);
}
}
