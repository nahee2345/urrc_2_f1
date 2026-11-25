#include "team04_race/traffic.hpp"
#include <algorithm>
#include <cmath>
namespace team04 {
bool rearEnvelopeClear(const Track&t,double margin){
 return t.observed && t.hits>=3 && t.x+.5*t.length+.64+t.uncertainty<-.26-margin;
}
double collisionTtc(const Track&t,double command,double margin){
 const double gap=t.x-.5*t.length-.64-.26-margin-t.uncertainty;
 // New or missed objects use commanded ego motion as an additional closing bound.
 double closing=std::max(0.,t.closing+.15+(t.observed && t.hits>=3?0:std::abs(command)));
 return closing>.05?std::max(0.,gap)/closing:std::numeric_limits<double>::infinity();
}
bool predictedCollision(const std::vector<Point>&path,const std::vector<double>&speeds,const std::vector<Track>&tracks,double command,double max_speed,double margin){
 if(path.size()<2 || speeds.size()!=path.size())return true;
 double time=0,s=0;
 for(size_t i=0;i+1<path.size();++i){
  if(i){double ds=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);s+=ds;double predicted=std::min(max_speed,std::sqrt(command*command+1.8*s));time+=ds/std::max(.25,std::min(predicted,std::max(.25,speeds[i])));}
  double heading=std::atan2(path[i+1].y-path[i].y,path[i+1].x-path[i].x),cs=std::cos(heading),sn=std::sin(heading);
  double hx=.260+margin+.105,hy=.119479+margin+.025;
  for(const auto&t:tracks){
   double unc=t.uncertainty+.025*time+.25*t.missed;
   double ox=.5*t.length+.64+unc,oy=.5*t.width+ .64+unc;
   double dx=t.x+t.vx*time-path[i].x,dy=t.y+t.vy*time-path[i].y;
   // SAT: full rotated ego rectangle vs inflated predicted object AABB.
   bool overlap=std::abs(dx)<=ox+hx*std::abs(cs)+hy*std::abs(sn) && std::abs(dy)<=oy+hx*std::abs(sn)+hy*std::abs(cs) && std::abs(dx*cs+dy*sn)<=hx+ox*std::abs(cs)+oy*std::abs(sn) && std::abs(-dx*sn+dy*cs)<=hy+ox*std::abs(sn)+oy*std::abs(cs);
   if(overlap)return true;
  }
 }
 return false;
}
}
