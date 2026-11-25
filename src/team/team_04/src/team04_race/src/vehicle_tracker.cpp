#include "team04_race/traffic.hpp"
#include <algorithm>
#include <cmath>
namespace team04 {
void VehicleTracker::reset(){objects.clear();last=-1;next_id=1;overflow=false;}
const std::vector<Track>& VehicleTracker::update(const std::vector<Detection>& detections,double stamp,double command,double steering,double wheelbase,double measured_yaw,double measured_lateral){
 if(last<0 || stamp<=last){objects.clear();last=stamp;}
 const double elapsed=std::max(.001,stamp-last);const double dt=std::min(elapsed,1.0),yaw=std::isfinite(measured_yaw)?measured_yaw:-command*std::tan(steering)/wheelbase*dt,cs=std::cos(yaw),sn=std::sin(yaw);last=stamp;
 const double dx=std::abs(yaw)<1e-8?command*dt:command*dt/yaw*sn+wheelbase*.5*(cs-1);
 const double dy=std::isfinite(measured_lateral)?measured_lateral:std::abs(yaw)<1e-8?0:command*dt/yaw*(1-cs)+wheelbase*.5*sn;
 std::vector<double> oldRanges;oldRanges.reserve(objects.size());
 for(auto&t:objects){oldRanges.push_back(t.range);double x=t.x+t.vx*dt-dx,y=t.y+t.vy*dt-dy;t.x=cs*x+sn*y;t.y=-sn*x+cs*y;double vx=cs*t.vx+sn*t.vy,vy=-sn*t.vx+cs*t.vy;t.vx=vx;t.vy=vy;t.observed=false;t.missed+=elapsed;t.uncertainty=std::min(.7,t.uncertainty+.10*dt);}
 struct Pair{double cost;size_t t,d;};std::vector<Pair> pairs;
 for(size_t i=0;i<objects.size();++i)for(size_t j=0;j<detections.size();++j){auto&t=objects[i];auto&d=detections[j];double distance=std::hypot(t.x-d.x,t.y-d.y);if(distance<.35+2.0*dt+t.uncertainty)pairs.push_back({distance+.2*std::abs(t.width-d.width),i,j});}
 std::sort(pairs.begin(),pairs.end(),[](auto a,auto b){return a.cost<b.cost;});std::vector<bool> assigned(detections.size()),used(objects.size());
 for(auto pair:pairs){if(used[pair.t] || assigned[pair.d])continue;used[pair.t]=true;assigned[pair.d]=true;auto&t=objects[pair.t];auto d=detections[pair.d];double ex=d.x-t.x,ey=d.y-t.y;
  int sign=ey>=0?1:-1;
  if(std::abs(ey)>.01){t.lateral_votes=t.lateral_sign==sign?std::min(10u,t.lateral_votes+1):1;t.lateral_sign=sign;}else if(t.lateral_votes)--t.lateral_votes;
  double betaY=t.lateral_votes>=3?.10:.015;
  double vx=std::clamp(t.vx+.10*ex/dt,-5.,5.),vy=std::clamp(t.vy+betaY*ey/dt,-2.,2.);double closing=.65*t.closing+.35*std::clamp((oldRanges[pair.t]-d.range)/std::max(dt,stamp-t.stamp),-8.,8.);unsigned id=t.id,hits=t.hits+1;double oldWidth=t.width,oldLength=t.length;bool vehicle=t.vehicle_like;
  static_cast<Detection&>(t)=d;t.id=id;t.width=std::max(t.width,oldWidth);t.length=std::max(t.length,oldLength);t.vehicle_like=t.vehicle_like||vehicle;t.x-=.20*ex;t.y-=.20*ey;t.vx=vx;t.vy=vy;t.closing=closing;t.hits=hits;t.observed=true;t.missed=0;t.uncertainty=std::max(t.lateral_votes>=3?.15:.055,.8*t.uncertainty);t.range=std::hypot(t.x,t.y);
 }
 for(size_t j=0;j<detections.size();++j)if(!assigned[j]){
  if(objects.size()>=16){overflow=true;continue;}
  Track t;static_cast<Detection&>(t)=detections[j];t.id=next_id++;t.closing=std::max(0.,command);objects.push_back(t);
 }
 objects.erase(std::remove_if(objects.begin(),objects.end(),[](const Track&t){return t.missed>1.0;}),objects.end());return objects;
}
}
