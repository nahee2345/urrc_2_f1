#include "team04_race/lidar_planner.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
namespace team04 {
namespace {
constexpr double halfLength=.260,halfWidth=.119479;
double dist(const Point&a,const Point&b){return std::hypot(a.x-b.x,a.y-b.y);}
Point nearest(const Point&p,const std::vector<Point>&line){
 Point best=line.front();double score=1e100;
 for(size_t i=1;i<line.size();++i){auto a=line[i-1],b=line[i];double x=b.x-a.x,y=b.y-a.y;double t=std::clamp(((p.x-a.x)*x+(p.y-a.y)*y)/std::max(1e-10,x*x+y*y),0.,1.);Point q{a.x+t*x,a.y+t*y,0};double d=dist(p,q);if(d<score){score=d;best=q;}}
 return best;
}
std::vector<double> arcs(const std::vector<Point>&p){std::vector<double>s(p.size());for(size_t i=1;i<p.size();++i)s[i]=s[i-1]+dist(p[i-1],p[i]);return s;}
std::vector<double> curvatures(const std::vector<Point>&p){
 std::vector<double>k(p.size());if(p.size()<4)return k;
 for(size_t i=1;i+1<p.size();++i){size_t a=i>2?i-3:0,b=std::min(i+3,p.size()-1);auto x=p[a],y=p[i],z=p[b];k[i]=2*((y.x-x.x)*(z.y-y.y)-(y.y-x.y)*(z.x-y.x))/std::max(1e-9,dist(x,y)*dist(y,z)*dist(x,z));}
 k.front()=k[1];k.back()=k[k.size()-2];return k;
}
bool segmentBox(Point a,Point b,const Point&p,double cs,double sn,double hx,double hy){
 auto local=[&](Point q){double x=q.x-p.x,y=q.y-p.y;return Point{x*cs+y*sn,-x*sn+y*cs,0};};a=local(a);b=local(b);
 double low=0,high=1;for(int axis=0;axis<2;++axis){double q=axis?a.y:a.x,d=axis?b.y-a.y:b.x-a.x,h=axis?hy:hx;if(std::abs(d)<1e-10){if(std::abs(q)>h)return false;}else{double x=(-h-q)/d,y=(h-q)/d;if(x>y)std::swap(x,y);low=std::max(low,x);high=std::min(high,y);if(low>high)return false;}}return true;
}
// Swept sampled rectangles include the complete steering envelope and retain
// .16m clearance. Extra half-sample inflation covers gaps between path poses.
bool footprint(const std::vector<Point>&path,const Plan&base,const PlannerConfig&cfg,bool obstacles,double&clearance){
 if(path.size()<6)return false;
 auto s=arcs(path);clearance=1e100;
 const double hx=halfLength+cfg.safety_margin+.105,hy=halfWidth+cfg.safety_margin+.025;
 for(size_t i=0;i+1<path.size() && s[i]<=s.back()-.55;++i){
  size_t a=i?i-1:i,b=std::min(i+1,path.size()-1);double heading=std::atan2(path[b].y-path[a].y,path[b].x-path[a].x),cs=std::cos(heading),sn=std::sin(heading);
  auto l=nearest(path[i],base.left),r=nearest(path[i],base.right);double nx=l.x-r.x,ny=l.y-r.y,width=std::hypot(nx,ny);if(width<1e-6)return false;nx/=width;ny/=width;
  // Oriented rectangle support in the measured cross-section, including length.
  double support=hx*std::abs(nx*cs+ny*sn)+hy*std::abs(-nx*sn+ny*cs);
  double dl=(l.x-path[i].x)*nx+(l.y-path[i].y)*ny,dr=(path[i].x-r.x)*nx+(path[i].y-r.y)*ny;
  if(std::min(dl,dr)<support)return false;
  double bodySupport=halfLength*std::abs(nx*cs+ny*sn)+halfWidth*std::abs(-nx*sn+ny*cs);
  clearance=std::min(clearance,std::min(dl,dr)-bodySupport);
  for(const auto*wall:{&base.left,&base.right})for(size_t j=1;j<wall->size();++j)if(segmentBox((*wall)[j-1],(*wall)[j],path[i],cs,sn,hx,hy))return false;
  if(obstacles)for(const auto&o:base.obstacles){double x=o.x-path[i].x,y=o.y-path[i].y;if(std::abs(x)>hx+hy || std::abs(y)>hx+hy)continue;if(std::abs(x*cs+y*sn)<hx && std::abs(-x*sn+y*cs)<hy)return false;}
 }
 return true;
}
struct Profile {std::vector<double>v,k;double buffer=0,time=1e100,speed=0,visibility=0,curve=0,steer=0;std::string limit;};
Profile profile(const std::vector<Point>&p,const PlannerConfig&cfg,double command){
 Profile out;auto s=arcs(p);out.k=curvatures(p);out.v.assign(p.size(),cfg.max_speed);if(p.size()<6 || s.back()<1)return out;
 // Full body remains before the last actual paired boundary observation.
 const double usable=std::max(0.,s.back()-.55),reaction=cfg.reaction_time;
 const double bt=cfg.braking*reaction;
 // Measured high-speed range contraction reserve scales with braking energy.
 // Low-speed corners retain the physical .45m reserve and hard current-speed check.
 out.buffer=cfg.visibility_buffer*std::clamp(command*command/(cfg.visibility_reference_speed*cfg.visibility_reference_speed),0.,1.);
 out.visibility=std::max(0.,std::sqrt(bt*bt+2*cfg.braking*std::max(0.,usable-.45-out.buffer))-bt);
 out.curve=cfg.max_speed;out.steer=cfg.max_speed;
 for(size_t i=0;i<p.size();++i){double k=std::abs(out.k[i]);double v=std::sqrt(cfg.lateral_acceleration/std::max(k,1e-5));
  size_t a=i?i-1:i,b=std::min(i+1,p.size()-1);double delta=std::abs(std::atan(cfg.wheelbase*out.k[b])-std::atan(cfg.wheelbase*out.k[a]))/std::max(.01,s[b]-s[a]);double steer=cfg.steering_rate/std::max(delta,1e-5);
  out.v[i]=std::min({cfg.max_speed,v,steer});if(s[i]>=usable)out.v[i]=0;
 }
 for(size_t i=p.size()-1;i>0;--i)out.v[i-1]=std::min(out.v[i-1],std::sqrt(out.v[i]*out.v[i]+2*cfg.braking*(s[i]-s[i-1])));
 double desired=std::min(cfg.max_speed,out.visibility);out.limit=out.visibility<cfg.max_speed?"observed_braking_distance":"speed_cap";
 for(size_t i=0;i<p.size();++i){double cap=std::sqrt(out.v[i]*out.v[i]+2*cfg.braking*std::max(0.,s[i]-.4-command*reaction-out.buffer));if(cap<desired){desired=cap;out.limit="future_curvature_or_steering";}}
 out.speed=desired;std::vector<double> predicted=out.v;predicted[0]=std::min(predicted[0],std::max(.15,command));
 for(size_t i=1;i<p.size();++i)predicted[i]=std::min(predicted[i],std::sqrt(predicted[i-1]*predicted[i-1]+2*cfg.acceleration*(s[i]-s[i-1])));
 out.time=0;for(size_t i=1;i<p.size() && s[i]<=usable;++i)out.time+=2*(s[i]-s[i-1])/std::max(.1,predicted[i]+predicted[i-1]);return out;
}
double lookahead(const std::vector<Point>&path,const std::vector<double>&curvature,const PlannerConfig&cfg,double command){
 auto s=arcs(path);double nominal=std::clamp(cfg.lookahead_min+cfg.lookahead_gain*std::abs(command),cfg.lookahead_min,cfg.lookahead_max),nearCurve=0;
 for(size_t i=0;i<curvature.size();++i)if(s[i]<=2.5)nearCurve=std::max(nearCurve,std::abs(curvature[i]));
 double curve=std::clamp(nominal/(1+.8*nearCurve),cfg.lookahead_min,cfg.lookahead_max);double observed=std::max(cfg.lookahead_min,.8*std::max(0.,s.back()-.55));return std::min({nominal,curve,observed});
}
double bump(double t,double a,double b){if(t<=a || t>=b)return 0;double x=(t-a)/(b-a);return std::sin(M_PI*x)*std::sin(M_PI*x);}
double smooth(double x){x=std::clamp(x,0.,1.);return x*x*(3-2*x);}
}
Plan planLidar(const std::vector<Point>&input,const PlannerConfig&cfg,double command){
 PlannerConfig perception=cfg;
 if(cfg.path_mode=="racing_line"){perception.lookahead_gain=0;perception.lookahead_max=perception.lookahead_min;}
 Plan out=cfg.path_mode=="racing_line"?planObservedCorridorLidar(input,perception,command):planCenterlineLidar(input,cfg,command);out.center_path=out.path;out.path_mode=cfg.path_mode;
 if(!out.valid || cfg.path_mode=="centerline"){
  out.visibility_speed_limit=std::sqrt(2*cfg.braking*std::max(0.,out.free_distance-.45));out.limiting_factor=out.visibility_speed_limit<cfg.max_speed?"observed_braking_distance":"legacy_curvature_or_cap";return out;
 }
 if(cfg.additional_obstacles)out.obstacles.insert(out.obstacles.end(),cfg.additional_obstacles->begin(),cfg.additional_obstacles->end());
 auto center=out.path;auto originalArc=arcs(center);
 if(cfg.stop_before_arc<originalArc.back()){
  size_t n=std::upper_bound(originalArc.begin(),originalArc.end(),cfg.stop_before_arc)-originalArc.begin();
  if(n<6){out.valid=false;out.speed=0;out.reason="TRAFFIC_STOPPING_CORRIDOR_TOO_SHORT";return out;}
  center.resize(n);
 }
 auto s=arcs(center);std::vector<Point> normals(center.size());std::vector<double> widths(center.size());
 for(size_t i=0;i<center.size();++i){size_t a=i?i-1:i,b=std::min(i+1,center.size()-1);double dx=center[b].x-center[a].x,dy=center[b].y-center[a].y,length=std::hypot(dx,dy);normals[i]={-dy/std::max(length,1e-9),dx/std::max(length,1e-9),0};auto l=nearest(center[i],out.left),r=nearest(center[i],out.right);widths[i]=std::max(0.,std::min(dist(center[i],l),dist(center[i],r))-halfWidth-cfg.safety_margin-.15);}
 auto bestProfile=profile(center,cfg,command);out.center_time=bestProfile.time;double bestCost=1e100,bestClearance=0;bool any=false;
 std::vector<Point> bestPath;unsigned evaluated=0;double bestOffset=0;
 const double continuation=-(center.front().x*normals.front().x+center.front().y*normals.front().y);
 // The three independent offsets permit outside/inside/outside and shifted
 // apexes; time/curvature/connectivity determine the choice, never map labels.
 const double amplitude=cfg.traffic_side?1.15:.75;
 for(double a:{0.,-amplitude,amplitude})for(double b:{0.,-amplitude,amplitude})for(double c:{0.,-amplitude,amplitude}){
  if(cfg.traffic_side && c*cfg.traffic_side<=0){++evaluated;continue;}
  std::vector<Point> path=center;double offsetMax=0;
  for(size_t i=0;i<path.size();++i){double t=s[i]/std::max(.1,s.back()),b1=bump(t,0,.6),b2=bump(t,.2,.8),b3=smooth((t-.4)/.6);double normalized=(a*b1+b*b2+c*b3)/std::max(1.,b1+b2+b3);double offset=widths[i]*normalized*smooth((s[i]-.7)/1.2);double anchored=offset+continuation*(1-smooth(s[i]/std::min(4.,.75*s.back())));
   if(cfg.traffic_side){
    const double transition=std::max(1.5,4.+a/amplitude-cfg.traffic_travel);
    const double goal=cfg.traffic_side*(widths[i]+.10)*(.85+.10*b/amplitude);
    anchored=continuation+(goal-continuation)*smooth(s[i]/transition);offset=anchored;
   }
   path[i].x+=normals[i].x*anchored;path[i].y+=normals[i].y*anchored;offsetMax=std::max(offsetMax,std::abs(offset));}
  ++evaluated;auto plan=profile(path,cfg,command);if(a==0 && b==0 && c==0)out.center_time=plan.time;double maxK=0;for(double k:plan.k)maxK=std::max(maxK,std::abs(k));if(maxK>1.4 || !std::isfinite(plan.time))continue;
  double clearance=0;if(!footprint(path,out,cfg,false,clearance)){++out.rejected_boundary;continue;}
  double regularization=0;for(size_t i=1;i<plan.k.size();++i)regularization+=std::pow(std::atan(cfg.wheelbase*plan.k[i])-std::atan(cfg.wheelbase*plan.k[i-1]),2);
  double look=lookahead(path,plan.k,cfg,command);Point target=path.back();auto candidateArc=arcs(path);for(size_t i=0;i<path.size();++i)if(candidateArc[i]>=look){target=path[i];break;}double pursuit=2*target.y/(std::pow(target.x+cfg.wheelbase/2,2)+target.y*target.y);double steering=-std::atan(cfg.wheelbase*pursuit);
  double cost=plan.time+.2*regularization+.015*offsetMax*offsetMax+1.5*std::pow(steering-cfg.previous_steering,2);
  if(out.candidates.size()<8){out.candidates.push_back(path);out.candidate_times.push_back(plan.time);}
  if(cost<bestCost){
   if(!footprint(path,out,cfg,true,clearance)){++out.rejected_raw_obstacle;continue;}
   if(cfg.dynamic_safe && !cfg.dynamic_safe(path,plan.v)){++out.rejected_dynamic;continue;}
   any=true;bestCost=cost;bestPath=std::move(path);bestProfile=std::move(plan);bestClearance=clearance;bestOffset=offsetMax;
  }
 }
 out.candidate_count=evaluated;
 if(!any){out.valid=false;out.speed=0;out.reason="FULL_FOOTPRINT_BLOCKED";out.fallback_reason="no certified candidate: boundary="+std::to_string(out.rejected_boundary)+" raw="+std::to_string(out.rejected_raw_obstacle)+" dynamic="+std::to_string(out.rejected_dynamic);return out;}
 out.path=std::move(bestPath);out.speed_profile=bestProfile.v;out.curvature_profile=bestProfile.k;out.predicted_time=bestProfile.time;out.min_clearance=bestClearance;out.applied_visibility_buffer=bestProfile.buffer;out.racing_selected=bestOffset>.025;out.max_offset=bestOffset;
 if(!out.racing_selected)out.fallback_reason="certified center path has lowest cost";
 const double look=lookahead(out.path,bestProfile.k,cfg,command);out.effective_lookahead=look;auto chosenArc=arcs(out.path);bool target=false;
 const double usable=chosenArc.back()-.55;const double required=command*command/(2*cfg.braking)+cfg.reaction_time*std::abs(command)+.45;
 if(required>usable){out.valid=false;out.speed=0;out.reason="INSUFFICIENT_STOPPING_DISTANCE";return out;}
 for(size_t i=0;i<out.path.size();++i)if(chosenArc[i]>=look){out.target=out.path[i];target=true;break;}
 if(!target || chosenArc.back()-.55<look+.05){out.valid=false;out.speed=0;out.reason="INSUFFICIENT_LOOKAHEAD";return out;}
 double tx=out.target.x+cfg.wheelbase/2,ty=out.target.y;if(tx<=.2){out.valid=false;out.speed=0;out.reason="TARGET_NOT_FORWARD";return out;}
 double pursuit=2*ty/(tx*tx+ty*ty);double maxVirtual=std::atan(cfg.wheelbase/(cfg.wheelbase/std::tan(35*M_PI/180)+.184/2));out.steering=-std::clamp(std::atan(cfg.wheelbase*pursuit),-maxVirtual,maxVirtual);
 out.curvature=0;for(double k:bestProfile.k)out.curvature=std::max(out.curvature,std::abs(k));out.curvature_speed_limit=std::sqrt(cfg.lateral_acceleration/std::max(std::abs(pursuit),1e-5));out.visibility_speed_limit=bestProfile.visibility;out.limiting_factor=bestProfile.limit;
 out.speed=std::min(bestProfile.speed,out.curvature_speed_limit);if(out.curvature_speed_limit<bestProfile.speed)out.limiting_factor="pure_pursuit_curvature";
 out.free_distance=std::max(0.,chosenArc.back()-.55);out.lateral_error=out.path.front().y;out.heading_error=std::atan2(out.path[2].y-out.path[0].y,out.path[2].x-out.path[0].x);
 if(!std::isfinite(out.speed)||out.speed<=0){out.valid=false;out.speed=0;out.reason="INVALID_COMMAND";}return out;
}
}
