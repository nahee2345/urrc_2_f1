#include "team04_race/lidar_planner.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <cstdint>
namespace team04 {
namespace {
bool solve(double a[3][4],std::array<double,3>& out){
 for(int i=0;i<3;++i){int pivot=i;for(int j=i+1;j<3;++j)if(std::abs(a[j][i])>std::abs(a[pivot][i]))pivot=j;
  if(std::abs(a[pivot][i])<1e-9)return false;
  for(int k=i;k<4;++k)std::swap(a[i][k],a[pivot][k]);
  double scale=a[i][i];for(int k=i;k<4;++k)a[i][k]/=scale;
  for(int j=0;j<3;++j)if(j!=i){double s=a[j][i];for(int k=i;k<4;++k)a[j][k]-=s*a[i][k];}
 }
 for(int i=0;i<3;++i)out[i]=a[i][3];
 return true;
}
bool fit(const std::vector<Point>& p,bool ground,std::array<double,3>& result){
 double a[3][4]{};
 for(const auto& v:p){const std::array<double,3> f=ground?std::array<double,3>{{v.x,v.y,1}}:std::array<double,3>{{v.x*v.x,v.x,1}};double z=ground?v.z:v.y;
  for(int i=0;i<3;++i){for(int j=0;j<3;++j)a[i][j]+=f[i]*f[j];a[i][3]+=f[i]*z;}}
 return p.size()>=3 && solve(a,result);
}
double distance(const Point&a,const Point&b){return std::hypot(a.x-b.x,a.y-b.y);}
}
Plan planObservedCorridorLidar(const std::vector<Point>& input,const PlannerConfig& cfg,double commanded_speed){
 Plan out;std::vector<Point> valid,ground;
 for(auto p:input){
  const double r=std::hypot(p.x,p.y);
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||r<.05||r>12.001)continue;
  ++out.finite_points;
  // Conservative envelope of own geometry, including every steering position.
  if(std::abs(p.x)<.28 && std::abs(p.y)<.14 && p.z<.05)continue;
  valid.push_back(p);
  if(r>.35 && r<2.8 && p.z>-.24 && p.z<-.075)ground.push_back(p);
 }
 if(ground.size()<40){out.reason="GROUND_UNOBSERVED";return out;}
 // Sensor-only ground plane: deterministic RANSAC, then least squares inliers.
 // Reject near-vertical facade planes and implausible mounting heights.
 std::array<double,3> plane{{0,0,-.13}};size_t best=0;uint32_t rng=1741;
 std::vector<Point> sample(3); // Reuse storage; preserve the exact RANSAC sample sequence.
 for(int iteration=0;iteration<64;++iteration){
  for(int j=0;j<3;++j){rng=1664525*rng+1013904223;sample[j]=ground[rng%ground.size()];}
  std::array<double,3> f;if(!fit(sample,true,f)||std::hypot(f[0],f[1])>.18||f[2]<-.20||f[2]>-.085)continue;
  size_t count=0;for(const auto&p:ground)if(std::abs(p.z-f[0]*p.x-f[1]*p.y-f[2])<.009)++count;
  if(count>best){plane=f;best=count;}
 }
 if(best<40){out.reason="GROUND_FIT_FAILED";return out;}
 std::vector<Point> inliers;for(const auto&p:ground)if(std::abs(p.z-plane[0]*p.x-plane[1]*p.y-plane[2])<.012)inliers.push_back(p);
 if(!fit(inliers,true,plane)){out.reason="GROUND_FIT_FAILED";return out;}
 out.ground_support=inliers.size();out.ground_a=plane[0];out.ground_b=plane[1];out.ground_c=plane[2];
 // Height relative to measured road plane, not a fixed world Z or a map.
 for(const auto&p:valid){double h=p.z-(plane[0]*p.x+plane[1]*p.y+plane[2]);
  if(h>=cfg.min_obstacle_height && h<=cfg.max_obstacle_height)out.obstacles.push_back(p);}
 // Keep wall identity through bends using polar continuity, not the sign of Y.
 constexpr int bins=720;std::array<Point,bins> polar{};std::array<double,bins> range;range.fill(std::numeric_limits<double>::infinity());
 for(const auto&p:out.obstacles){int i=std::clamp(int(std::lround((std::atan2(p.y,p.x)+M_PI)*719/(2*M_PI))),0,719);double r=std::hypot(p.x,p.y);if(r<range[i]){range[i]=r;polar[i]=p;}}
 auto trace=[&](int seed,int direction){
  std::vector<Point> line;int start=-1;
  for(int delta=0;delta<35 && start<0;++delta)for(int sign:{-1,1}){int i=(seed+sign*delta+bins)%bins;if(std::isfinite(range[i])){start=i;break;}}
  if(start<0)return line;
  line.push_back(polar[start]);int index=start,skipped=0;
  for(int step=1;step<380;++step){int i=(start+direction*step+bins)%bins;
   if(!std::isfinite(range[i])){if(++skipped>6)break;continue;}
   double gap=distance(polar[i],polar[index]);
   if(gap>.25+.06*std::min(range[i],range[index])){
    // Sparse grazing-angle returns may be far apart along a wall even when
    // adjacent observed rays agree. Join only actual endpoints with a bounded
    // gap and a strongly consistent measured tangent/ray intersection.
    bool consistent=false;
    if(line.size()>=5 && gap<=1.0){
     const auto &anchor=line[line.size()-5],&last=line.back();double tx=last.x-anchor.x,ty=last.y-anchor.y,norm=std::hypot(tx,ty);
     if(norm>.03){tx/=norm;ty/=norm;double dx=polar[i].x-last.x,dy=polar[i].y-last.y,r=range[i],ux=polar[i].x/r,uy=polar[i].y/r;
      double denominator=ux*ty-uy*tx;double predicted=std::abs(denominator)>1e-5?(last.x*ty-last.y*tx)/denominator:-1;
      double across=std::abs(dx*ty-dy*tx),along=dx*tx+dy*ty;
      consistent=along>0 && along/gap>.98 && across<.04+.01*r && predicted>0 && std::abs(predicted-r)*std::abs(denominator)<.04+.01*r;
     }
    }
    if(!consistent)break;
    ++out.observed_sparse_links;
   }
   line.push_back(polar[i]);index=i;skipped=0;
  }
  return line;
 };
 auto left=trace(539,-1),right=trace(180,1);
 if(left.size()<4 || right.size()<4){out.reason="INSUFFICIENT_TWO_SIDED_BOUNDARIES";return out;}
 const double roadWidth=distance(left.front(),right.front());const double margin=cfg.vehicle_width/2+cfg.safety_margin;
 if(roadWidth<2*margin){out.reason="CORRIDOR_TOO_NARROW";return out;}
 auto nearest=[](const Point&p,const std::vector<Point>&line){
  Point best=line.front();double d=std::numeric_limits<double>::infinity();
  for(size_t i=1;i<line.size();++i){const auto&a=line[i-1];const auto&b=line[i];double vx=b.x-a.x,vy=b.y-a.y;double t=std::clamp(((p.x-a.x)*vx+(p.y-a.y)*vy)/std::max(1e-10,vx*vx+vy*vy),0.0,1.0);Point q{a.x+t*vx,a.y+t*vy,0};double ds=std::hypot(p.x-q.x,p.y-q.y);if(ds<d){best=q;d=ds;}}
  return best;
 };
 struct Corridor {std::vector<Point> mid,l,r;double length=0;};
 auto medial=[&](const std::vector<Point>&a,const std::vector<Point>&b,bool fromLeft){
  Corridor c;
  for(const auto&p:a){Point q=nearest(p,b);double w=distance(p,q);Point m{(p.x+q.x)/2,(p.y+q.y)/2,0};
   if(w<2*margin || w<roadWidth*.65 || w>roadWidth*1.30){if(c.mid.size()>2)break;continue;}
   if(c.mid.empty() && m.x<-.25)continue;
   if(!c.mid.empty()){
    const auto&last=c.mid.back();double step=distance(last,m);if(step<.07)continue;if(step>.75)break;
    if(c.mid.size()>1){const auto&prev=c.mid[c.mid.size()-2];if((m.x-last.x)*(last.x-prev.x)+(m.y-last.y)*(last.y-prev.y)<-.001)break;}
    c.length+=step;
   }
   c.mid.push_back(m);c.l.push_back(fromLeft?p:q);c.r.push_back(fromLeft?q:p);
   if(c.length>cfg.planning_horizon)break;
  }
  return c;
 };
 auto lc=medial(left,right,true),rc=medial(right,left,false);auto corridor=lc.length>=rc.length?lc:rc;
 out.left=corridor.l;out.right=corridor.r;out.width=roadWidth;
 if(corridor.mid.size()<3 || corridor.length<1.05){out.reason="INSUFFICIENT_TWO_SIDED_BOUNDARIES";return out;}
 // Resample measured medial pairs by arc length. This also works at a 90-degree bend.
 std::vector<Point> sampled;double next=0,arc=0;
 for(size_t i=1;i<corridor.mid.size();++i){auto a=corridor.mid[i-1],b=corridor.mid[i];double ds=distance(a,b);if(ds<1e-6)continue;
  while(next<=arc+ds){double t=std::clamp((next-arc)/ds,0.0,1.0);sampled.push_back({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),0});next+=.2;}
  arc+=ds;
 }
 if(sampled.size()<6){out.reason="INSUFFICIENT_LOOKAHEAD";return out;}
 // Short arc-domain smoothing; never extrapolate beyond actual paired observations.
 for(size_t i=0;i<sampled.size();++i){Point p{};double sum=0;
  for(int j=std::max(0,int(i)-2);j<=std::min(int(sampled.size())-1,int(i)+2);++j){double w=3-std::abs(int(i)-j);p.x+=sampled[j].x*w;p.y+=sampled[j].y*w;sum+=w;}
  p.x/=sum;p.y/=sum;out.path.push_back(p);
 }
 out.lateral_error=out.path.front().y;
 out.heading_error=std::atan2(out.path[2].y-out.path[0].y,out.path[2].x-out.path[0].x);
 const double lookahead=std::clamp(cfg.lookahead_min+cfg.lookahead_gain*std::abs(commanded_speed),cfg.lookahead_min,cfg.lookahead_max);
 std::vector<double> curvature(out.path.size(),0),arcs(out.path.size(),0);double travel=0;
 for(size_t i=1;i<out.path.size();++i){travel+=distance(out.path[i-1],out.path[i]);arcs[i]=travel;}
 for(size_t i=0;i<out.path.size();++i){size_t a=i>2?i-3:0,b=std::min(i+3,out.path.size()-1);if(a==i||b==i)continue;
  auto p=out.path[a],q=out.path[i],r=out.path[b];double ab=distance(p,q),bc=distance(q,r),ac=distance(p,r);curvature[i]=2*std::abs((q.x-p.x)*(r.y-p.y)-(q.y-p.y)*(r.x-p.x))/std::max(1e-8,ab*bc*ac);out.curvature=std::max(out.curvature,curvature[i]);}
 bool targetSet=false;double desired=cfg.max_speed;
 for(size_t i=0;i<out.path.size();++i){const auto&p=out.path[i];
  const double k=curvature[i];double curveSpeed=std::min(cfg.max_speed/(1+1.5*k),std::sqrt(cfg.lateral_acceleration/std::max(k,1e-4)));
  desired=std::min(desired,std::sqrt(curveSpeed*curveSpeed+2*cfg.braking*std::max(0.0,arcs[i]-.4)));
  if(!targetSet && arcs[i]>=lookahead){out.target=p;targetSet=true;}
  bool blocked=false;if(!cfg.boundary_only)for(const auto&ob:out.obstacles)if(distance(ob,p)<margin){blocked=true;break;}
  if(blocked){out.path.resize(i);travel=i?arcs[i-1]:0;break;}
 }
 out.free_distance=travel;
 if(!targetSet || travel<lookahead+.05){out.reason="INSUFFICIENT_LOOKAHEAD";return out;}
 const double tx=out.target.x+cfg.wheelbase/2,ty=out.target.y;
 if(tx<=.2){out.reason="TARGET_NOT_FORWARD";return out;}
 const double pursuit=2*ty/(tx*tx+ty*ty);
 const double maxVirtual=std::atan(cfg.wheelbase/(cfg.wheelbase/std::tan(35*M_PI/180)+.184/2));
 out.steering=-std::clamp(std::atan(cfg.wheelbase*pursuit),-maxVirtual,maxVirtual);
 desired=std::min(desired,cfg.max_speed/(1+1.5*std::abs(pursuit)));
 desired=std::min(desired,std::sqrt(cfg.lateral_acceleration/std::max(std::abs(pursuit),1e-4)));
 desired=std::min(desired,std::sqrt(2*cfg.braking*std::max(0.0,travel-.45)));
 if(!std::isfinite(desired)||!std::isfinite(out.steering)||desired<=0){out.reason="INVALID_COMMAND";return out;}
 out.speed=desired;out.valid=true;out.reason="TRACKING";return out;
}
}
