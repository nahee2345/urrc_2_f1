#include "team04_race/traffic.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <queue>
namespace team04 {
namespace {
long long key(int x,int y){return (static_cast<long long>(x+4096)<<20)|(y+4096);}
double segmentDistance(const Point&p,const std::vector<Point>&line){double best=1e9;for(size_t i=1;i<line.size();++i){auto a=line[i-1],b=line[i];double x=b.x-a.x,y=b.y-a.y,t=std::clamp(((p.x-a.x)*x+(p.y-a.y)*y)/std::max(1e-9,x*x+y*y),0.,1.);best=std::min(best,std::hypot(p.x-a.x-t*x,p.y-a.y-t*y));}return best;}
bool inTriangle(const Point&p,const Point&a,const Point&b,const Point&c){
 auto cross=[](const Point&u,const Point&v,const Point&w){return (v.x-u.x)*(w.y-u.y)-(v.y-u.y)*(w.x-u.x);};
 if(std::abs(cross(a,b,c))<1e-8)return false;
 double x=cross(a,b,p),y=cross(b,c,p),z=cross(c,a,p);
 return (x>=0 && y>=0 && z>=0)||(x<=0 && y<=0 && z<=0);
}
bool measuredInterior(const Point&p,const Plan&base){
 size_t count=std::min(base.left.size(),base.right.size());
 for(size_t i=1;i<count;++i){auto a=base.left[i-1],b=base.left[i],c=base.right[i],d=base.right[i-1];
  if(inTriangle(p,a,b,c)||inTriangle(p,a,c,d))return true;
 }
 return false;
}
}
std::vector<Detection> detectVehicles(const Plan& base,double stamp){
 std::vector<Detection> result;if(base.ground_support<40 || base.left.size()<3 || base.right.size()<3)return result;
 const auto wallFit=observeWalls(base);
 // A 12cm grid bounds neighbor searches; Z is already relative to the measured ground.
 constexpr double cell=.12;std::unordered_map<long long,std::vector<size_t>> grid;
 std::vector<Point> points;points.reserve(base.obstacles.size());
 for(auto p:base.obstacles){if(std::abs(p.x)<.29 && std::abs(p.y)<.15)continue;if(std::abs(p.x)>11.9 || std::abs(p.y)>5)continue;points.push_back(p);grid[key(int(std::floor(p.x/cell)),int(std::floor(p.y/cell)))].push_back(points.size()-1);}
 std::vector<bool> visited(points.size());
 for(size_t seed=0;seed<points.size();++seed){if(visited[seed])continue;std::vector<size_t> cluster{seed};visited[seed]=true;
  for(size_t j=0;j<cluster.size();++j){auto p=points[cluster[j]];int x=std::floor(p.x/cell),y=std::floor(p.y/cell);
   for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy){auto it=grid.find(key(x+dx,y+dy));if(it==grid.end())continue;for(auto i:it->second)if(!visited[i] && std::hypot(p.x-points[i].x,p.y-points[i].y)<=.17){visited[i]=true;cluster.push_back(i);}}
  }
  if(cluster.size()<3)continue;
  Detection d;d.min_x=d.min_y=1e9;d.max_x=d.max_y=-1e9;double zmin=1e9,zmax=-1e9,wall=1e9;
  for(auto i:cluster){auto p=points[i];d.min_x=std::min(d.min_x,p.x);d.max_x=std::max(d.max_x,p.x);d.min_y=std::min(d.min_y,p.y);d.max_y=std::max(d.max_y,p.y);double h=p.z-base.ground_a*p.x-base.ground_b*p.y-base.ground_c;zmin=std::min(zmin,h);zmax=std::max(zmax,h);}
  std::vector<double> ys;for(auto i:cluster)ys.push_back(points[i].y);std::nth_element(ys.begin(),ys.begin()+ys.size()/2,ys.end());
  d.x=d.min_x;d.y=ys[ys.size()/2];d.length=d.max_x-d.min_x;d.width=d.max_y-d.min_y;
  // Long continuous facades and wall-touching components remain static obstacles.
  if(std::max(d.length,d.width)>1.25 || zmax<.035 || std::max(d.length,d.width)<.02)continue;
  for(auto i:cluster){wall=std::min(wall,std::min(segmentDistance(points[i],base.left),segmentDistance(points[i],base.right)));if(wall<.10)break;}
  if(wall<.10)continue;
  // Ambiguous collinear returns stay STATIC. This only classifies actual
  // points: it never creates a wall point or certifies an unobserved path.
  if(wallFit.valid){
   unsigned staticSupport=0;
   for(auto i:cluster){const auto&q=points[i];double l=std::abs(q.y-wallFit.left_slope*q.x-wallFit.left_offset)/std::hypot(1.,wallFit.left_slope),r=std::abs(q.y-wallFit.right_slope*q.x-wallFit.right_offset)/std::hypot(1.,wallFit.right_slope);staticSupport+=std::min(l,r)<.065;}
   if(staticSupport*2>=cluster.size())continue;
  }
  // Nearest actual paired cross-section, with bounded longitudinal support only.
  double distance=1e9;size_t nearest=0;
  for(size_t i=0;i<base.left.size() && i<base.right.size();++i){Point m{(base.left[i].x+base.right[i].x)/2,(base.left[i].y+base.right[i].y)/2,0};double v=std::hypot(m.x-d.x,m.y-d.y);if(v<distance){distance=v;nearest=i;}}
  auto l=base.left[nearest],r=base.right[nearest];double nx=l.x-r.x,ny=l.y-r.y,w=std::hypot(nx,ny);if(w<.5)continue;nx/=w;ny/=w;
  double across=(d.x-r.x)*nx+(d.y-r.y)*ny;
  // Classification also requires containment in actual consecutive measured
  // cross-sections. A nearest section's normal alone falsely labels distant
  // curved-wall endpoints as interior by extending that section longitudinally.
  // Front/rear paired traces begin a little ahead of their own origin. At the
  // side sector only, CURRENT well-supported straight walls classify the gap
  // between those traces; no such extension is used to certify a path.
  bool measuredSide=wallFit.valid && std::abs(d.x)<.8 && d.y>wallFit.right_slope*d.x+wallFit.right_offset+.12 && d.y<wallFit.left_slope*d.x+wallFit.left_offset-.12;
  d.inside=across>.12 && across<w-.12 && distance<std::hypot(w/2,d.x<0?1.8:.8) && (measuredInterior({d.x,d.y,0},base)||measuredSide);
  if(!d.inside)continue;
  d.points=cluster.size();d.range=std::hypot(d.x,d.y);d.bearing=std::atan2(d.y,d.x);d.stamp=stamp;d.id=result.size()+1;
  d.vehicle_like=std::max(d.width,d.length)>=.08 && std::max(d.width,d.length)<=.65;
  // A partially visible rear face can remain well supported while the total
  // bbox length grows as another body panel appears. Fit actual face returns
  // instead of switching orientation confidence at an arbitrary bbox aspect.
  const size_t samples=std::min<size_t>(24,cluster.size());
  for(size_t i=0;i<samples && !d.heading_supported;++i)for(size_t j=i+1;j<samples;++j){
   auto a=points[cluster[i*cluster.size()/samples]],b=points[cluster[j*cluster.size()/samples]];
   double dx=b.x-a.x,dy=b.y-a.y,span=std::hypot(dx,dy);
   bool rear=span>=.06 && std::abs(dx)<.342*span;
   bool side=span>=.35 && std::abs(dy)<.342*span;
   if(!rear && !side)continue;
   unsigned support=0;double residual=0;
   for(size_t k=0;k<samples;++k){auto q=points[cluster[k*cluster.size()/samples]];double error=std::abs((q.x-a.x)*dy-(q.y-a.y)*dx)/span;if(error<.012){++support;residual=std::max(residual,error);}}
   if(support>=3){d.heading_supported=true;double angle=std::asin(std::min(1.,(rear?std::abs(dx):std::abs(dy))/span))+std::atan2(.01+residual,span);d.lateral_padding=std::min(.61,.25+.55*std::sin(std::min(M_PI/2,angle)));break;}
  }
  d.confidence=std::min(.95,.35+.025*std::min(24u,d.points));
  result.push_back(d);if(result.size()==17)break;
 }
 // Sparse beams can split the rear body and front housing of one physical
 // car. Coalesce only longitudinal fragments with overlapping lateral support
 // whose entire union fits one legal-sized forward body. Keep the UNION bbox;
 // never discard occupied extent or merge side-by-side vehicle envelopes.
 if(result.size()>16)return result; // capacity overflow remains fail closed
 for(size_t i=0;i<result.size();++i)for(size_t j=i+1;j<result.size();){
  auto&a=result[i];const auto&b=result[j];
  double loX=std::min(a.min_x,b.min_x),hiX=std::max(a.max_x,b.max_x),loY=std::min(a.min_y,b.min_y),hiY=std::max(a.max_y,b.max_y);
  double gapX=std::max({0.,a.min_x-b.max_x,b.min_x-a.max_x});
  bool sharedY=std::min(a.max_y,b.max_y)>=std::max(a.min_y,b.min_y);
  if(sharedY && gapX<=.45 && hiX-loX<=.60 && hiY-loY<=.30){
   a.y=(a.y*a.points+b.y*b.points)/(a.points+b.points);a.points+=b.points;
   a.min_x=loX;a.max_x=hiX;a.min_y=loY;a.max_y=hiY;a.x=loX;a.length=hiX-loX;a.width=hiY-loY;
   a.range=std::hypot(a.x,a.y);a.bearing=std::atan2(a.y,a.x);a.vehicle_like=a.vehicle_like||b.vehicle_like;a.confidence=std::min(.95,.35+.025*std::min(24u,a.points));
   result.erase(result.begin()+j);j=i+1;
  }else ++j;
 }
 for(size_t i=0;i<result.size();++i)result[i].id=i+1;
 return result;
}
}
