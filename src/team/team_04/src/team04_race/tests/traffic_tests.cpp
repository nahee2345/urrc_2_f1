#include "team04_race/traffic.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace team04;
unsigned count=0;
void check(bool ok,const char*n){if(!ok)throw std::runtime_error(n);++count;std::cout<<"PASS "<<n<<'\n';}
std::vector<Point> scene(double width=2.4){std::vector<Point> p;for(double x=-2.6;x<2.7;x+=.1)for(double y=-1;y<1.1;y+=.1)if(std::hypot(x,y)>.35)p.push_back({x,y,-.13});for(double x=-3;x<12;x+=.025)for(double z:{-.09,-.05,-.01,.03,.10}){p.push_back({x,width/2,z});p.push_back({x,-width/2,z});}return p;}
void car(std::vector<Point>&p,double x,double y){for(double a=-.10;a<=.101;a+=.015)for(double z:{-.09,-.05,-.01})p.push_back({x,y+a,z});}
Detection detection(double x,double y,double stamp){Detection d;d.x=x;d.y=y;d.min_x=x;d.max_x=x;d.min_y=y-.1;d.max_y=y+.1;d.width=.2;d.range=std::hypot(x,y);d.stamp=stamp;d.inside=true;d.vehicle_like=true;d.points=20;return d;}
int main(){
 PlannerConfig c;c.path_mode="racing_line";c.planning_horizon=10;c.braking=2.5;c.acceleration=.9;c.lateral_acceleration=1;c.max_speed=1.2;c.visibility_buffer=1.8;
 auto empty=scene();auto raw=planLidar(empty,c,.8);check(raw.valid,"baseline corridor valid");check(detectVehicles(raw,1).empty(),"continuous walls are not vehicles");
 auto p=empty;car(p,5,0);auto observed=planLidar(p,c,.8);auto ds=detectVehicles(observed,1);check(ds.size()==1 && std::abs(ds[0].x-5)<.05 && ds[0].points>=3,"vehicle rear detection and relative range");
 check(ds[0].vehicle_like && ds[0].inside,"vehicle geometry inside actual corridor");
 auto ground=empty;for(int i=0;i<20;++i)ground.push_back({4.+i*.01,0,-.13});check(detectVehicles(planLidar(ground,c,.8),1).empty(),"ground points do not become vehicles");
 auto single=empty;single.push_back({4,0,-.03});check(detectVehicles(planLidar(single,c,.8),1).empty(),"single return never confirms a vehicle");
 auto housing=empty;for(double y:{-.02,0.,.02})housing.push_back({3,y,.03});auto hd=detectVehicles(planLidar(housing,c,.8),1);check(hd.size()==1 && !hd[0].vehicle_like,"sparse housing cluster remains conservative unknown occupancy");
 auto multiple=empty;car(multiple,4,-.5);car(multiple,6,.5);check(detectVehicles(planLidar(multiple,c,.8),1).size()==2,"two separate interior vehicles");
 VehicleTracker tracker;unsigned id=0;std::vector<Track> tracks;
 for(int i=0;i<30;++i){double t=1+i*.05;tracks=tracker.update({detection(5-.7*i*.05,0,t)},t,1,0,.32);if(i==0)id=tracks[0].id;check(tracks[0].id==id,"association retains straight target ID");}
 check(std::abs(tracks[0].closing-.7)<.15,"range-rate closing estimate converges");check(std::abs(tracks[0].vx-.3)<.15,"command-compensated object velocity converges");
 tracks=tracker.update({},2.5,1,0,.32);check(tracks.size()==1 && !tracks[0].observed && tracks[0].missed>0,"occlusion preserves predicted track");
 tracks=tracker.update({},4,0,0,.32);check(tracks.empty(),"old track ID expires after actual elapsed time");
 Track t;static_cast<Detection&>(t)=detection(5,0,1);t.closing=1;t.hits=4;t.observed=true;check(std::isfinite(collisionTtc(t,1,.16)) && collisionTtc(t,1,.16)>2,"TTC uses inflated body gap");t.closing=-1;check(std::isinf(collisionTtc(t,0,.16)),"nonapproaching TTC is infinite");t.x=.5;t.closing=1;check(collisionTtc(t,1,.16)==0,"overlapping occupancy TTC zero");
 std::vector<Point> path;for(double x=0;x<8;x+=.2)path.push_back({x,0,0});std::vector<double> speed(path.size(),1.2);t.x=4;t.vx=0;check(predictedCollision(path,speed,{t},1,1.2,.16),"dynamic full-body collision rejects blocked path");for(auto&q:path)q.y=1.4;check(!predictedCollision(path,speed,{t},1,1.2,.16),"wide lateral passing path clears inflated occupancy");
 TrafficPlanner engine;auto solo=engine.update(empty,c,.8,0,1,false);check(solo.valid && solo.speed==raw.speed && solo.steering==raw.steering && solo.path.size()==raw.path.size(),"empty traffic is exact stage4 decision");
 for(int i=0;i<5;++i)engine.update(p,c,.8,0,2+i*.05,false);
 check(engine.debug().state=="FOLLOWING" || engine.debug().state=="EMERGENCY_BRAKING","solo mode keeps obstacle safety without overtaking");
 auto passScene=empty;car(passScene,5,-.7);engine.reset();Plan left;for(int i=0;i<30;++i)left=engine.update(passScene,c,.8,0,3+i*.05,true);if(!left.valid || !engine.debug().attempts)std::cerr<<"pass diagnostic "<<engine.debug().state<<" / "<<engine.debug().reason<<" / "<<left.reason<<" / detections "<<engine.debug().detections.size()<<" / hits "<<(engine.debug().tracks.empty()?0:engine.debug().tracks[0].hits)<<" / curvature "<<left.curvature<<"\n";check(left.valid && engine.debug().attempts>0 && engine.debug().side==1,"clear straight supports a certified passing candidate");
 int side=engine.debug().side;engine.update(passScene,c,.8,0,4.6,true);check(engine.debug().side==side,"passing direction hysteresis");
 auto rightScene=empty;car(rightScene,5,.7);engine.reset();Plan rightPass;for(int i=0;i<30;++i)rightPass=engine.update(rightScene,c,.8,0,5+i*.05,true);check(rightPass.valid && engine.debug().side==-1,"right-space scenario selects right pass");
 auto narrow=scene(1.2);car(narrow,4,0);engine.reset();for(int i=0;i<5;++i)engine.update(narrow,c,.8,0,4+i*.05,true);check(engine.debug().attempts==0,"narrow corridor never initiates a pass");
 auto walls=observeWalls(raw);check(walls.valid && std::abs(walls.heading)<.01,"static wall motion features available on straight");
 auto turned=raw;for(auto*line:{&turned.left,&turned.right})for(auto&q:*line){double x=q.x-.04,y=q.y-.02;q.x=std::cos(.04)*x+std::sin(.04)*y;q.y=-std::sin(.04)*x+std::cos(.04)*y;}
 auto wt=observeWalls(turned);check(wt.valid && std::abs(wt.heading+.04)<.002,"LiDAR walls measure ego yaw without IMU");
 for(auto*line:{&turned.left,&turned.right}){for(auto&q:*line)q.y+=.1*q.x*q.x;}
 check(!observeWalls(turned).valid,"curved walls cannot masquerade as straight-motion features");
 VehicleTracker many;bool stable=true;unsigned first=0,second=0;for(int i=0;i<15;++i){auto one=detection(4-.02*i,-.5,10+i*.05),two=detection(6-.01*i,.5,10+i*.05);auto tracks=many.update(i%2?std::vector<Detection>{two,one}:std::vector<Detection>{one,two},10+i*.05,.8,0,.32);if(i==0){first=tracks[0].id;second=tracks[1].id;}stable=stable && tracks.size()==2 && tracks[0].id==first && tracks[1].id==second;}check(stable,"multi-object IDs survive changing detection order");
 Track rear;rear.observed=true;rear.hits=5;rear.x=-.7;check(!rearEnvelopeClear(rear,.16),"center crossing is insufficient for merge clearance");rear.x=-1.6;check(rearEnvelopeClear(rear,.16),"entire observed front envelope behind ego rear allows merge");rear.observed=false;check(!rearEnvelopeClear(rear,.16),"lost rear target never authorizes return");
 auto recovered=refineWallsFromReturns(raw,walls);check(recovered.valid,"current raw wall returns recover classification geometry");
 Plan blind;check(!refineWallsFromReturns(blind,walls).valid,"prior walls never synthesize missing current returns");
 VehicleTracker crowded;std::vector<Detection> crowd;for(int i=0;i<17;++i)crowd.push_back(detection(2+i*.8,0,20));
 auto capped=crowded.update(crowd,20,0,0,.32);check(capped.size()==16 && crowded.capacityExceeded(),"track capacity overflow is reported instead of silently dropping occupancy");
 crowded.reset();check(!crowded.capacityExceeded(),"explicit tracker reset clears latched capacity fault");
 auto fragments=empty;car(fragments,4,0);car(fragments,4.4,0);auto joined=detectVehicles(planLidar(fragments,c,.8),1);
 check(joined.size()==1 && joined[0].length>.39,"split rear and front returns retain their full union as one body");
 auto abreast=empty;car(abreast,5,-.4);car(abreast,5,.4);
 check(detectVehicles(planLidar(abreast,c,.8),1).size()==2,"side-by-side vehicles remain separate tracks");
 Plan finite;finite.ground_support=100;finite.ground_c=-.13;
 finite.left={{0,1,0},{1,1,0},{2,1,0}};finite.right={{0,-1,0},{1,-1,0},{2,-1,0}};
 for(double y:{-.05,0.,.05})finite.obstacles.push_back({2.6,y,-.05});
 check(detectVehicles(finite,1).empty(),"objects beyond actual paired sections are not classified inside an extended corridor");
 auto sideGap=raw;
 for(auto*line:{&sideGap.left,&sideGap.right})line->erase(std::remove_if(line->begin(),line->end(),[](const Point&p){return p.x<.5;}),line->end());
 for(double y:{-.75,-.70,-.65})sideGap.obstacles.push_back({0,y,-.05});
 check(detectVehicles(sideGap,1).size()==1,"current straight wall support preserves objects in the front-rear side-sector gap");
 std::cout<<"Traffic checks passed: "<<count<<'\n';
}
