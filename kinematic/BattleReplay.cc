// Three-car Gazebo kinematic scenario: 9m/s S-traffic, team11, 11m/s grid10 pursuer.
// Uses direct pose commands. SAT-shaped collision checks; no camera or wheel dynamics.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <atomic>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
namespace urrc {
constexpr double LEN=.53, WID=.25;
double wrap(double a){return atan2(sin(a),cos(a));}
double ease(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
struct Pt {double s,x,y,z,yaw;};
struct Pos {double x,y,z,yaw;};
struct Car {
 std::string name;gz::sim::Entity entity{gz::sim::kNullEntity};
 double s=0,v=0,lat=0,from=0,to=0,changeStart=0,changeDuration=2.5;
 bool changing=false;
};
class BattleReplay:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 public:
 void Configure(const gz::sim::Entity& world,const std::shared_ptr<const sdf::Element> &cfg,
                gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
  worldEntity=world;
  const std::string topic=cfg->Get<std::string>("start_topic");
  node.Subscribe(topic,&BattleReplay::Start,this);
  ready=node.Advertise<gz::msgs::Boolean>(topic+"/ready");
  std::ifstream src(cfg->Get<std::string>("centerline_file"));
  std::string row;std::getline(src,row);
  while(std::getline(src,row)){
   std::replace(row.begin(),row.end(),',',' ');
   Pt p{};std::istringstream in(row);if(in>>p.s>>p.x>>p.y>>p.z>>p.yaw)route.push_back(p);
  }
  lap=cfg->Get<double>("lap_length_m");
  lateralOffset=cfg->Get<double>("lane_offset_m");
  team.name="team_11";team.s=cfg->Get<double>("team_start_s");
  team.lat=cfg->Get<double>("team_start_offset_m");teamStart=team.s;
  lead.name="practice_s_09";lead.s=cfg->Get<double>("lead_start_s");lead.lat=-lateralOffset;
  rear.name="practice_rear_11";rear.s=cfg->Get<double>("rear_start_s");
  rear.lat=cfg->Get<double>("rear_start_offset_m");
  report.open(cfg->Get<std::string>("log_file"));
  if(route.size()<100||lap<100||lateralOffset<=0||!report){
   std::cerr<<"[BATTLE] INVALID SETUP"<<std::endl;return;
  }
  report<<"time_s,team_s,team_lat,team_speed,lead_s,lead_lat,lead_speed,rear_s,rear_lat,rear_speed,front_ttc,rear_ttc,clearance_m\n";
  ok=true;
  std::cout<<"[BATTLE] READY team grid1 10m/s, S-car 9m/s, rear grid10 11m/s"<<std::endl;
 }
 void PreUpdate(const gz::sim::UpdateInfo& info,gz::sim::EntityComponentManager& ecm) override {
  if(info.paused||!ok||ended)return;
  const double now=std::chrono::duration<double>(info.simTime).count();
  for(Car*a:{&team,&lead,&rear})if(a->entity==gz::sim::kNullEntity)
   ecm.Each<gz::sim::components::Model,gz::sim::components::Name,gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity&id,const gz::sim::components::Model*,
        const gz::sim::components::Name*name,const gz::sim::components::ParentEntity*p)->bool {
      if(name->Data()==a->name&&p->Data()==worldEntity){a->entity=id;return false;}return true;
    });
  if(team.entity==gz::sim::kNullEntity||lead.entity==gz::sim::kNullEntity||rear.entity==gz::sim::kNullEntity)return;
  if(!started){
    if(!startSignal.load())return;
    started=true;begin=now;last=now;lastCsv=now;
    std::cout<<"[BATTLE] START (all five lights off)"<<std::endl;
  }
  double elapsed=now-begin,dt=std::clamp(now-last,0.,.05);last=now;
  if(dt<=0)return;
  const double lateral[5]={-lateralOffset,0,lateralOffset,0,-lateralOffset};
  int period=static_cast<int>(elapsed/2.0);
  int phase=period%4;
  lead.lat=lateral[phase]+(lateral[phase+1]-lateral[phase])*ease((elapsed-2.0*period)/2.0);
  double frontGap=gap(team.s,lead.s),rearGap=gap(rear.s,team.s);
  double frontTTC=team.v>lead.v+.01?frontGap/(team.v-lead.v):1e6;
  double rearTTC=rear.v>team.v+.01?rearGap/(rear.v-team.v):1e6;
  bool corner=fabs(curve(team.s))>.18;
  // Defender: rear TTC below 5s and shrinking; avoid a late lane block.
  if(!team.changing&&!defended&&!corner&&rearGap>1.5&&rearTTC<=5&&rearTTC<previousRearTTC-.002){
   double target=fabs(team.lat)<lateralOffset*.3?lateralOffset:0.;
   if(fabs(target-team.lat)>.2 && safe(target,elapsed)){
     change(target,now);defended=true;
     std::cout<<"[BATTLE] DEFENSE TTC="<<rearTTC<<"s target_offset="<<target<<std::endl;
   }
  }
  // Overtake only when front TTC <=3s, never dive into a swinging practice car.
  if(!team.changing&&!passed&&!corner&&frontGap>1.0&&frontGap<20.&&frontTTC<=3.){
   for(const double dst:{-lateralOffset,0.,lateralOffset}){
    if(fabs(dst-team.lat)<.2*lateralOffset)continue;
    if(safe(dst,elapsed)){
      change(dst,now);attempted=true;
      std::cout<<"[BATTLE] OVERTAKE ATTEMPT TTC="<<frontTTC<<"s offset="<<dst<<std::endl;
      break;
    }
   }
  }
  if(team.changing){
   const double t=(now-team.changeStart)/team.changeDuration;
   team.lat=team.from+(team.to-team.from)*ease(t);
   if(t>=1){team.lat=team.to;team.changing=false;}
  }
  // Practice vehicles slow for any obstacle ahead on the same lane.
  auto cruise=[&](Car &a,double limit){
   double v=limit;
   for(Car*b:{&team,&lead,&rear}){
    if(b==&a)continue;
    double d=gap(a.s,b->s);
    if(d<15.&&fabs(a.lat-b->lat)<WID+.16)
     v=std::min(v,std::max(0.,b->v+1.3*(d-1.3)));
   }
   return std::clamp(v,0.,limit);
  };
  double vt=cruise(team,10),vl=cruise(lead,9),vr=cruise(rear,11);
  if((vl<8.9||vr<10.9)&&elapsed-lastBrake>1.5){
    lastBrake=elapsed;braked=true;
    std::cout<<"[BATTLE] PRACTICE BRAKING lead="<<vl<<" rear="<<vr<<std::endl;
  }
  auto update=[dt](Car &a,double desired){
    a.v+=std::clamp(desired-a.v,-12*dt,6*dt);a.s+=a.v*dt;
  };
  update(team,vt);update(lead,vl);update(rear,vr);
  if(team.s>lead.s+.8&&!passed){
    passed=true;std::cout<<"[BATTLE] OVERTAKE COMPLETED"<<std::endl;
  }
  Pos a=pose(team),b=pose(lead),c=pose(rear);
  for(auto pair:std::array<std::pair<Pos,Pos>,3>{{{a,b},{a,c},{b,c}}}){
    double dist=hypot(pair.first.x-pair.second.x,pair.first.y-pair.second.y);
    minimumCenter=std::min(minimumCenter,dist);
    if(collide(pair.first,pair.second)){
      std::cout<<"[BATTLE] COLLISION at t="<<elapsed<<"s"<<std::endl;
      finish("FAIL_COLLISION");return;
    }
  }
  for(auto pr:std::array<std::pair<Car*,Pos>,3>{{{&team,a},{&lead,b},{&rear,c}}})
   gz::sim::Model(pr.first->entity).SetWorldPoseCmd(
    ecm,gz::math::Pose3d(pr.second.x,pr.second.y,pr.second.z+.09,0,0,pr.second.yaw));
  if(now-lastCsv>=.1){
    report<<std::fixed<<std::setprecision(4)<<elapsed<<','<<team.s<<','<<team.lat<<','<<team.v<<','
          <<lead.s<<','<<lead.lat<<','<<lead.v<<','<<rear.s<<','<<rear.lat<<','<<rear.v<<','
          <<frontTTC<<','<<rearTTC<<','<<minimumCenter<<'\n';
    report.flush();lastCsv=now;
  }
  if(team.s-teamStart>=lap){
    std::cout<<"[BATTLE] SUMMARY overtake="<<passed<<" defense="<<defended
             <<" practice_braking="<<braked<<" minimum_center_m="<<minimumCenter<<std::endl;
    finish(passed&&defended?"PASS_BATTLE":"FAIL_INCOMPLETE");return;
  }
  if(elapsed>300)finish("FAIL_TIMEOUT");
  previousRearTTC=rearTTC;
 }
 private:
 double gap(double from,double to)const {
  double x=fmod(to-from,lap);if(x<0)x+=lap;return x;
 }
 Pos pose(const Car&a)const {
  double t=fmod(a.s,lap);if(t<0)t+=lap;
  auto it=std::upper_bound(route.begin(),route.end(),t,[](double x,const Pt&p){return x<p.s;});
  size_t i=it==route.begin()?0:static_cast<size_t>(it-route.begin()-1);
  size_t j=(i+1)%route.size();const Pt &p=route[i],&q=route[j];
  double end=j==0?lap:q.s;double f=std::clamp((t-p.s)/std::max(1e-6,end-p.s),0.,1.);
  double yaw=p.yaw+wrap(q.yaw-p.yaw)*f;
  return {p.x+(q.x-p.x)*f-a.lat*sin(yaw),
          p.y+(q.y-p.y)*f+a.lat*cos(yaw),
          p.z+(q.z-p.z)*f,yaw};
 }
 double curve(double s)const {
  double t=fmod(s,lap);if(t<0)t+=lap;
  auto it=std::lower_bound(route.begin(),route.end(),t,[](const Pt&p,double x){return p.s<x;});
  size_t i=it==route.end()?0:static_cast<size_t>(it-route.begin());
  const Pt&a=route[(i+route.size()-3)%route.size()],&b=route[(i+3)%route.size()];
  double d=gap(a.s,b.s);return d>.01?wrap(b.yaw-a.yaw)/d:0.;
 }
 bool collide(const Pos&a,const Pos&b)const {
  double dx=b.x-a.x,dy=b.y-a.y;
  for(double yaw:{a.yaw,b.y}){
    for(int k=0;k<2;k++){
      double ux=k==0?cos(yaw):-sin(yaw),uy=k==0?sin(yaw):cos(yaw);
      auto r=[&](const Pos&p){double cs=cos(p.yaw),sn=sin(p.yaw);
        return LEN*.5*fabs(ux*cs+uy*sn)+WID*.5*fabs(-ux*sn+uy*cs);};
      if(fabs(dx*ux+dy*uy)>r(a)+r(b)+.025)return false;
    }
  }
  return true;
 }
 bool safe(double destination,double elapsed)const{
  // Every 0.1s up to 3s: prevent predicted bounding-box overlap.
  const double l[5]={-lateralOffset,0,lateralOffset,0,-lateralOffset};
  for(int i=1;i<=30;i++){
    double future=i*.1;
    Car temp=team;temp.s+=future*std::max(team.v,1.0);
    temp.lat=team.lat+(destination-team.lat)*ease(future/2.5);
    Pos p=pose(temp);
    for(const Car*o:{&lead,&rear}){
      Car sample=*o;sample.s+=future*o->v;
      if(o==&lead){
        int k=static_cast<int>((elapsed+future)/2);
        sample.lat=l[k%4]+(l[(k%4)+1]-l[k%4])*ease(fmod(elapsed+future,2.)/2.);
      }
      if(collide(p,pose(sample)))return false;
    }
  }
  return true;
 }
 void change(double dest,double now){
  team.from=team.lat;team.to=dest;team.changeStart=now;
  team.changeDuration=2.5;team.changing=true;
 }
 void finish(const char*result){
  if(ended)return;ended=true;report.flush();
  std::cout<<"[BATTLE] RESULT "<<result<<" overtake="<<passed<<" defense="<<defended
           <<" braked="<<braked<<" min_center="<<minimumCenter<<std::endl;
 }
 void Start(const gz::msgs::Boolean&b){if(b.data())startSignal.store(true);}
 gz::sim::Entity worldEntity{gz::sim::kNullEntity};
 gz::transport::Node node;gz::transport::Node::Publisher ready;
 std::atomic<bool> startSignal{false};
 std::vector<Pt>route;Car team,lead,rear;
 std::ofstream report;double lap=0,lateralOffset=0,teamStart=0,begin=0,last=0,lastCsv=0,lastBrake=-10;
 double previousRearTTC=1e6,minimumCenter=1e9;
 bool ok=false,started=false,ended=false,passed=false,defended=false,attempted=false,braked=false;
};
}
GZ_ADD_PLUGIN(urrc::BattleReplay,gz::sim::System,urrc::BattleReplay::ISystemConfigure,urrc::BattleReplay::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::BattleReplay,"urrc::BattleReplay")
