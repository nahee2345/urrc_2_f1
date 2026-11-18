// TEAM08 direct-pose replay. Based on the geometric sampling, yaw smoothing,
// 5th-order lane changes and speed limits of legacy/BattleReplay.cc.
// Unlike the legacy three-car scripted scenario, this plugin controls TEAM08
// only. Actual team_01/team_02 are observed, never repositioned.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/float.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sdf/Element.hh>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace urrc {
constexpr double kPi=3.14159265358979323846;
constexpr double kHalfLength=.265,kHalfWidth=.125;
double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
double clamp(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
double ease(double t){t=clamp(t,0.,1.);return t*t*t*(10.+t*(-15.+6.*t));}
struct Pt{double s,x,y,z,yaw,width;};
struct Pose{double x=0,y=0,z=0,yaw=0;};
struct Other {std::string id;bool found=false;Pose pose;double speed=0;double last=0;Pose before;};
class Team08Pose_08 final:public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate {
 public:
 void Configure(const gz::sim::Entity& entity,
                const std::shared_ptr<const sdf::Element>&,
                gz::sim::EntityComponentManager&,
                gz::sim::EventManager&) override {
   model=entity;
   try {
     const auto share=ament_index_cpp::get_package_share_directory("team08_race");
     for(const std::string &track:std::array<std::string,6>{"monza","spa","monaco","cota","hungaroring","baku"}){
       std::ifstream in(share+"/maps/"+track+"/centerline.csv");
       std::string line;std::getline(in,line);
       std::vector<Pt> points;
       while(std::getline(in,line)){
         std::replace(line.begin(),line.end(),',',' ');
         std::istringstream ss(line);Pt p{};
         if(ss>>p.s>>p.x>>p.y>>p.z>>p.yaw>>p.width)points.push_back(p);
       }
       if(points.size()>=200)maps.push_back({track,std::move(points)});
     }
   }catch(const std::exception&e){std::cerr<<"[TEAM08 POSE] map load: "<<e.what()<<std::endl;}
   node.Subscribe("/team08/replay_go_08",&Team08Pose_08::onStart,this);
   speedPub=node.Advertise<gz::msgs::Float>("/team08/speed_08");
   steerPub=node.Advertise<gz::msgs::Float>("/team08/steering_08");
   std::cout<<"[TEAM08 POSE] loaded "<<maps.size()<<" maps; awaits gate/camera"<<std::endl;
 }
 void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm) override {
   if(info.paused)return;
   const double now=std::chrono::duration<double>(info.simTime).count();
   if(!initialized){
     const auto current=gz::sim::worldPose(model,ecm);
     if(!std::isfinite(current.Pos().X()))return;
     const double x=current.Pos().X(),y=current.Pos().Y();
     const double yaw=current.Rot().Yaw();
     double best=1e30;int which=-1;double nearestS=0,lat=0;
     // Project the actual official spawn on each centerline segment. Never
     // assume grid 01 coordinates, and do not teleport before lights-out.
     for(size_t m=0;m<maps.size();++m){
       const auto&v=maps[m].second;
       for(size_t i=0;i+1<v.size();++i){
         const auto&a=v[i],&b=v[i+1];
         const double dx=b.x-a.x,dy=b.y-a.y;
         const double t=clamp(((x-a.x)*dx+(y-a.y)*dy)/std::max(1e-9,dx*dx+dy*dy),0.,1.);
         const double px=a.x+t*dx,py=a.y+t*dy;
         const double d=std::hypot(px-x,py-y),head=std::abs(wrap(yaw-a.yaw));
         const double score=d+(head>.8?100.:0.);
         if(score<best){
           best=score;which=static_cast<int>(m);
           nearestS=a.s+t*(b.s-a.s);
           lat=(x-px)*(-std::sin(a.yaw))+(y-py)*std::cos(a.yaw);
         }
       }
     }
     if(which<0||best>1.5){return;}
     mapIndex=which;s=nearestS;offset=lat;goalOffset=offset;
     const auto&v=maps[mapIndex].second;
     lap=v.back().s+std::hypot(v.front().x-v.back().x,v.front().y-v.back().y);
     lastPose={x,y,current.Pos().Z(),yaw};
     spawnPose=current;
     initialized=true;lastTime=now;
     std::cout<<"[TEAM08 POSE] track="<<maps[mapIndex].first<<" start_s="<<s
              <<" lateral="<<offset<<" map_offset_error="<<best<<std::endl;
   }
   if(!go.load()){
     // The original battle replay holds every car at the spawn pose before
     // lights-out. Keep TEAM08 anchored even when Gazebo physics is active.
     gz::sim::Model(model).SetWorldPoseCmd(ecm,spawnPose);
     lastPose={spawnPose.Pos().X(),spawnPose.Pos().Y(),
               spawnPose.Pos().Z(),spawnPose.Rot().Yaw()};
     lastTime=now;speed=0;publish(0,0);return;
   }
   double dt=clamp(now-lastTime,0.,.20);lastTime=now;
   if(dt<=0){publish(speed,0);return;}
   if(!started){started=true;std::cout<<"[TEAM08 POSE] START direct coordinates"<<std::endl;}
   // Match the previously verified BattleReplay08 speed envelope:
   // straight up to 15 m/s, mild corners 10 m/s, sharp corners 9 m/s.
   // Keep the existing +3/-5 m/s^2 acceleration/braking ramps.
   const double curveNow=std::abs(curve(s));
   double cap=15.;
   const double brakeLook=std::max(16.,speed*speed/10.+8.);
   bool sharpAhead=false;
   for(double d=0;d<=brakeLook;d+=.5){
     const double k=std::abs(curve(s+d));
     if(k>=.18){
       sharpAhead=true;
       cap=std::min(cap,std::sqrt(81.+10.*std::max(0.,d-1.5)));
     }else if(k>=.08)
       cap=std::min(cap,std::sqrt(100.+10.*std::max(0.,d-1.)));
   }
   // Real opponents: read-only world poses; no pre-scripted S-car/rear car.
   auto other=readOther(ecm,now);
   double forwardGap=1e9,closing=0;
   bool blocked=false;
   for(const auto&o:other){
     if(!o.found)continue;
     const double dx=o.pose.x-lastPose.x,dy=o.pose.y-lastPose.y;
     const double along=dx*std::cos(lastPose.yaw)+dy*std::sin(lastPose.yaw);
     const double side=std::abs(-dx*std::sin(lastPose.yaw)+dy*std::cos(lastPose.yaw));
     if(along>0 && along<18. && side<.50){
       forwardGap=std::min(forwardGap,along-.53);
       closing=std::max(closing,speed-o.speed);
       blocked=true;
     }
   }
   if(blocked){
     if(forwardGap<1.0)cap=0.;
     else if(forwardGap<3.)cap=std::min(cap,2.*(forwardGap-1.));
     else if(closing>0.2 && forwardGap/closing<3.)cap=std::min(cap,std::max(0.,speed-2.));
   }
   // No automatic lateral teleport: every lateral change uses the original
   // 2.5-second quintic blend. In solo runs hold the original grid lane.
   const double roadSafe=std::max(0.,.5*sample(s).width-kHalfWidth-.10);
   offset=clamp(offset,-roadSafe,roadSafe);
   if(changing){
     const double t=(now-changeAt)/2.5;
     offset=fromOffset+(goalOffset-fromOffset)*ease(t);
     offset=clamp(offset,-roadSafe,roadSafe);
     if(t>=1.)changing=false;
   }
   // The original opponent-avoidance lane-change policy is activated only
   // if a real teammate lies directly ahead and there is room.
   if(blocked && forwardGap>2. && forwardGap<9. &&
      closing>1. && !sharpAhead && !changing && now-lastChange>5.){
     const double laneSpacing=.465;
     for(double dir:{-1.,1.}){
       const double candidate=clamp(offset+dir*laneSpacing,-roadSafe,roadSafe);
       if(std::abs(candidate-offset)<.25)continue;
       const Pose end=position(s+18.,candidate);
       bool free=true;
       for(const auto&o:other)if(o.found &&
         std::hypot(end.x-o.pose.x,end.y-o.pose.y)<1.2)free=false;
       if(free){fromOffset=offset;goalOffset=candidate;changing=true;changeAt=now;lastChange=now;break;}
     }
   }
   const double desired=clamp(cap,0.,15.);
   speed+=clamp(desired-speed,-5.*dt,3.*dt);
   const double ds=speed*dt;
   s+=ds;
   const Pose next=position(s,offset);
   // Match the original yaw interpolation: rotate towards the actual motion
   // tangent rather than sliding with the old heading.
   const double dx=next.x-lastPose.x,dy=next.y-lastPose.y;
   double heading=next.yaw;
   if(std::hypot(dx,dy)>.015)
     heading=lastPose.yaw+.35*wrap(std::atan2(dy,dx)-lastPose.yaw);
   else
     heading=lastPose.yaw+.35*wrap(next.yaw-lastPose.yaw);
   // Position command is issued only for TEAM08; never for team_01/02.
   const Pose moving{next.x,next.y,next.z+zOffset,heading};
   gz::sim::Model(model).SetWorldPoseCmd(ecm,gz::math::Pose3d(
       moving.x,moving.y,moving.z,0.,0.,moving.yaw));
   const double rate=wrap(heading-lastPose.yaw)/dt;
   const double steering=clamp(std::atan2(rate*.34,std::max(.1,speed)),-.52,.52);
   lastPose=moving;
   publish(speed,steering);
   if(now-lastReport>3.){
     lastReport=now;
     std::cout<<"[TEAM08 POSE] s="<<s<<" speed="<<speed
              <<" curve="<<curveNow<<" yaw="<<heading
              <<" lat="<<offset<<" opponent_gap="<<forwardGap<<std::endl;
   }
 }
 private:
 void onStart(const gz::msgs::Boolean&b){go.store(b.data());}
 void publish(double v,double steer){
   gz::msgs::Float x;x.set_data(static_cast<float>(v));speedPub.Publish(x);
   gz::msgs::Float y;y.set_data(static_cast<float>(steer));steerPub.Publish(y);
 }
 Pt sample(double distance)const {
   const auto&route=maps[mapIndex].second;
   double t=std::fmod(distance,lap);if(t<0)t+=lap;
   auto it=std::upper_bound(route.begin(),route.end(),t,
      [](double a,const Pt&b){return a<b.s;});
   size_t i=it==route.begin()?0:static_cast<size_t>(it-route.begin()-1);
   size_t j=(i+1)%route.size();
   const auto&a=route[i],&b=route[j];
   double end=j==0?lap:b.s;
   double f=clamp((t-a.s)/std::max(.00001,end-a.s),0.,1.);
   return {t,a.x+(b.x-a.x)*f,a.y+(b.y-a.y)*f,a.z+(b.z-a.z)*f,
           a.yaw+wrap(b.yaw-a.yaw)*f,a.width+(b.width-a.width)*f};
 }
 Pose position(double distance,double lateral)const {
   const auto p=sample(distance);
   return {p.x-lateral*std::sin(p.yaw),p.y+lateral*std::cos(p.y),p.z,p.yaw};
 }
 double curve(double distance)const{
   const auto&a=sample(distance-1.),b=sample(distance+1.);
   return wrap(b.yaw-a.yaw)/2.;
 }
 std::array<Other,2> readOther(gz::sim::EntityComponentManager&ecm,double now){
   for(auto &o:opponents)o.found=false;
   ecm.Each<gz::sim::components::Model,gz::sim::components::Name>(
     [&](const gz::sim::Entity&e,const gz::sim::components::Model*,
         const gz::sim::components::Name*name)->bool{
       for(auto&o:opponents)if(name->Data()==o.id){
         const auto pose=gz::sim::worldPose(e,ecm);
         const Pose p{pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z(),pose.Rot().Yaw()};
         if(o.last>0 && now>o.last){
           const double dt=now-o.last;
           if(dt>.02 && dt<2.)o.speed=std::hypot(p.x-o.before.x,p.y-o.before.y)/dt;
         }
         o.pose=p;o.before=p;o.last=now;o.found=true;
       }
       return true;
     });
   return opponents;
 }
 gz::sim::Entity model{gz::sim::kNullEntity};
 gz::transport::Node node;
 gz::transport::Node::Publisher speedPub,steerPub;
 std::atomic<bool>go{false};
 std::vector<std::pair<std::string,std::vector<Pt>>> maps;
 std::array<Other,2> opponents{{{"team_01"},{"team_02"}}};
 bool initialized=false,started=false,changing=false;
 int mapIndex=-1;
 double s=0,offset=0,goalOffset=0,fromOffset=0,lap=0;
 double zOffset=.04, speed=0,lastTime=0,changeAt=0,lastChange=-1e6,lastReport=0;
 Pose lastPose;
 gz::math::Pose3d spawnPose;
};
}
GZ_ADD_PLUGIN(urrc::Team08Pose_08,gz::sim::System,urrc::Team08Pose_08::ISystemConfigure,urrc::Team08Pose_08::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::Team08Pose_08,"urrc::Team08Pose_08")
