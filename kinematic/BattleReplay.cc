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
#include <chrono>
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
  gz::msgs::Boolean readyMsg;readyMsg.set_data(true);ready.Publish(readyMsg);
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
  preferredLane=cfg->Get<double>("team_preferred_offset_m");
  initialLane=cfg->Get<int>("team_initial_lane");
  teamGrid=cfg->Get<int>("team_grid");
  lead.name="practice_s_09";lead.s=cfg->Get<double>("lead_start_s");lead.lat=-lateralOffset;
  rear.name="practice_rear_11";rear.s=cfg->Get<double>("rear_start_s");
  rear.lat=cfg->Get<double>("rear_start_offset_m");
  report.open(cfg->Get<std::string>("log_file"));
  if(route.size()<100||lap<100||lateralOffset<=0||!report){
   std::cerr<<"[BATTLE] INVALID SETUP"<<std::endl;return;
  }
  report<<"time_s,team_s,team_lat,team_speed,lead_s,lead_lat,lead_speed,rear_s,rear_lat,rear_speed,front_ttc,rear_ttc,clearance_m\n";
  ok=true;
  std::cout<<"[BATTLE] READY grid="<<teamGrid<<" lane="<<initialLane<<" team 12m/s, S-car 9m/s, rear grid10 11m/s"<<std::endl;
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
    // Start at the actual official grid, then request a safe lane join.
    joinPending=std::fabs(team.lat-preferredLane)>.05;
    std::cout<<"[BATTLE] START grid="<<teamGrid<<" (all five lights off)"<<std::endl;
  }
  const double elapsed=now-begin;
  // Use actual simulation-time delta. Limiting dt to 50ms dropped distance
  // whenever Gazebo advances in larger steps and caused visible stutter.
  const double dt=std::clamp(now-last,0.,.2);last=now;
  if(dt<=0)return;
  // Preserve previous poses BEFORE lateral and longitudinal motion.
  // Needed to orient each car along its actual diagonal travel tangent.
  const Pos previousTeam=pose(team), previousLead=pose(lead), previousRear=pose(rear);
  const double lateral[5]={-lateralOffset,0,lateralOffset,0,-lateralOffset};
  int period=static_cast<int>(elapsed/2.0);
  int phase=period%4;
  lead.lat=lateral[phase]+(lateral[phase+1]-lateral[phase])*ease((elapsed-2.0*period)/2.0);
  double frontGap=gap(team.s,lead.s),rearGap=gap(rear.s,team.s);
  double frontTTC=team.v>lead.v+.01?frontGap/(team.v-lead.v):1e6;
  double rearTTC=rear.v>team.v+.01?rearGap/(rear.v-team.v):1e6;
  bool corner=fabs(curve(team.s))>.18;
  minRearTTC=std::min(minRearTTC,rearTTC);
  if(rearTTC<=5. && rearGap>LEN)defenseOpportunity=true;
  // A change is a committed manoeuvre: keep its goal unless a *predicted
  // footprint collision* is detected. Negative clearance by itself is not
  // a collision and must not create oscillating replan requests.
  if(team.changing){
    const double risk=forecast(team.to,elapsed,now,true);
    if(risk<=-0.5 && now-lastReplan>1.0){
      lastReplan=now;
      const double stay=forecast(team.lat,elapsed,now,false);
      const double back=forecast(team.from,elapsed,now,false);
      if(back>0.0 && back>stay+0.15){
        change(team.from,now);replanCount++;
      }else if(stay>0.0){
        change(team.lat,now);replanCount++;
      }
      emergencyBraking=true;
      std::cout<<"[BATTLE] HAZARD_REPLAN future="<<risk
               <<" stay="<<stay<<" back="<<back<<std::endl;
    }
  }else if(joinPending){
    if(forecast(preferredLane,elapsed,now,false)>0){
      change(preferredLane,now);joinPending=false;
      std::cout<<"[BATTLE] JOIN initial lane="<<initialLane<<std::endl;
    }
  }else if(now-changeFinishedAt<2.0){
    // Hysteresis prevents immediate re-selection of the opposite lane.
  }else if(!corner && !passed && frontGap>1.0 && frontGap<20. && frontTTC<=3. &&
            (fabs(team.lat-lead.lat)<WID+.18 ||
             forecast(team.lat,elapsed,now,false,team.v,2.5)<=-0.5)){
    // Never turn into another lane merely to "overtake" a vehicle already
    // travelling safely in the adjacent lane; accelerate past it instead.
    double bestScore=-1e9,bestTarget=team.lat;
    for(double dst:{-lateralOffset,0.,lateralOffset}){
      if(fabs(dst-team.lat)<.2*lateralOffset)continue;
      const double score=forecast(dst,elapsed,now,false);
      const double cost=.08*fabs(dst-team.lat);
      if(score>0.0 && score-cost>bestScore){
        bestScore=score-cost;bestTarget=dst;
      }
    }
    if(fabs(bestTarget-team.lat)>.2*lateralOffset){
      change(bestTarget,now);attempted=true;attemptCount++;
      std::cout<<"[BATTLE] OVERTAKE ATTEMPT TTC="<<frontTTC
               <<"s target_offset="<<bestTarget<<" score="<<bestScore<<std::endl;
    }
  }else if(!corner && !defended && defenseOpportunity &&
            rearGap>1.5 && rearTTC<=5.0 &&
            rearTTC<previousRearTTC-.002 && frontTTC>3.){
    // Defender is allowed to choose a corridor only before the pursuer
    // reaches its flank. No deliberate abrupt block of an adjacent car.
    const double currentScore=forecast(team.lat,elapsed,now,false);
    const double target=rear.lat>=0?lateralOffset:-lateralOffset;
    if(fabs(target-team.lat)<.2*lateralOffset && currentScore>0){
      defended=true;defenseOutcome="HELD_SAFE_LANE";
      std::cout<<"[BATTLE] DEFENSE SAFE_HOLD TTC="<<rearTTC<<std::endl;
    }else{
      double score=forecast(target,elapsed,now,false);
      if(score>0 && score>currentScore+0.2 && rearGap>3.0){
        change(target,now);defended=true;defenseOutcome="POSITIONED";
        std::cout<<"[BATTLE] DEFENSE TTC="<<rearTTC
                 <<"s target_offset="<<target<<std::endl;
      }else{
        defenseOutcome="SAFE_ABORT";
        if(now-lastDefenseLog>=1.0){
          lastDefenseLog=now;
          std::cout<<"[BATTLE] DEFENSE SAFE_ABORT TTC="<<rearTTC
                   <<" candidate="<<score<<" current="<<currentScore<<std::endl;
        }
      }
    }
  }
  if(team.changing){
   const double t=(now-team.changeStart)/team.changeDuration;
   team.lat=team.from+(team.to-team.from)*ease(t);
   if(t>=1){team.lat=team.to;team.changing=false;changeFinishedAt=now;}
  }
  // Only cars on overlapping physical corridors restrict speed.
  // A neighbouring car cannot force TEAM11 to match its 9m/s speed.
  auto cruise=[&](const Car &a,double limit){
    double wanted=limit;
    for(const Car*b:{&team,&lead,&rear}){
      if(b==&a)continue;
      const double forward=gap(a.s,b->s);
      if(forward>=16.)continue;
      const double lateralGap=fabs(a.lat-b->lat);
      const bool overlapping=lateralGap<(WID+.09);
      if(!overlapping)continue;
      // Keep 1m+speed-dependent following buffer, but do not instantly
      // match the slower car when a clear pass is already under way.
      const double buffer=1.0+0.10*a.v;
      const double permitted=std::max(0.0,b->v+
        std::max(0.0,forward-buffer)*1.7);
      wanted=std::min(wanted,permitted);
    }
    return std::clamp(wanted,0.,limit);
  };
  const double teamCruise=12.0;
  double vt=cruise(team,teamCruise),vl=cruise(lead,9),vr=cruise(rear,11);
  if(team.changing)vt=std::min(vt,teamCruise-1.5);
  if(emergencyBraking)vt=std::min(vt,5.0);
  // Short-horizon emergency forecast, not continuous 5s speed matching.
  // Long-horizon forecast remains for candidate lane *selection*.
  if(now-lastSpeedForecast>=.1){
    lastSpeedForecast=now;
    speedCeiling=teamCruise;
    const double selected=team.changing?team.to:team.lat;
    const bool crossing=fabs(selected-lead.lat)<(WID+.15) ||
                        fabs(selected-rear.lat)<(WID+.15);
    if(crossing){
      for(double trial:{12.,10.5,9.,7.,5.,3.,0.}){
        if(trial>vt+.01)continue;
        const double safety=forecast(selected,elapsed,now,team.changing,trial,1.8);
        if(safety>=-0.5){speedCeiling=trial;break;}
        speedCeiling=0.;
      }
    }
    if(speedCeiling<vt-1. && now-lastSpeedLog>=2.){
      lastSpeedLog=now;
      std::cout<<"[BATTLE] CLOSE_HAZARD_BRAKE desired="<<vt
               <<" limit="<<speedCeiling<<" front_ttc="<<frontTTC<<std::endl;
    }
  }
  vt=std::min(vt,speedCeiling);
  if(!team.changing && frontGap>2.0 && !joinPending)emergencyBraking=false;
  if((vl<8.9||vr<10.9)&&elapsed-lastBrake>1.5){
    lastBrake=elapsed;braked=true;
    std::cout<<"[BATTLE] PRACTICE BRAKING lead="<<vl<<" rear="<<vr<<std::endl;
  }
  auto update=[dt](Car &a,double desired){
    // Smoothed speed transitions: integrate trapezoidally over real dt.
    const double before=a.v;
    a.v+=std::clamp(desired-a.v,-5.0*dt,3.0*dt);
    a.s+=(before+a.v)*.5*dt;
  };
  update(team,vt);update(lead,vl);update(rear,vr);
  if(team.s>lead.s+LEN+0.4&&!passed &&
     fabs(team.lat-lead.lat)>WID+.12){
    passed=true;
    std::cout<<"[BATTLE] OVERTAKE COMPLETED mode="
             <<(attempted?"LANE_CHANGE":"CLEAR_LANE_ACCELERATION")<<std::endl;
  }
  Pos a=pose(team),b=pose(lead),c=pose(rear);
  // Look-ahead tangent is less sensitive to microscopic position noise
  // than a one-frame heading; retain straight-road orientation at low speed.
  auto alignToMotion=[](Pos &next,const Pos &previous){
    const double dx=next.x-previous.x,dy=next.y-previous.y;
    if(std::hypot(dx,dy)>0.015){
      const double measured=std::atan2(dy,dx);
      next.yaw=previous.yaw+wrap(measured-previous.yaw)*0.35;
    }else next.yaw=previous.yaw;
  };
  alignToMotion(a,previousTeam);
  alignToMotion(b,previousLead);
  alignToMotion(c,previousRear);
  // Swept SAT: sample previous-to-next movement to prevent tunneling.
  auto blend=[](const Pos&old,const Pos&next,double t){
    return Pos{old.x+(next.x-old.x)*t,old.y+(next.y-old.y)*t,
               old.z+(next.z-old.z)*t,
               old.yaw+wrap(next.yaw-old.yaw)*t};
  };
  for(int sweep=0;sweep<=10;sweep++){
    double t=static_cast<double>(sweep)/10.;
    Pos ap=blend(previousTeam,a,t);
    Pos bp=blend(previousLead,b,t);
    Pos cp=blend(previousRear,c,t);
    for(auto pair:std::array<std::pair<Pos,Pos>,3>{{{ap,bp},{ap,cp},{bp,cp}}}){
      double dist=hypot(pair.first.x-pair.second.x,pair.first.y-pair.second.y);
      minimumCenter=std::min(minimumCenter,dist);
      if(collide(pair.first,pair.second)){
        std::cout<<"[BATTLE] COLLISION at t="<<elapsed<<"s"<<std::endl;
        finish("FAIL_COLLISION");return;
      }
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
             <<" defense_status="<<(defenseOpportunity?defenseOutcome:"NOT_TRIGGERED")
             <<" minimum_rear_ttc="<<minRearTTC<<" attempts="<<attemptCount
             <<" replans="<<replanCount<<" practice_braking="<<braked
             <<" minimum_center_m="<<minimumCenter<<std::endl;
    finish(!passed?"FAIL_NO_OVERTAKE":(defended?"PASS_BATTLE":"PARTIAL_DEFENSE_UNVERIFIED"));return;
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
 // Predict both opponents' future occupancy and score each lane-change
 // candidate. Negative means a predicted collision or >30deg equivalent
 // steering. Positive means minimum centre clearance above collision radius.
 double forecast(double destination,double elapsed,double now,bool active,double desiredSpeed=-1.,double horizon=5.)const{
  double minGap=1e6;
  Pos lastP=pose(team);
  Pos lastLastP=lastP;
  double lastHeading=lastP.yaw;
  for(int i=1;i<=static_cast<int>(horizon*10+.5);i++){
    const double t=i*.1;
    Car candidate=team;
    const double vmax=desiredSpeed<0?std::max(0.,team.v):desiredSpeed;
    const double acc=vmax<team.v?-12.:6.;
    const double reach=fabs(vmax-team.v)/fabs(acc);
    const double until=std::min(t,reach);
    const double distance=team.v*until+.5*acc*until*until+
                          vmax*std::max(0.,t-until);
    candidate.s+=std::max(0.,distance);
    const double from=active?team.from:team.lat;
    const double start=active?team.changeStart:now;
    const double duration=active?team.changeDuration:2.5;
    candidate.lat=from+(destination-from)*ease((now+t-start)/duration);
    Pos p=pose(candidate);
    const double ds=hypot(p.x-lastP.x,p.y-lastP.y);
    if(ds>1e-4){
      p.yaw=atan2(p.y-lastP.y,p.x-lastP.x);
      if(i>1){
        const double k=fabs(wrap(p.yaw-lastHeading))/std::max(.001,ds);
        if(atan(.36*k)>30.*3.141592653589793/180. + .001)return -2.;
      }
      lastHeading=p.yaw;
    }
    for(const Car*o:{&lead,&rear}){
      Car sample=*o;
      sample.s+=t*std::min(o==&lead?9.:11.,std::max(o->v,0.)+6.*t);
      if(o==&lead){
        const double seq[5]={-lateralOffset,0,lateralOffset,0,-lateralOffset};
        const double absolute=elapsed+t;
        const int period=static_cast<int>(absolute/2.);
        const int phase=period%4;
        sample.lat=seq[phase]+(seq[phase+1]-seq[phase])*ease((absolute-2.*period)/2.);
      }
      const Pos op=pose(sample);
      const double center=hypot(p.x-op.x,p.y-op.y);
      minGap=std::min(minGap,center);
      // Inflate both future footprints with 0.15m positional uncertainty.
      if(center<LEN+.15 && collide(p,op))return -1.;
    }
    lastLastP=lastP;lastP=p;
  }
  return minGap-(LEN+.15);
 }
 void change(double dest,double now){
  team.from=team.lat;team.to=dest;team.changeStart=now;
  team.changeDuration=2.5;team.changing=true;
 }
 void finish(const char*result){
  if(ended)return;ended=true;report.flush();
  std::cout<<"[BATTLE] RESULT "<<result<<" overtake="<<passed<<" defense="<<defended
           <<" defense_status="<<(defenseOpportunity?defenseOutcome:"NOT_TRIGGERED")
           <<" rear_ttc_min="<<minRearTTC<<" attempts="<<attemptCount
           <<" replans="<<replanCount<<" braked="<<braked
           <<" min_center="<<minimumCenter<<std::endl;
 }
 void Start(const gz::msgs::Boolean&b){if(b.data())startSignal.store(true);}
 gz::sim::Entity worldEntity{gz::sim::kNullEntity};
 gz::transport::Node node;gz::transport::Node::Publisher ready;
 std::atomic<bool> startSignal{false};
 std::vector<Pt>route;Car team,lead,rear;
 std::ofstream report;double lap=0,lateralOffset=0,teamStart=0,begin=0,last=0,lastCsv=0,lastBrake=-10;
 double previousRearTTC=1e6,minRearTTC=1e6,minimumCenter=1e9,preferredLane=0.;
 double lastReplan=-1e6,changeFinishedAt=-1e6;
 double lastSpeedForecast=-1e6,lastSpeedLog=-1e6,lastDefenseLog=-1e6,speedCeiling=10.;
 int attemptCount=0,replanCount=0;
 std::string defenseOutcome="NOT_TRIGGERED";
 bool defenseOpportunity=false;
 bool joinPending=false,emergencyBraking=false;
 int initialLane=1,teamGrid=1;
 bool ok=false,started=false,ended=false,passed=false,defended=false,attempted=false,braked=false;
};
}
GZ_ADD_PLUGIN(urrc::BattleReplay,gz::sim::System,urrc::BattleReplay::ISystemConfigure,urrc::BattleReplay::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::BattleReplay,"urrc::BattleReplay")
