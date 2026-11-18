#!/usr/bin/env python3
"""Create two independent scripted practice entrants for TEAM08 official-map tests.
Team 01: 9 m/s weaving lead car; Team 02: 11 m/s following rear car.
This never modifies TEAM08's proven driving code.
"""
from pathlib import Path
import json
import shutil
import textwrap

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src/team/team_08"
assert (SRC / "models/team08_car/model.sdf").exists(), "TEAM08 model missing"
TRACKS = ("monza", "spa", "monaco", "cota", "hungaroring", "baku")

CPP = r'''
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
#include <atomic>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
namespace urrc {
struct Pt {double s,x,y,z,yaw,width;};
double bound(double v,double a,double b){return std::max(a,std::min(v,b));}
double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
double ease(double t){t=bound(t,0.,1.);return t*t*t*(10.+t*(-15.+6.*t));}
class PracticeTraffic__NUM__ final:public gz::sim::System,
 public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 public:
 void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&,
                gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
  entity=e;
  node.Subscribe("/team__SHORT__/replay_go___SHORT__",
                 &PracticeTraffic__NUM__::start,this);
  speedPub=node.Advertise<gz::msgs::Float>("/team__SHORT__/speed___SHORT__");
  steerPub=node.Advertise<gz::msgs::Float>("/team__SHORT__/steering___SHORT__");
  try{
   auto share=ament_index_cpp::get_package_share_directory("team__SHORT___race");
   for(auto name:{"monza","spa","monaco","cota","hungaroring","baku"}){
    std::ifstream file(share+"/maps/"+name+"/centerline.csv");
    std::string row;std::getline(file,row);std::vector<Pt> pts;
    while(std::getline(file,row)){std::replace(row.begin(),row.end(),',',' ');
     Pt p{};std::istringstream in(row);
     if(in>>p.s>>p.x>>p.y>>p.z>>p.yaw>>p.width)pts.push_back(p);
    }
    if(pts.size()>100)maps.push_back({name,std::move(pts)});
   }
  }catch(const std::exception&e){std::cerr<<"[PRACTICE] maps: "<<e.what()<<std::endl;}
 }
 void PreUpdate(const gz::sim::UpdateInfo&i,gz::sim::EntityComponentManager&ecm) override{
  if(i.paused)return;
  const double now=std::chrono::duration<double>(i.simTime).count();
  if(!ready && !initialize(ecm,now))return;
  if(!go.load()){
   if(!started)gz::sim::Model(entity).SetWorldPoseCmd(ecm,initial);
   last=now;speed=0;publish(now,0,0);return;
  }
  if(!started){started=true;t0=now;last=now;}
  double delta=now-last;
  if(delta<.005)return;
  const double dt=std::min(delta,.2);last=now;
  double desired=__SPEED__;
  // Original BattleReplay08 practice rules: lead cruises at 9 m/s;
  // rear pursuer cruises at 11 m/s unless another vehicle ahead
  // requires the original front-gap/TTC following adjustment.
  if(__REAR__){
   double bestGap=1e9;double targetSpeed=desired;
   ecm.Each<gz::sim::components::Model,gz::sim::components::Name,
            gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity&e,const gz::sim::components::Model*,
        const gz::sim::components::Name*n,
        const gz::sim::components::ParentEntity*)->bool{
      if(e==entity || n->Data()!="team_08")return true;
      auto p=gz::sim::worldPose(e,ecm);
      double targetS=0,targetLat=0;
      if(!project(p,targetS,targetLat))return true;
      const double gap=std::remainder(targetS-s,lap);
      if(gap>0. && gap<20. && gap<bestGap){
        const double bumper=std::max(0.,gap-.53);
        const bool inCorner=std::abs(curve(s))>.18;
        const double minimum=inCorner?.70:.50;
        const double ttc=speed>9.05?(bumper-minimum)/(speed-9.):1e6;
        double cap=desired;
        if(std::abs(targetLat-lateral)<.34){
          if(ttc<=1. || bumper<=minimum)
            cap=std::min(cap,9.+std::max(0.,bumper-minimum)*1.2);
          else if(inCorner&&bumper<4.)
            cap=std::min(cap,9.+std::max(0.,bumper-minimum)*.45);
          else if(ttc<3.)
            cap=std::min(cap,9.+std::max(0.,bumper-minimum)*.9);
        }
        // Original Monaco rear-car anticipatory braking horizon.
        const bool nearCorner=inCorner||
          std::abs(curve(s+speed*2.))>.18||
          std::abs(curve(targetS+18.))>.18;
        const double desiredGap=(nearCorner?1.:1.)+.55;
        const double free=gap-.53-desiredGap;
        const double closing=std::max(0.,speed-9.);
        const double anticipation=std::max(7.,closing*3.+
                                closing*closing/(2.*4.)+desiredGap);
        if(free<anticipation){
          const double permitted=9.+bound((free-1.4)*.22,-1.,1.2);
          cap=std::min(cap,std::max(0.,permitted));
        }
        bestGap=gap;targetSpeed=cap;
      }
      return true;
    });
   if(bestGap<15.)desired=targetSpeed;
  }
  speed+=bound(desired-speed,-5.*dt,3.*dt);
  s+=(speed*dt);
  if(!__REAR__){
   // Original BattleReplay08 S-traffic: start from the administrator's
   // actual grid lateral offset, then follow its same 2-second quintic
   // [-offset, 0, +offset, 0, -offset] lane sequence.
   const double elapsed=now-t0,offset=.465;
   const double lanes[5]={-offset,0.,offset,0.,-offset};
   const int phase=static_cast<int>(elapsed/2.)%4;
   const double fraction=ease((elapsed-2.*static_cast<int>(elapsed/2.))/2.);
   const double from=(phase==0 && elapsed<2.)?initialLateral:lanes[phase];
   lateral=from+(lanes[phase+1]-from)*fraction;
  }
  const auto p=sample(s);
  const double safe=std::max(0.,p.width*.5-.125-.1);
  lateral=bound(lateral,-safe,safe);
  const double x=p.x-lateral*std::sin(p.yaw);
  const double y=p.y+lateral*std::cos(p.yaw);
  const double movement=std::hypot(x-xPrev,y-yPrev);
  const double heading=movement>.015?
    yawPrev+.35*wrap(std::atan2(y-yPrev,x-xPrev)-yawPrev):
    yawPrev+.35*wrap(p.yaw-yawPrev);
  const double rate=wrap(heading-yawPrev)/dt;
  xPrev=x;yPrev=y;yawPrev=heading;
  gz::sim::Model(entity).SetWorldPoseCmd(ecm,gz::math::Pose3d(
    x,y,p.z+.04,0.,0.,heading));
  publish(now,speed,bound(std::atan2(rate*.34,std::max(.1,speed)),-.52,.52));
 }
 private:
 void start(const gz::msgs::Boolean&m){go.store(m.data());}
 void publish(double now,double v,double steer){
  if(now-lastPub<.025)return;lastPub=now;
  gz::msgs::Float a;a.set_data(v);speedPub.Publish(a);
  gz::msgs::Float b;b.set_data(steer);steerPub.Publish(b);
 }
 double curve(double distance)const{
   const auto a=sample(distance-1.),b=sample(distance+1.);
   return wrap(b.yaw-a.yaw)/2.;
 }
 Pt sample(double distance)const{
  double d=std::fmod(distance,lap);if(d<0)d+=lap;
  auto it=std::upper_bound(route.begin(),route.end(),d,
     [](double a,const Pt&b){return a<b.s;});
  size_t j=it==route.begin()?0:static_cast<size_t>(it-route.begin()-1);
  size_t k=(j+1)%route.size();
  auto&a=route[j];auto&b=route[k];
  const double end=k==0?lap:b.s;
  const double t=bound((d-a.s)/std::max(1e-6,end-a.s),0.,1.);
  return {d,a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t,
    a.yaw+wrap(b.yaw-a.yaw)*t,a.width+(b.width-a.width)*t};
 }
 bool project(const gz::math::Pose3d&p,double&distance,double&lat)const{
  const double x=p.Pos().X(),y=p.Pos().Y();double best=1e30;
  for(size_t i=0;i+1<route.size();++i){
   const auto&a=route[i],&b=route[i+1];
   const double dx=b.x-a.x,dy=b.y-a.y;
   const double t=bound(((x-a.x)*dx+(y-a.y)*dy)/
                   std::max(1e-9,dx*dx+dy*dy),0.,1.);
   const double px=a.x+t*dx,py=a.y+t*dy,d=std::hypot(px-x,py-y);
   if(d<best){best=d;distance=a.s+t*(b.s-a.s);
     lat=(x-px)*(-std::sin(a.yaw))+(y-py)*std::cos(a.yaw);}
  }
  return best<2.;
 }
 bool initialize(gz::sim::EntityComponentManager&ecm,double now){
  if(entity==gz::sim::kNullEntity||maps.empty())return false;
  initial=gz::sim::worldPose(entity,ecm);
  double minScore=1e30,chosenS=0,chosenLat=0;int which=-1;
  for(size_t m=0;m<maps.size();++m){
   route=maps[m].second;
   double ss=0,lat=0;
   if(!project(initial,ss,lat))continue;
   const auto p=sample(ss);
   const double score=std::hypot(p.x-initial.Pos().X(),p.y-initial.Pos().Y())
                + (std::abs(wrap(p.yaw-initial.Rot().Yaw()))>.8?100.:0.);
   if(score<minScore){minScore=score;which=static_cast<int>(m);
      chosenS=ss;chosenLat=lat;}
  }
  if(which<0||minScore>1.5)return false;
  route=maps[which].second;
  lap=route.back().s+std::hypot(route.front().x-route.back().x,
                                route.front().y-route.back().y);
  s=chosenS;lateral=chosenLat;initialLateral=chosenLat;
  const auto p=sample(s);
  xPrev=p.x-lateral*std::sin(p.yaw);
  yPrev=p.y+lateral*std::cos(p.yaw);yawPrev=p.yaw;
  last=now;ready=true;
  std::cout<<"[PRACTICE team___SHORT__] map="<<maps[which].first
           <<" initial_s="<<s<<" lateral="<<lateral<<std::endl;
  return true;
 }
 gz::sim::Entity entity{gz::sim::kNullEntity};
 gz::transport::Node node;
 gz::transport::Node::Publisher speedPub,steerPub;
 std::atomic<bool>go{false};
 bool ready=false,started=false;
 gz::math::Pose3d initial;
 std::vector<Pt>route;
 std::vector<std::pair<std::string,std::vector<Pt>>> maps;
 double s=0,lap=0,lateral=0,initialLateral=0,speed=0,last=0,t0=0,lastPub=-1e9,
        xPrev=0,yPrev=0,yawPrev=0;
};
}
GZ_ADD_PLUGIN(urrc::PracticeTraffic__NUM__,gz::sim::System,
 urrc::PracticeTraffic__NUM__::ISystemConfigure,
 urrc::PracticeTraffic__NUM__::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::PracticeTraffic__NUM__,"urrc::PracticeTraffic__NUM__")
'''

CONTROL = '''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class Gate(Node):
    def __init__(self):
        super().__init__("team__SHORT___practice_gate")
        self.publisher=self.create_publisher(Bool,"/team__SHORT__/replay_go___SHORT__",10)
        self.create_subscription(Bool,"/race/start/team___SHORT__",self.receive,10)
        self.value=False
        self.timer=self.create_timer(.05,self.publish)
    def receive(self,msg):
        self.value=bool(msg.data)
    def publish(self):
        msg=Bool()
        msg.data=self.value
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node=Gate()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
'''
LAUNCH = '''
from launch import LaunchDescription
from launch_ros.actions import Node
# Official start gate: /race/start/team___SHORT__

def generate_launch_description():
    return LaunchDescription([
        Node(package="ros_gz_bridge",executable="parameter_bridge",
             name="team__SHORT___practice_bridge",arguments=[
                "/team__SHORT__/replay_go___SHORT__@std_msgs/msg/Bool]gz.msgs.Boolean",
                "/team__SHORT__/speed___SHORT__@std_msgs/msg/Float32[gz.msgs.Float",
                "/team__SHORT__/steering___SHORT__@std_msgs/msg/Float32[gz.msgs.Float",
                "/team__SHORT__/front_scan___SHORT__@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/team__SHORT__/front_camera___SHORT__/image@sensor_msgs/msg/Image[gz.msgs.Image"
             ],output="screen"),
        Node(package="team__SHORT___race",executable="team__SHORT___control",
             name="team__SHORT___practice_control",output="screen",
             parameters=[{"use_sim_time": True}])
    ])
'''
CMAKE = '''
cmake_minimum_required(VERSION 3.16)
project(team__SHORT___race)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
ament_python_install_package(team__SHORT___race)
add_library(team__SHORT___practice SHARED pose_plugin/PracticeTraffic.cc)
target_compile_features(team__SHORT___practice PRIVATE cxx_std_17)
ament_target_dependencies(team__SHORT___practice ament_index_cpp)
target_link_libraries(team__SHORT___practice gz-sim8::gz-sim8 gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
install(TARGETS team__SHORT___practice LIBRARY DESTINATION lib)
install(PROGRAMS scripts/team__SHORT___control DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch maps models DESTINATION share/${PROJECT_NAME})
install(FILES race_entry.json DESTINATION share/${PROJECT_NAME})
ament_package()
'''
PACKAGE = '''<?xml version="1.0"?>
<package format="3">
  <name>team__SHORT___race</name>
  <version>0.1.0</version>
  <description>Independent TEAM__SHORT__ practice racing entrant</description>
  <maintainer email="practice@example.com">URRC Practice</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>ament_cmake_python</build_depend>
  <depend>ament_index_cpp</depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
'''

for number, role, pace in [(1, "weaving_lead", 9.0), (2, "following_rear", 11.0)]:
    short = f"{number:02d}"
    team = f"team_{short}"
    root = ROOT / "src/team" / team
    if root.exists():
        raise RuntimeError(f"{root} exists: refusing to overwrite a practice team folder")
    for d in ["models/practice_car", f"src/{team}/models/practice_car", "maps",
              "pose_plugin", "launch", f"team{short}_race", "scripts",
              "resource"]:
        (root / d).mkdir(parents=True, exist_ok=True)
    for track in TRACKS:
        destination = root / "maps" / track
        destination.mkdir(parents=True, exist_ok=True)
        shutil.copy2(SRC / "maps" / track / "centerline.csv", destination / "centerline.csv")
    model = (SRC / "models/team08_car/model.sdf").read_text()
    model = model.replace("team_08", team).replace("team08", f"team{short}")
    # TEAM08's plugin filename and private topic suffix must also be
    # remapped; otherwise practice models silently load the wrong plugin.
    model = model.replace(f"libteam{short}_pose_08.so", f"libteam{short}_practice.so")
    model = model.replace("urrc::Team08Pose_08", f"urrc::PracticeTraffic{short}")
    model = model.replace("_08", f"_{short}")
    (root / "models/practice_car/model.sdf").write_text(model)
    (root / f"src/{team}/models/practice_car/model.sdf").write_text(model)
    code = CPP.replace("__NUM__", short).replace("__SHORT__", short)
    code = code.replace("__SPEED__", f"{pace:.1f}").replace("__REAR__", "true" if number==2 else "false")
    (root / "pose_plugin/PracticeTraffic.cc").write_text(code)
    for path,source in [
        ("CMakeLists.txt",CMAKE),("package.xml",PACKAGE),
        ("launch/race.launch.py",LAUNCH),
        (f"team{short}_race/control.py",CONTROL)]:
        (root / path).write_text(textwrap.dedent(source).replace("__SHORT__",short))
    (root / f"team{short}_race/__init__.py").write_text("")
    (root / f"resource/team{short}_race").write_text("")
    executable = root / f"scripts/team{short}_control"
    executable.write_text(
       f"#!/usr/bin/env python3\nfrom team{short}_race.control import main\nif __name__=='__main__': main()\n")
    executable.chmod(0o755)
    entry = {
        "entry_id":team, "display_name":f"Practice {role} {team}",
        "ros_package":f"team{short}_race",
        "algorithm_launch_file":"race.launch.py",
        "vehicle_model_file":f"src/{team}/models/practice_car/model.sdf",
        "vehicle_model_format":"sdf", "base_link_name":"base_link",
        "spawn_z_offset_m":.04,
        "speed_topic":f"/team{short}/speed_{short}",
        "steering_topic":f"/team{short}/steering_{short}",
        "lidar_topic":f"/team{short}/front_scan_{short}",
        "launch_arguments":{}
    }
    (root / "race_entry.json").write_text(json.dumps(entry,indent=2)+"\n")
    (root / "README.md").write_text(
      f"# {team}: practice {role}\n\nIndependent Gazebo model controller. "
      f"Target speed {pace} m/s; starts only at official race gate. "
      "May be run in the same multi-team official rehearsal as team_08. "
      "Does not modify or command team_08.\n"
    )
print("Generated separate team_01 and team_02 Gazebo/ROS packages")
