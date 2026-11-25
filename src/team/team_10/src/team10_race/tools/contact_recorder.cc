// Read-only Gazebo contact telemetry. No publishers or vehicle commands.
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <atomic>
#include <csignal>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
std::atomic<bool> running{true};
void stop(int){running=false;}
int main(int argc,char**argv){
 if(argc!=2)return 2;std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
 std::ofstream out(argv[1]);out<<std::setprecision(12);std::mutex mutex;
 struct Count{long messages=0,contacts=0;double last=0,lastwrite=-1;};
 std::map<std::string,Count> counts;gz::transport::Node node;
 auto callback=[&](const gz::msgs::Contacts &m){
  std::lock_guard<std::mutex> guard(mutex);
  static double heartbeat=-1;
  double t=m.header().stamp().sec()+m.header().stamp().nsec()*1e-9;
  if(t-heartbeat>=1){out<<"{\"sim\":"<<t<<",\"heartbeat\":true,\"raw_contacts\":"<<m.contact_size()<<"}\n";out.flush();heartbeat=t;}
  for(const auto &v:m.contact()){
   auto a=v.collision1().name();auto b=v.collision2().name();
   if(a.find("team_10::")==std::string::npos&&b.find("team_10::")==std::string::npos)continue;
   if(a>b)std::swap(a,b);
   std::string pair=a+" | "+b;auto &c=counts[pair];c.messages++;c.contacts+=v.position_size();c.last=t;
   bool wall=pair.find("wall_")!=std::string::npos;
   bool road=pair.find("road_collision")!=std::string::npos||pair.find("ground_collision")!=std::string::npos;
   double interval=wall?.1:1.;
   if(t-c.lastwrite<interval)continue;c.lastwrite=t;
   double depth=0;for(double d:v.depth())depth=std::max(depth,d);
   out<<"{\"sim\":"<<t<<",\"pair\":\""<<pair<<"\",\"wall\":"<<(wall?"true":"false")<<",\"road\":"<<(road?"true":"false")<<",\"messages\":"<<c.messages<<",\"contact_points\":"<<c.contacts<<",\"max_depth\":"<<depth<<"}\n";
   out.flush();
  }
 };
 for(const std::string link:{"all","base_link","front_left_wheel","front_right_wheel","rear_left_wheel","rear_right_wheel","start_camera_link"}){
  node.Subscribe<gz::msgs::Contacts>("/team_10/contacts_10/"+link,callback);
 }
 node.Subscribe<gz::msgs::Float_V>("/team_10/steering_10/measured",std::function<void(const gz::msgs::Float_V&)>([&](const gz::msgs::Float_V &m){
  std::lock_guard<std::mutex> guard(mutex);double t=m.header().stamp().sec()+m.header().stamp().nsec()*1e-9;out<<"{\"sim\":"<<t<<",\"joint_angles\":[";for(int i=0;i<m.data_size();++i){if(i)out<<",";out<<m.data(i);}out<<"]}\n";out.flush();}));
 while(running)std::this_thread::sleep_for(std::chrono::milliseconds(100));
 return 0;
}
