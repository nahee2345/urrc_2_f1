// Read-only native image receipt timing. Never publishes camera/control data.
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>
std::atomic<bool> running{true};
int main(int argc,char**argv) {
 if(argc!=2)return 2;
 std::ofstream f(argv[1]);f<<std::setprecision(16);std::mutex m;
 std::signal(SIGINT,[](int){running=false;});std::signal(SIGTERM,[](int){running=false;});
 gz::transport::Node n;
 n.Subscribe("/team_10/start_camera_image_10",std::function<void(const gz::msgs::Image&)>([&](const gz::msgs::Image&i){
  double wall=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  double stamp=i.header().stamp().sec()+i.header().stamp().nsec()*1e-9;
  std::lock_guard<std::mutex> guard(m);
  f<<"{\"wall\":"<<wall<<",\"stamp\":"<<stamp<<"}\n";f.flush();
 }));
 while(running)std::this_thread::sleep_for(std::chrono::milliseconds(50));
}
