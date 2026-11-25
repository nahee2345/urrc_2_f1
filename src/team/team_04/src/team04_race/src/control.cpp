#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
class Control: public rclcpp::Node {
 rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speedSub,steerSub;
 rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr startSub;
 rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cameraSub;
 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive;
 rclcpp::TimerBase::SharedPtr timer;
 bool exclusive=false;std::string speedTopic,steerTopic;
 bool permit=false,redSeen=false,lightsOut=false,steeringValid=true;int offFrames=0,redFrames=0;
 double speed=0,steer=0,timeout,cameraRoi;std::string mode;
 std::chrono::steady_clock::time_point speedTime{},steerTime{};
 rclcpp::Time lastImage{0,0,RCL_ROS_TIME};
 public: Control():Node("control_04") {
  mode=declare_parameter("race_start_mode",std::string("signal"));
  if(mode!="signal" && mode!="camera")throw std::invalid_argument("race_start_mode must be signal or camera");
  timeout=declare_parameter("command_timeout_s",.5);
  exclusive=declare_parameter("exclusive_command_source",false);
  speedTopic=declare_parameter("speed_topic",std::string("/team04/speed"));
  steerTopic=declare_parameter("steering_topic",std::string("/team04/steering"));
  cameraRoi=declare_parameter("camera_roi_y_max",.25);
  if(!std::isfinite(cameraRoi) || cameraRoi<=0 || cameraRoi>1)throw std::invalid_argument("invalid camera ROI");
  drive=create_publisher<geometry_msgs::msg::Twist>("/team04/drive",10);
  auto qos=rclcpp::QoS(10).reliable().durability_volatile();
  startSub=create_subscription<std_msgs::msg::Bool>(declare_parameter("race_start_topic",std::string("/race/start/team04")),qos,[this](std_msgs::msg::Bool::ConstSharedPtr m){
   if(!m->data){if(permit){redSeen=false;lightsOut=false;offFrames=redFrames=0;}speed=0;speedTime={};}
   permit=m->data;publish();
  });
  speedSub=create_subscription<std_msgs::msg::Float32>(speedTopic,10,[this](std_msgs::msg::Float32::ConstSharedPtr m){
   speed=std::isfinite(m->data)?std::clamp(double(m->data),-2.,5.):0.;speedTime=std::chrono::steady_clock::now();publish();
  });
  steerSub=create_subscription<std_msgs::msg::Float32>(steerTopic,10,[this](std_msgs::msg::Float32::ConstSharedPtr m){
   steeringValid=std::isfinite(m->data);
   if(!steeringValid){steer=0;speed=0;speedTime={};}else steer=std::clamp(double(m->data),-.6108652382,.6108652382);
   steerTime=std::chrono::steady_clock::now();publish();
  });
  if(mode=="camera")cameraSub=create_subscription<sensor_msgs::msg::Image>("/team04/camera/image_raw",rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::Image::ConstSharedPtr m){image(*m);});
  timer=create_wall_timer(std::chrono::milliseconds(20),[this]{publish();});
 }
 void image(const sensor_msgs::msg::Image &m){
  const auto stamp=rclcpp::Time(m.header.stamp,get_clock()->get_clock_type());
  const double age=(now()-stamp).seconds();
  if(age<-.02 || age>.3 || stamp<=lastImage)return;
  lastImage=stamp;
  const bool bgr=m.encoding=="bgr8" || m.encoding=="bgra8";
  const size_t channels=(m.encoding=="rgb8" || m.encoding=="bgr8")?3:((m.encoding=="rgba8" || m.encoding=="bgra8")?4:0);
  if(!channels || m.step<m.width*channels || m.data.size()<size_t(m.step)*m.height)return;
  size_t red=0; // Upper quarter of the field of view: camera is exclusively used for start recognition.
  for(size_t y=0;y<size_t(m.height*cameraRoi);++y)for(size_t x=0;x<m.width;++x){
   const auto p=&m.data[y*m.step+x*channels];int r=p[bgr?2:0],g=p[1],b=p[bgr?0:2];
   if(r>150 && r>g*1.7 && r>b*1.7)++red;
  }
  if(red>=12){if(++redFrames>=2)redSeen=true;offFrames=0;lightsOut=false;}
  else {redFrames=0;if(redSeen && ++offFrames>=3)lightsOut=true;}
  publish();
 }
 void publish(){
  const auto t=std::chrono::steady_clock::now();geometry_msgs::msg::Twist out;
  const bool cameraOK=mode=="signal" || (redSeen && lightsOut);
  const bool sourceOK=!exclusive || (count_publishers(speedTopic)<=1 && count_publishers(steerTopic)<=1);
  if(sourceOK && permit && steeringValid && cameraOK && std::chrono::duration<double>(t-speedTime).count()<timeout)out.linear.x=speed;
  // Official +right -> internal +left. Steering times out independently to centre.
  if(sourceOK && std::chrono::duration<double>(t-steerTime).count()<timeout)out.angular.z=-steer;
  drive->publish(out);
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Control>());rclcpp::shutdown();}
