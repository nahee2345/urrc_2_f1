#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
class Scan:public rclcpp::Node {
 rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
 rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
 double zmin,zmax,maxAge,rate; rclcpp::Time previous{0,0,RCL_ROS_TIME};
 public:Scan():Node("scan_04"){
  zmin=declare_parameter("min_height",-.04);zmax=declare_parameter("max_height",.12);
  maxAge=declare_parameter("max_cloud_age_s",.2);rate=declare_parameter("lidar_hz",20.);
  if(rate<=0 || zmin>zmax)throw std::invalid_argument("invalid scan parameters");
  pub=create_publisher<sensor_msgs::msg::LaserScan>(declare_parameter("lidar_topic",std::string("/team04/scan")),rclcpp::SensorDataQoS().keep_last(1));
  sub=create_subscription<sensor_msgs::msg::PointCloud2>("/team04/points",rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){convert(*m);});
 }
 void convert(const sensor_msgs::msg::PointCloud2 &m){
  auto stamp=rclcpp::Time(m.header.stamp,get_clock()->get_clock_type());const double age=(now()-stamp).seconds();
  if(age<-.02 || age>maxAge)return;
  if(stamp<previous)previous=rclcpp::Time(0,0,get_clock()->get_clock_type());
  if(stamp<=previous)return;
  previous=stamp;
  std::array<int,3> offsets{{-1,-1,-1}};
  for(const auto &f:m.fields)for(size_t i=0;i<3;++i)if(f.name==std::array<std::string,3>{{"x","y","z"}}[i] && f.datatype==7 && f.count==1)offsets[i]=f.offset;
  const uint16_t endian=1;const bool hostBig=*reinterpret_cast<const uint8_t*>(&endian)==0;
  if(m.is_bigendian!=hostBig || !m.point_step || m.row_step<uint64_t(m.width)*m.point_step || m.data.size()<uint64_t(m.row_step)*m.height)return;
  for(int o:offsets)if(o<0 || uint32_t(o+4)>m.point_step)return;
  sensor_msgs::msg::LaserScan s;s.header=m.header;s.header.frame_id="team04/lidar_link";
  s.angle_min=-M_PI;s.angle_max=M_PI;s.angle_increment=2*M_PI/719;s.scan_time=1/rate;s.time_increment=0;
  s.range_min=.05;s.range_max=12;s.ranges.assign(720,std::numeric_limits<float>::infinity());
  for(size_t row=0;row<m.height;++row)for(size_t col=0;col<m.width;++col){
   const auto p=&m.data[row*m.row_step+col*m.point_step];float xyz[3];for(int i=0;i<3;++i)std::memcpy(&xyz[i],p+offsets[i],4);
   if(!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]) || xyz[2]<zmin || xyz[2]>zmax)continue;
   // Remove returns physically on our own chassis/camera. Raw cloud stays intact.
   const double x=std::abs(xyz[0]),y=std::abs(xyz[1]),z=xyz[2]+.13;
   const bool ownBody=(z>=.024 && z<=.096 && ((x<=.121 && y<=.101) || (x<=.231 && y<=.046)));
   const bool ownCamera=(xyz[0]>=.185 && xyz[0]<=.211 && y<=.014 && z>=.106 && z<=.134);
   const bool ownBumper=x>=.239 && x<=.261 && y<=.101 && z>=.024 && z<=.046;
   const bool ownCameraMount=xyz[0]>=.188 && xyz[0]<=.208 && y<=.010 && z>=.092 && z<=.108;
   if(ownBody || ownCamera || ownBumper || ownCameraMount)continue;
   const double r=std::hypot(xyz[0],xyz[1]);if(r<s.range_min || r>s.range_max)continue;
   const int bin=std::clamp(int(std::lround((std::atan2(xyz[1],xyz[0])-s.angle_min)/s.angle_increment)),0,719);
   s.ranges[bin]=std::min(s.ranges[bin],float(r));
  }
  pub->publish(s);
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Scan>());rclcpp::shutdown();}
