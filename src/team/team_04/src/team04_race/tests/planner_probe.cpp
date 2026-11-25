#include "team04_race/lidar_planner.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
int main(int argc,char**argv){if(argc!=2)return 2;std::ifstream in(argv[1]);std::vector<team04::Point> points;team04::Point p;while(in>>p.x>>p.y>>p.z)points.push_back(p);team04::PlannerConfig c;const auto out=team04::planLidar(points,c,0);std::cout<<std::setprecision(8)<<"{\"valid\":"<<(out.valid?"true":"false")<<",\"reason\":\""<<out.reason<<"\",\"ground\":["<<out.ground_a<<","<<out.ground_b<<","<<out.ground_c<<"],\"ground_support\":"<<out.ground_support<<",\"obstacles\":"<<out.obstacles.size()<<",\"width\":"<<out.width<<",\"steering\":"<<out.steering<<",\"speed\":"<<out.speed<<",\"path\":[";for(size_t i=0;i<out.path.size();++i){if(i)std::cout<<",";std::cout<<"["<<out.path[i].x<<","<<out.path[i].y<<"]";}std::cout<<"]}"<<std::endl;}
