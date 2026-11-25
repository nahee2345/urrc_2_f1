#include "team04_race/lidar_planner.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <cstdint>
#include <cmath>
int main(int argc,char**argv){
 if(argc<2)return 2;std::ifstream in(argv[1]);std::vector<team04::Point> points;team04::Point p;
 while(in>>p.x>>p.y>>p.z)points.push_back(p);
 int count=argc>2?std::stoi(argv[2]):200;team04::PlannerConfig cfg;cfg.max_speed=1.2;team04::Plan result;
 auto start=std::chrono::steady_clock::now();for(int i=0;i<count;++i)result=team04::planLidar(points,cfg,1.2);
 double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/count;
 uint64_t hash=1469598103934665603ULL;for(const auto&q:result.path)for(double x:{q.x,q.y,q.z}){hash^=uint64_t(std::llround(x*1e9));hash*=1099511628211ULL;}
 std::cout<<"{\"iterations\":"<<count<<",\"mean_ms\":"<<ms<<",\"valid\":"<<(result.valid?"true":"false")<<",\"path_hash_1nm\":\""<<hash<<"\",\"steering\":"<<result.steering<<",\"speed\":"<<result.speed<<"}"<<std::endl;
}
