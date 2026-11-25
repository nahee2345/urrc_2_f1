// Offline raw-cloud diagnostic, never a control input.
#include "team04_race/traffic.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
int main(int argc,char**argv){if(argc!=2)return 2;std::ifstream in(argv[1]);std::vector<team04::Point> points;team04::Point p;while(in>>p.x>>p.y>>p.z)points.push_back(p);team04::PlannerConfig c;c.path_mode="racing_line";c.max_speed=1;c.planning_horizon=10;c.braking=2.5;auto plan=team04::planLidar(points,c,.7);std::cerr<<plan.reason<<" ground "<<plan.ground_a<<" "<<plan.ground_b<<" "<<plan.ground_c<<" pairs "<<plan.left.size()<<"\n";auto ds=team04::detectVehicles(plan,1);std::cerr<<"detections "<<ds.size()<<"\n";std::cout<<std::setprecision(10)<<"kind,x,y,z\n";for(auto [name,ps]:{std::pair<const char*,std::vector<team04::Point>>("obstacle",plan.obstacles),{"left",plan.left},{"right",plan.right}})for(auto p:ps)std::cout<<name<<","<<p.x<<","<<p.y<<","<<p.z<<"\n";for(auto d:ds)std::cout<<"detection,"<<d.x<<","<<d.y<<",0\n";}
