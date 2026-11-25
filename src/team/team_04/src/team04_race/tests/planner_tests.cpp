#include "team04_race/lidar_planner.hpp"
#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>
std::vector<team04::Point> scene(double offset=0,double curve=0,double width=2.4,bool right=true){
 std::vector<team04::Point> p;
 for(double x=-2.6;x<=2.6;x+=.1)for(double y=-1;y<=1;y+=.1)if(std::hypot(x,y)>.35)p.push_back({x,y,-.13+.01*x});
 for(double x=-2;x<10;x+=.010)for(double z=.04;z<.31;z+=.04){p.push_back({x,offset+curve*x*x+width/2,z-.13+.01*x});if(right)p.push_back({x,offset+curve*x*x-width/2,z-.13+.01*x});}
 return p;
}
void require(bool test,const char*name){if(!test)throw std::runtime_error(name);std::cout<<"PASS "<<name<<'\n';}
int main(){
 team04::PlannerConfig cfg;
 auto straight=team04::planLidar(scene(),cfg,0);require(straight.valid && std::abs(straight.steering)<.01,"centred straight corridor");
 require(std::abs(straight.ground_a-.01)<.002 && std::abs(straight.ground_c+.13)<.002,"sloping road fitted from points");
 auto left=team04::planLidar(scene(.5),cfg,0);require(left.valid && left.steering<-.1,"offset centre requires official negative left");
 auto right=team04::planLidar(scene(-.5),cfg,0);require(right.valid && right.steering>.1,"offset centre requires official positive right");
 require(!team04::planLidar(scene(0,0,.45),cfg,0).valid,"too narrow corridor stops");
 require(!team04::planLidar(scene(0,0,2.4,false),cfg,0).valid,"one missing boundary stops");
 require(!team04::planLidar({},cfg,0).valid,"empty cloud stops");
 auto blocked=scene();for(double y=-.3;y<=.3;y+=.02)for(double x=.8;x<=1.1;x+=.04)blocked.push_back({x,y,-.09+.01*x});
 require(!team04::planLidar(blocked,cfg,0).valid,"40mm obstacle below legacy slice stops");
 auto invalid=scene();invalid.push_back({std::numeric_limits<double>::quiet_NaN(),0,0});invalid.push_back({0,std::numeric_limits<double>::infinity(),0});require(team04::planLidar(invalid,cfg,0).valid,"invalid points excluded without losing genuine corridor");
 cfg.max_speed=1.5;auto bend=team04::planLidar(scene(0,.10),cfg,.5);auto fast=team04::planLidar(scene(),cfg,.5);
 require(bend.valid && bend.steering<0 && bend.curvature>.1,"curved corridor produces left path");
 require(bend.speed<fast.speed,"future curvature reduces speed");
 require(std::abs(bend.steering)<35*M_PI/180,"steering mechanical limit respected");
}
