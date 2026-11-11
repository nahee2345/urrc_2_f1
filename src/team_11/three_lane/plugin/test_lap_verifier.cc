#include "LapVerifier.hh"
#include <cassert>
#include <cmath>
#include <vector>
#include <iostream>

int main() {
  constexpr double pi=3.141592653589793;
  constexpr double radius=50.;
  constexpr int n=1000;
  std::vector<urrc::lapcheck::Point> route;
  for(int i=0;i<n;i++) {
    double a=2*pi*i/n;
    route.push_back({radius*a,radius*std::cos(a),radius*std::sin(a),0.});
  }
  urrc::lapcheck::Verifier good;
  good.Configure(route,2*pi*radius,1.0);
  bool lap=false;
  const double ds=.40,dt=.05;
  for(double s=0;s<2*pi*radius+1.;s+=ds){
    double a=s/radius;
    auto r=good.Observe(radius*std::cos(a),radius*std::sin(a),0.,dt,8.);
    assert(!r.invalid);
    if(r.lap)lap=true;
  }
  assert(lap);
  urrc::lapcheck::Verifier offroad;
  offroad.Configure(route,2*pi*radius,1.0);
  offroad.Observe(radius,0.,0.,dt,8.);
  auto bad=offroad.Observe(60.,0.,0.,dt,8.);
  assert(bad.invalid);
  assert(std::string(bad.reason)=="OFF_TRACK");
  urrc::lapcheck::Verifier teleport;
  teleport.Configure(route,2*pi*radius,1.0);
  teleport.Observe(radius,0.,0.,dt,8.);
  auto jump=teleport.Observe(radius*std::cos(.2),radius*std::sin(.2),0.,dt,8.);
  assert(jump.invalid);
  std::cout<<"PASS physical lap/off-track/teleport guards\n";
}
