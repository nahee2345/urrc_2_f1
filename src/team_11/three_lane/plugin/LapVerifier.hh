#pragma once
// Independent practice-only physical pose lap validator.
// Lap logs require forward physical movement, continuous in-track samples,
// and ordered 10m checkpoints; commanded distance alone never earns a lap.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>
#include <utility>

namespace urrc::lapcheck {
struct Point { double s, x, y, z; };
struct Result {
  bool lap{false};
  bool invalid{false};
  int completed{0};
  double progress{0.};
  double nearestRoadM{0.};
  const char *reason{"OK"};
};
class Verifier {
 public:
  void Configure(std::vector<Point> points, double lapLength,
                 double allowedCenterlineOffsetM) {
    points_ = std::move(points);
    length_ = lapLength;
    allowed_ = allowedCenterlineOffsetM;
    initialized_ = false;
    invalid_ = false;
    completed_ = 0;
    progress_ = 0.;
    lastCheckpoint_ = 0;
    nearest_ = 0;
  }
  Result Observe(double x,double y,double z,double dt,double permittedSpeed) {
    Result result;
    result.completed=completed_;
    result.progress=progress_;
    if(invalid_) { result.invalid=true; result.reason=reason_; return result; }
    if(points_.size()<10 || length_<=100. || dt<=0. ||
       !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return Fail("BAD_SAMPLE");
    }
    double best=std::numeric_limits<double>::infinity();
    double bestS=0.,bestZ=0.;
    std::size_t bestIndex=0;
    const std::size_t n=points_.size();
    auto check=[&](std::size_t i) {
      const auto &a=points_[i], &b=points_[(i+1)%n];
      const double dx=b.x-a.x,dy=b.y-a.y;
      const double len2=dx*dx+dy*dy;
      if(len2<1e-12)return;
      const double t=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/len2,0.,1.);
      const double sq=std::pow(x-a.x-t*dx,2)+std::pow(y-a.y-t*dy,2);
      if(sq<best) {
        best=sq;bestIndex=i;
        const double end=(i+1==n)?length_:b.s;
        bestS=std::fmod(a.s+t*std::max(0.,end-a.s),length_);
        bestZ=a.z+t*(b.z-a.z);
      }
    };
    if(initialized_) {
      for(int offset=-40;offset<=40;offset++)
        check((nearest_+n+static_cast<std::size_t>((offset+static_cast<int>(n))%static_cast<int>(n)))%n);
    } else {
      for(std::size_t i=0;i<n;i++)check(i);
    }
    if(!std::isfinite(best))return Fail("NO_PROJECTION");
    nearest_=bestIndex;
    result.nearestRoadM=std::sqrt(best);
    if(result.nearestRoadM>allowed_ || std::abs(z-bestZ)>.45)
      return Fail("OFF_TRACK");
    if(!initialized_) { initialized_=true;lastS_=bestS;return result; }
    double ds=bestS-lastS_;
    if(ds< -length_/2.)ds+=length_;
    if(ds> length_/2.)ds-=length_;
    const double stepLimit=std::max(1.0,2.*std::max(0.,permittedSpeed)*dt+.50);
    if(ds< -.50)return Fail("REVERSE_TRAVEL");
    if(ds>stepLimit)return Fail("POSE_JUMP");
    lastS_=bestS;
    if(ds>0.)progress_+=ds;
    const int currentCheckpoint=static_cast<int>(progress_/10.);
    if(currentCheckpoint>lastCheckpoint_+1)return Fail("SKIPPED_CHECKPOINT");
    lastCheckpoint_=std::max(lastCheckpoint_,currentCheckpoint);
    if(progress_>=length_*static_cast<double>(completed_+1)) {
      ++completed_;
      result.lap=true;
    }
    result.completed=completed_;
    result.progress=progress_;
    return result;
  }
  bool invalid()const{return invalid_;}
  double progress()const{return progress_;}
  const char * reason()const{return reason_;}
 private:
  Result Fail(const char *why) {
    invalid_=true;reason_=why;
    Result result;result.invalid=true;result.reason=why;
    result.completed=completed_;result.progress=progress_;return result;
  }
  std::vector<Point> points_;
  double length_{0.},allowed_{0.};
  double lastS_{0.},progress_{0.};
  std::size_t nearest_{0};
  int completed_{0},lastCheckpoint_{0};
  bool initialized_{false},invalid_{false};
  const char *reason_{"OK"};
};
} // namespace urrc::lapcheck
