#include "RearThreatHistory.hh"
#include <cassert>
#include <iostream>
using urrc::defense::ThreatHistory;
int main(){
  {
    ThreatHistory h;
    // One hit can never trigger defense.
    assert(!h.update(9,1.,4.8,0.,false));
    // Two decreasing imminent TTC frames are sufficient.
    assert(h.update(9,1.15,4.65,0.,false));
  }
  {
    ThreatHistory h;
    assert(!h.update(9,1.,4.9,0.,false));
    assert(!h.update(9,1.1,4.9,0.,false));
    assert(h.update(9,1.2,4.9,0.,false));
    assert(!h.weaving());
  }
  {
    ThreatHistory h;
    assert(!h.update(9,1.,9.0,0.,true));
    assert(!h.update(9,1.2,9.0,.1,true));
    assert(!h.update(9,1.4,9.0,.2,true));
    assert(h.weaving());
    assert(!h.update(9,1.5,4.9,.2,false));
    assert(h.update(9,1.65,4.7,.3,false));
    // Rear threat disappears: stale confirmations must be discarded.
    assert(!h.update(9,2.65,4.5,.3,false));
  }
  {
    ThreatHistory h;
    assert(!h.update(9,1.,4.9,0.,false));
    assert(!h.update(9,1.0,4.8,0.,false)); // duplicate timestamp
    assert(!h.update(10,1.1,4.5,0.,false)); // different opponent id
  }
  std::cout<<"PASS rear threat requires new LiDAR frames, sustained TTC,"
           <<" repeated weaving; never single-frame panic\n";
}
