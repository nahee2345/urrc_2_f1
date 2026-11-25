// TEST INSTRUMENT ONLY. Read actual contact data; no commands, forces or world changes.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/plugin/Register.hh>
#include <chrono>
namespace team04 {
class TrialObserver:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPostUpdate {
 std::vector<gz::sim::Entity> collisions;gz::transport::Node node;gz::transport::Node::Publisher pub;std::chrono::steady_clock::duration last{};gz::msgs::Contacts pending;
 public:void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&,gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override{
  for(auto l:gz::sim::Model(e).Links(ecm))for(auto c:gz::sim::Link(l).Collisions(ecm)){
   collisions.push_back(c);if(!ecm.Component<gz::sim::components::ContactSensorData>(c))ecm.CreateComponent(c,gz::sim::components::ContactSensorData());}
  pub=node.Advertise<gz::msgs::Contacts>("/team04/test/contacts");
 }
 void PostUpdate(const gz::sim::UpdateInfo&info,const gz::sim::EntityComponentManager&ecm)override{
  if(info.paused)return;
  if(info.simTime<last){pending.Clear();last=info.simTime;}
  // Inspect every physics step, then batch transport at 100Hz. A short contact
  // between publications must not disappear from the measurement.
  for(auto c:collisions){auto data=ecm.Component<gz::sim::components::ContactSensorData>(c);if(data)for(const auto&contact:data->Data().contact())pending.add_contact()->CopyFrom(contact);}
  if(info.simTime-last<std::chrono::milliseconds(10))return;
  last=info.simTime;
  const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();pending.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);pending.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
  pub.Publish(pending);pending.Clear();
 }
};
}
GZ_ADD_PLUGIN(team04::TrialObserver,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPostUpdate)
