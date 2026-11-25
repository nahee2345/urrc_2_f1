// Validation-only system: requests existing physics contact data; no forces/poses/commands.
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/msgs/float_v.pb.h>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <vector>
class HanlaContactTelemetry:public gz::sim::System,public gz::sim::ISystemConfigure,
 public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate {
 gz::sim::Entity model;std::vector<gz::sim::Entity> collisions;
 gz::transport::Node node;gz::transport::Node::Publisher pub,jointPub;
 std::vector<gz::sim::Entity> joints;double lastJoint=-1;
 public:void Configure(const gz::sim::Entity &e,const std::shared_ptr<const sdf::Element>&,
 gz::sim::EntityComponentManager&,gz::sim::EventManager&) override{
  model=e;pub=node.Advertise<gz::msgs::Contacts>("/team_10/contacts_10/all");
  jointPub=node.Advertise<gz::msgs::Float_V>("/team_10/steering_10/measured");
 }
 public:void PreUpdate(const gz::sim::UpdateInfo&,gz::sim::EntityComponentManager &ecm) override{
  if(joints.empty())ecm.Each<gz::sim::components::Joint,gz::sim::components::Name>([&](const gz::sim::Entity &e,const gz::sim::components::Joint*,const gz::sim::components::Name *name){
   auto p=ecm.Component<gz::sim::components::ParentEntity>(e);
   if(p&&p->Data()==model&&(name->Data()=="front_left_steering_joint"||name->Data()=="front_right_steering_joint"))joints.push_back(e);return true;});
  if(!collisions.empty())return;
  ecm.Each<gz::sim::components::Collision>([&](const gz::sim::Entity &e,const gz::sim::components::Collision*){
   auto parent=e;bool belongs=false;
   for(int i=0;i<12;++i){auto p=ecm.Component<gz::sim::components::ParentEntity>(parent);if(!p)break;parent=p->Data();if(parent==model){belongs=true;break;}}
   if(belongs){collisions.push_back(e);if(!ecm.Component<gz::sim::components::ContactSensorData>(e))ecm.CreateComponent(e,gz::sim::components::ContactSensorData());}
   return true;
  });
 }
 public:void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override{
  if(info.paused||!pub.HasConnections())return;
  gz::msgs::Contacts msg;auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
  msg.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);msg.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
  for(auto e:collisions){auto data=ecm.Component<gz::sim::components::ContactSensorData>(e);if(data)for(const auto &c:data->Data().contact())msg.add_contact()->CopyFrom(c);}
  // Publish empty messages too: absence of collision is measurable, not inferred
  // from a disconnected topic. Reader keeps per-second stream heartbeat.
  pub.Publish(msg);
  double t=ns*1e-9;if(t-lastJoint>=.02){gz::msgs::Float_V v;v.mutable_header()->CopyFrom(msg.header());for(auto e:joints){auto p=ecm.Component<gz::sim::components::JointPosition>(e);if(p&&!p->Data().empty())v.add_data(p->Data()[0]);}jointPub.Publish(v);lastJoint=t;}
 }
};
GZ_ADD_PLUGIN(HanlaContactTelemetry,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(HanlaContactTelemetry,"team10::ContactTelemetry")
