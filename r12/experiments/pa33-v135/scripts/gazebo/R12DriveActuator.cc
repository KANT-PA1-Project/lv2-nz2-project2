// Force-only actuator model. Wheel targets expire independently inside Gazebo.
// Position / velocity components are refreshed each physics update (encoders).
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/transport/Node.hh>

extern "C" double r12_dc_effort(double target, double position,
                               double velocity, int steering) {
  const double speed = steering ? 7.330382858 : 29.321531434;
  const double stall = steering ? 2.7 : .54;
  const double continuous = steering ? .54 : .108;
  const double demand = steering ? 5*(target-position)-.2*velocity
                                : .02*(target-velocity);
  const double upper = std::min(continuous, stall*(1-velocity/speed));
  const double lower = std::max(-continuous, stall*(-1-velocity/speed));
  return std::min(upper, std::max(lower, demand));
}

namespace r12 {
class DriveActuator : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate {
  std::array<gz::sim::Entity,8> entities{};
  std::array<double,8> targets{};
  std::chrono::steady_clock::time_point lastCommand{};
  std::mutex mutex;
  gz::transport::Node node;
  gz::transport::Node::Publisher telemetry;
  bool initialized=false;
  std::uint64_t step=0;

 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    const std::array<std::string,4> modules={"lh1","rh1","lh2","rh2"};
    for (int i=0;i<8;++i) {
      entities[i]=model.JointByName(ecm,modules[i%4]+(i<4?"_steer_joint":"_wheel_joint"));
      if (entities[i]==gz::sim::kNullEntity) throw std::runtime_error("Missing R12 joint");
      gz::sim::Joint joint(entities[i]);
      joint.EnablePositionCheck(ecm); joint.EnableVelocityCheck(ecm);
    }
    if (!node.Subscribe("/r12/gz/drive_targets", &DriveActuator::Command, this))
      throw std::runtime_error("Cannot subscribe R12 actuator command");
    telemetry=node.Advertise<gz::msgs::Double_V>("/r12/gz/actuator_feedback");
  }

  void Command(const gz::msgs::Double_V &msg) {
    if (msg.data_size()!=8) return;
    for (int i=0;i<8;++i) if (!std::isfinite(msg.data(i))) return;
    std::lock_guard<std::mutex> guard(mutex);
    for(int i=0;i<8;++i) {
      const double limit=i<4?1.5707963267948966:29.321531434;
      targets[i]=std::clamp(msg.data(i),-limit,limit);
    }
    lastCommand=std::chrono::steady_clock::now();
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    std::array<double,8> q{},v{},t{},effort{};
    bool fresh;
    {
      std::lock_guard<std::mutex> guard(mutex);
      t=targets;
      fresh=std::chrono::duration<double>(std::chrono::steady_clock::now()-lastCommand).count()<.5;
    }
    bool valid=true;
    for(int i=0;i<8;++i) {
      gz::sim::Joint joint(entities[i]);
      auto p=joint.Position(ecm); auto velocity=joint.Velocity(ecm);
      if (!p || p->empty() || !velocity || velocity->empty() ||
          !std::isfinite((*p)[0]) || !std::isfinite((*velocity)[0])) {valid=false;continue;}
      q[i]=(*p)[0];v[i]=(*velocity)[0];
    }
    if (!initialized && valid) {
      std::lock_guard<std::mutex> guard(mutex);
      for(int i=0;i<4;++i) targets[i]=t[i]=q[i];
      initialized=true;
    }
    for(int i=0;i<8;++i) {
      // A missing command means zero wheel speed with bounded active braking.
      if (i>=4 && !fresh) t[i]=0;
      effort[i]=valid?r12_dc_effort(t[i],q[i],v[i],i<4):0;
      gz::sim::Joint(entities[i]).SetForce(ecm,{effort[i]});
    }
    if (++step%8==0) {
      gz::msgs::Double_V msg;
      msg.add_data(fresh && valid ? 1:0);
      for(double x:q)msg.add_data(x);
      for(double x:v)msg.add_data(x);
      for(double x:effort)msg.add_data(x);
      telemetry.Publish(msg);
    }
  }
};
}
GZ_ADD_PLUGIN(r12::DriveActuator, gz::sim::System,
              r12::DriveActuator::ISystemConfigure,
              r12::DriveActuator::ISystemPreUpdate)
