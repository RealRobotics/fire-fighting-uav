#include "mission_controller/is_battery_ok.h"

#include <memory>

IsBatteryOk::IsBatteryOk(const std::string & instance_name, const BT::NodeConfiguration & conf,
              ros::NodeHandle& nh)
  : BT::ConditionNode(instance_name, conf), nh_ (nh) {
  battery_status_sub_ = nh_.subscribe<uav_msgs::BatteryStatus>("battery_monitor/battery_status", 10,
                                                                            &IsBatteryOk::batteryStatusCallback_, this);
  getInput<int>("required_status", required_status_);
}


BT::NodeStatus IsBatteryOk::tick() {
  status_mtx_.lock();
  if (current_status_ == required_status_) {
    status_mtx_.unlock();
    return BT::NodeStatus::SUCCESS;
  } else {
    status_mtx_.unlock();
    return BT::NodeStatus::FAILURE;
  }
}

void IsBatteryOk::batteryStatusCallback_(const uav_msgs::BatteryStatus::ConstPtr& message) {
  status_mtx_.lock();
  current_status_ = message->status;
  status_mtx_.unlock();
}

BT::PortsList IsBatteryOk:: providedPorts() {
  return {BT::InputPort<uint8_t>("required_status")};
}

/// Method to register the service into a factory.
void IsBatteryOk::Register(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle)
{
  BT::NodeBuilder builder = [&node_handle](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<IsBatteryOk>(name, config,node_handle);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<IsBatteryOk>();
  manifest.ports = IsBatteryOk::providedPorts();
  manifest.registration_ID = registration_ID;
  factory.registerBuilder( manifest, builder );
}