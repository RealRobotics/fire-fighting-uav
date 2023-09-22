#include "mission_controller/is_battery_required_status.h"

#include <memory>

IsBatteryRequiredStatus::IsBatteryRequiredStatus(const std::string & instance_name, const BT::NodeConfiguration & conf,
              ros::NodeHandle& nh)
  : BT::ConditionNode(instance_name, conf), nh_ (nh) {
  battery_status_sub_ = nh_.subscribe<uav_msgs::BatteryStatus>("battery_monitor/battery_status", 10,
                                                                            &IsBatteryRequiredStatus::batteryStatusCallback_, this);
}


BT::NodeStatus IsBatteryRequiredStatus::tick() {
  getInput<int>("required_status", required_status_);
  status_mtx_.lock();
  if (current_status_ == uav_msgs::BatteryStatus::UNSET) {
    status_mtx_.unlock();
    return BT::NodeStatus::FAILURE;
  }
  if (current_status_ == required_status_) {
    status_mtx_.unlock();
    return BT::NodeStatus::SUCCESS;
  } else {
    status_mtx_.unlock();
    return BT::NodeStatus::FAILURE;
  }
}

void IsBatteryRequiredStatus::batteryStatusCallback_(const uav_msgs::BatteryStatus::ConstPtr& message) {
  status_mtx_.lock();
  current_status_ = message->status;
  status_mtx_.unlock();
}

BT::PortsList IsBatteryRequiredStatus:: providedPorts() {
  return {BT::InputPort<uint8_t>("required_status")};
}

/// Method to register the service into a factory.
void IsBatteryRequiredStatus::Register(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle)
{
  BT::NodeBuilder builder = [&node_handle](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<IsBatteryRequiredStatus>(name, config,node_handle);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<IsBatteryRequiredStatus>();
  manifest.ports = IsBatteryRequiredStatus::providedPorts();
  manifest.registration_ID = registration_ID;
  factory.registerBuilder( manifest, builder );
}