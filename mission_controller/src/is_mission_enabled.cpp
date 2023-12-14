#include "mission_controller/is_mission_enabled.h"

#include <memory>

IsMissionEnabled::IsMissionEnabled(const std::string & instance_name, const BT::NodeConfiguration & conf, ros::NodeHandle& nh)
  : BT::ConditionNode(instance_name, conf), nh_(nh) {
}

BT::NodeStatus IsMissionEnabled::tick() 
{
  bool current_status {false};
  getInput<bool>("mission_enabled", current_status);
  //TODO I can pass mutex owned by mission_controller if I want to be sure
  if (current_status) {
    ROS_INFO("IsMissionEnabled: SUCCESS");
    return BT::NodeStatus::SUCCESS;
  } else {
    //ROS_INFO("IsMissionEnabled: FAILURE");
    //return BT::NodeStatus::FAILURE; disabled for debugging
    ROS_INFO("IsMissionEnabled: SUCCESS"); // added for debugging purpose
    return BT::NodeStatus::SUCCESS;        // added for debugging purpose
  }
}

BT::PortsList IsMissionEnabled:: providedPorts() {
  return {BT::InputPort<bool>("mission_enabled")};
}

/// Method to register the service into a factory.
void IsMissionEnabled::Register(BT::BehaviorTreeFactory& factory,
                        const std::string& registration_ID,
                        ros::NodeHandle& node_handle)
{
  BT::NodeBuilder builder = [&node_handle](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<IsMissionEnabled>(name, config, node_handle);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<IsMissionEnabled>();
  manifest.ports = IsMissionEnabled::providedPorts();
  manifest.registration_ID = registration_ID;
  factory.registerBuilder( manifest, builder );
}