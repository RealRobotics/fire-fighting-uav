#include "mission_controller/spray_bt_action.h"

#include "uav_msgs/SprayAction.h"
#include "uav_msgs/PumpStatus.h"

SprayBTAction::SprayBTAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf)
: BT::RosActionNode<uav_msgs::SprayAction>(handle, name, conf){}

BT::PortsList SprayBTAction::providedPorts() {
  return {BT::InputPort<uint8_t>("type_of_action")};  
}

bool SprayBTAction::sendGoal(GoalType& goal){
  if(!getInput<uint8_t>("type_of_action", goal.pump_control.status))
  {
    // abort the entire action. Result in a FAILURE
    return false;
  }
  ROS_INFO("Sending goal request: %d", goal.pump_control.status);
  return true;
}

void SprayBTAction::halt(){
  if( status() == BT::NodeStatus::RUNNING ) {
    ROS_WARN("Spray action is being halted");
    BaseClass::halt();
  }
}

BT::NodeStatus SprayBTAction::onResult( const ResultType& res){
  uint8_t type_of_action;

  getInput<uint8_t>("type_of_action", type_of_action);
  ROS_INFO("Spray action %d has succeeded", type_of_action);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SprayBTAction::onFailedRequest(FailureCause failure){
  ROS_ERROR("Spray action request failed %d", static_cast<int>(failure));
  return BT::NodeStatus::FAILURE;
}