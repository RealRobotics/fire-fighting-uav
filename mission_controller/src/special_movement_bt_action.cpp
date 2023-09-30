#include "mission_controller/special_movement_bt_action.h"

#include "uav_msgs/SpecialMovementAction.h"
#include "uav_msgs/SpecialMovement.h"

SpecialMovementBTAction::SpecialMovementBTAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf)
: BT::RosActionNode<uav_msgs::SpecialMovementAction>(handle, name, conf){}

BT::PortsList SpecialMovementBTAction::providedPorts() {
  return {BT::InputPort<uint8_t>("type_of_action")};  
}

bool SpecialMovementBTAction::sendGoal(GoalType& goal){
  if(!getInput<uint8_t>("type_of_action", goal.movement.data))
  {
    // abort the entire action. Result in a FAILURE
    return false;
  }
  ROS_INFO("Sending goal request: %d", goal.movement.data);
  return true;
}

void SpecialMovementBTAction::halt(){
  if( status() == BT::NodeStatus::RUNNING ) {
    ROS_WARN("Special movement action is being halted");
    BaseClass::halt();
  }
}

BT::NodeStatus SpecialMovementBTAction::onResult( const ResultType& res){
  uint8_t type_of_action;

  getInput<uint8_t>("type_of_action", type_of_action);
  ROS_INFO("Special movement action %d has succeeded", type_of_action);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SpecialMovementBTAction::onFailedRequest(FailureCause failure){
  ROS_ERROR("Special movement action request failed %d", static_cast<int>(failure));
  return BT::NodeStatus::FAILURE;
}