#include "mission_controller/special_movement_bt_action.h"

#include "uav_msgs/SpecialMovementAction.h"
#include "uav_msgs/SpecialMovement.h"

// SpecialMovementBTAction::SpecialMovementBTAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf)
// : BT::RosActionNode<uav_msgs::SpecialMovementAction>(handle, name, conf){}

SpecialMovementBTAction::SpecialMovementBTAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf)
    : BT::RosActionNode<uav_msgs::SpecialMovementAction>(handle, name, conf), nh_(handle)
{
    battery_status_sub_ = nh_.subscribe<uav_msgs::BatteryStatus>(
        "battery_monitor/battery_status", 10, &SpecialMovementBTAction::batteryStatusCallback_, this);
}

BT::PortsList SpecialMovementBTAction::providedPorts() {
  return {BT::InputPort<uint8_t>("type_of_action")};  
}

bool SpecialMovementBTAction::sendGoal(GoalType& goal)
{
    
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
  // setStatus(BT::NodeStatus::FAILURE);
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

void SpecialMovementBTAction::batteryStatusCallback_(const uav_msgs::BatteryStatus::ConstPtr& message) 
{
  status_mtx_.lock();
  current_status_ = message->status;
  status_mtx_.unlock();
  if (status() == BT::NodeStatus::RUNNING && current_status_ == required_status_ && !statementExecuted) 
  {
            SpecialMovementBTAction::halt();
            ros::Duration(2.0).sleep();
            statementExecuted = true;
  }
}