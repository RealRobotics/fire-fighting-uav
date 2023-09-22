#include "mission_controller/fly_to_wp_bt_action.h"

#include "uav_msgs/FlyToWPAction.h"
#include <sensor_msgs/NavSatFix.h>

FlyToWpBTAction::FlyToWpBTAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf)
: BT::RosActionNode<uav_msgs::FlyToWPAction>(handle, name, conf){}

BT::PortsList FlyToWpBTAction::providedPorts() {
  //return {InputPort<sensor_msgs::NavSatFix>("waypoint")};
  return {BT::InputPort<double>("waypoint")};  
}

bool FlyToWpBTAction::sendGoal(GoalType& goal){
  //if(!getInput<sensor_msgs::NavSatFix>("waypoint", goal.current_location))
  if(!getInput<double>("waypoint", goal.goal_location.latitude))
  {
    // abort the entire action. Result in a FAILURE
    return false;
  }
  ROS_INFO("Sending goal request");
  return true;
}

void FlyToWpBTAction::halt(){
  if( status() == BT::NodeStatus::RUNNING ) {
    ROS_WARN("Fly to wp action is being halted");
    BaseClass::halt();
  }
}

BT::NodeStatus FlyToWpBTAction::onResult( const ResultType& res){
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FlyToWpBTAction::onFailedRequest(FailureCause failure){
  ROS_ERROR("Fly to wp action request failed %d", static_cast<int>(failure));
  return BT::NodeStatus::FAILURE;
}