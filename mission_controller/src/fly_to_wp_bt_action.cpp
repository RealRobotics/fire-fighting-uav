#include "mission_controller/fly_to_wp_bt_action.h"

#include "uav_msgs/FlyToWPAction.h"
#include <sensor_msgs/NavSatFix.h>

FlyToWpAction::FlyToWpAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf)
: BT::RosActionNode<uav_msgs::FlyToWPAction>(handle, name, conf){}

BT::PortsList FlyToWpAction::providedPorts() {
  //return {InputPort<sensor_msgs::NavSatFix>("waypoint")};
  return {BT::InputPort<double>("waypoint")};  
}

bool FlyToWpAction::sendGoal(GoalType& goal){
  //if(!getInput<sensor_msgs::NavSatFix>("waypoint", goal.current_location))
  if(!getInput<double>("waypoint", goal.goal_location.latitude))
  {
    // abort the entire action. Result in a FAILURE
    return false;
  }
  ROS_INFO("Sending goal request");
  return true;
}

void FlyToWpAction::halt(){
  if( status() == BT::NodeStatus::RUNNING ) {
    ROS_WARN("Fly to wp action is being halted");
    BaseClass::halt();
  }
}

BT::NodeStatus FlyToWpAction::onResult( const ResultType& res){
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FlyToWpAction::onFailedRequest(FailureCause failure){
  ROS_ERROR("Fly to wp action request failed %d", static_cast<int>(failure));
  return BT::NodeStatus::FAILURE;
}