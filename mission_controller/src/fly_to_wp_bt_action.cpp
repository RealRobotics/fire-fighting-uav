// Copyright 2023 University of Leeds.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "mission_controller/fly_to_wp_bt_action.h"

#include "uav_msgs/FlyToWPAction.h"
#include "uav_msgs/GpsLocationWithPrecision.h"

/**
 * This method is constructor.
 * BehaviourCpp library already provides ROS wrapper for ROS action. 
 * So this class just inherits from there.
 * @param handle -  reference to ros nodehandle
 * @param name - reference to name for the BT node
 * @param conf - reference to BT::node configuration
*/
FlyToWpBTAction::FlyToWpBTAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf)
: BT::RosActionNode<uav_msgs::FlyToWPAction>(handle, name, conf){}

/**
 * This method defines what ports this BT node has.
 * For this class we need the waypoint where the drone should fly
*/
BT::PortsList FlyToWpBTAction::providedPorts() {
  return {BT::InputPort<uav_msgs::GpsLocationWithPrecision>("waypoint")};  
}

/**
 * This method gets callled automatically when the BT node is ticked.
 * It reads the input port and sets it as the goal for ROS action
 * @param goal - this is automatically filled based on what action is being wrapped
*/
bool FlyToWpBTAction::sendGoal(GoalType& goal){
  if(!getInput<uav_msgs::GpsLocationWithPrecision>("waypoint", goal.goal))
  {
    ROS_INFO("no input");
    // abort the entire action. Result in a FAILURE
    return false;
  }
  ROS_INFO("Sending goal request with latitude %f and longitude %f", goal.goal.location.latitude, goal.goal.location.longitude);
  return true;
}

/**
 * This method is called automatically by BT.
 * If BT node is running, it can be halted.
 * In this case, we use method provided by the library.
 * It should lead to ROS action being cancelled.
*/
void FlyToWpBTAction::halt(){
  if( status() == BT::NodeStatus::RUNNING ) {
    ROS_WARN("Fly to wp action is being halted");
    BaseClass::halt();
  }
}

/**
 * After ROS action returns result, the BT node is done.
 * This method gets called automatically by BT
*/
BT::NodeStatus FlyToWpBTAction::onResult( const ResultType& res){
  return BT::NodeStatus::SUCCESS;
}

/**
 * If the action server refuse the action, this method gets called automatically.
*/
BT::NodeStatus FlyToWpBTAction::onFailedRequest(FailureCause failure){
  ROS_ERROR("Fly to wp action request failed %d", static_cast<int>(failure));
  return BT::NodeStatus::FAILURE;
}