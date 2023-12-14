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
#include "uav_msgs/WayPtSearch.h"
#include "uav_msgs/SearchWPAction.h"
#include "ros/ros.h"
#include "uav_msgs/BatteryStatus.h"

// FlyToWpBTAction::FlyToWpBTAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf)
//     : BT::RosActionNode<uav_msgs::SearchWPAction>(handle, name, conf) {}

FlyToWpBTAction::FlyToWpBTAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf)
    : BT::RosActionNode<uav_msgs::SearchWPAction>(handle, name, conf), nh_(handle)
{
    battery_status_sub_ = nh_.subscribe<uav_msgs::BatteryStatus>(
        "battery_monitor/battery_status", 10, &FlyToWpBTAction::batteryStatusCallback_, this);
}


BT::PortsList FlyToWpBTAction::providedPorts()
{
    return {BT::InputPort<uint8_t>("type_of_action")};
}

bool FlyToWpBTAction::sendGoal(GoalType& goal)
{
  if(!getInput<uint8_t>("type_of_action", goal.goal.status))
  {
    // abort the entire action. Result in a FAILURE
    return false;
  }
  ROS_INFO("Sending goal request with WSP1 %d", goal.goal.WSP1);
  //setStatus(BT::NodeStatus::RUNNING);
  return true;
}

void FlyToWpBTAction::halt()
{
    if (status() == BT::NodeStatus::RUNNING)
    {
        ROS_WARN("Fly to wp action is being Cancled");
        BaseClass::halt();
        // action_client_->cancelGoal();
         setStatus(BT::NodeStatus::FAILURE);
    }
}

BT::NodeStatus FlyToWpBTAction::onResult(const ResultType& res)
{
    uint8_t type_of_action;
    getInput<uint8_t>("type_of_action", type_of_action);
    ROS_INFO("Fly to waypoints action %d has succeeded", type_of_action);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FlyToWpBTAction::onFailedRequest(FailureCause failure)
{
    ROS_INFO("Fly to wp action request failed %d", static_cast<int>(failure));
    if (failure != ABORTED_BY_SERVER) {
        FlyToWpBTAction::halt();
    }
    return BT::NodeStatus::FAILURE;
}
void FlyToWpBTAction::batteryStatusCallback_(const uav_msgs::BatteryStatus::ConstPtr& message) 
{
  status_mtx_.lock();
  current_status_ = message->status;
  status_mtx_.unlock();
  if (status() == BT::NodeStatus::RUNNING && current_status_ != required_status_)
  {
    FlyToWpBTAction::halt();
     ros::Duration(5.0).sleep();
  }
}
